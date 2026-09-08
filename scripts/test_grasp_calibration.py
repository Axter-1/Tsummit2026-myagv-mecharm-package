#!/usr/bin/env python3
"""Ensaya una calibracion sin mover la base ni actuar la pinza.

Recorre: home -> intermedio -> preagarre -> contacto -> preagarre ->
intermedio -> home. "contacto" es la ultima pose antes del cierre de la
pinza; este script no envia ninguna orden a la pinza. Las calibraciones
nuevas usan send_coords y las antiguas articulares sirven de respaldo.
"""

import argparse
import os
import sys

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from home_service_interfaces.action import MoveArm


WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_CALIBRATIONS = os.path.join(
    WORKSPACE_ROOT,
    "src",
    "home_service_behaviors",
    "config",
    "grasp_calibrations.yaml",
)
POSE_KEYS = (
    "intermediate_joint_angles",
    "pregrasp_joint_angles",
    "contact_joint_angles",
)
COORD_KEYS = (
    "intermediate_coords",
    "pregrasp_coords",
    "contact_coords",
)


def load_calibration(path, object_name, table_mm):
    if not os.path.isfile(path):
        raise RuntimeError(f"no existe el archivo de calibraciones: {path}")
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    calibration = (data.get("calibrations", {}) or {}).get(object_name)
    if not isinstance(calibration, dict):
        raise RuntimeError(f"no hay calibracion para '{object_name}'")

    if not any(key in calibration for key in POSE_KEYS):
        calibration = calibration.get(str(table_mm))
    if not isinstance(calibration, dict):
        raise RuntimeError(
            f"no hay calibracion de '{object_name}' para {table_mm} mm"
        )

    execution_mode = str(calibration.get("execution_mode", "coords")).lower()
    coords = []
    if execution_mode == "coords" and all(key in calibration for key in COORD_KEYS):
        for key in COORD_KEYS:
            pose = calibration.get(key)
            if not isinstance(pose, (list, tuple)) or len(pose) != 6:
                raise RuntimeError(f"calibracion invalida: falta {key}")
            coords.append([float(value) for value in pose])
        return "coords", coords

    poses = []
    for key in POSE_KEYS:
        angles = calibration.get(key)
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            raise RuntimeError(f"calibracion invalida: falta {key}")
        poses.append([float(value) for value in angles])
    return "angles", poses


class CalibrationTest(Node):

    def __init__(self):
        super().__init__("grasp_calibration_test")
        self.client = ActionClient(self, MoveArm, "/mecharm/move_arm")

    def move(self, label, angles=None, coords=None):
        if angles is not None and coords is not None:
            raise ValueError("move requiere angles o coords, exactamente uno")
        goal = MoveArm.Goal()
        if angles is None and coords is None:
            goal.pose_name = "home"
        elif angles is not None:
            goal.joint_angles = angles
        else:
            goal.coords = coords
            goal.move_mode = 1
        goal.speed_percent = 20.0

        self.get_logger().info(f"Moviendo: {label}")
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"objetivo rechazado en {label}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result()
        if response is None or not response.result.success:
            message = response.result.message if response is not None else "sin respuesta"
            raise RuntimeError(f"fallo en {label}: {message}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "object", choices=("engranaje", "poste", "rueda", "estrella")
    )
    parser.add_argument("table_mm", type=int)
    parser.add_argument("--calibrations", default=DEFAULT_CALIBRATIONS)
    args = parser.parse_args()

    calibration_mode, calibration_poses = load_calibration(
        args.calibrations, args.object, args.table_mm
    )
    intermediate, pregrasp, contact = calibration_poses
    rclpy.init()
    node = CalibrationTest()
    try:
        if not node.client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(
                "/mecharm/move_arm no esta disponible; inicia el driver"
            )
        node.move("home")
        if calibration_mode == "coords":
            poses = (
                ("intermedio", intermediate),
                ("preagarre", pregrasp),
                ("contacto (sin cerrar pinza)", contact),
                ("preagarre", pregrasp),
                ("intermedio", intermediate),
            )
            for label, coords in poses:
                node.move(label, coords=coords)
        else:
            poses = (
                ("intermedio", intermediate),
                ("preagarre", pregrasp),
                ("contacto (sin cerrar pinza)", contact),
                ("preagarre", pregrasp),
                ("intermedio", intermediate),
            )
            for label, angles in poses:
                node.move(label, angles=angles)
        node.move("home")
        print("Ensayo completado: no se movio la base ni la pinza.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
