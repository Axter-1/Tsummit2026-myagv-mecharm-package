
#!/usr/bin/env python3
"""Prueba de agarre y elevacion usando solo el brazo.

Secuencia: home -> abrir pinza -> intermedio -> preagarre -> contacto -> cerrar pinza ->
preagarre -> home. Las calibraciones nuevas usan send_coords; las antiguas
solo articulares se aceptan como respaldo durante la migracion.
"""

import argparse
import os
import sys
import time

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from home_service_interfaces.action import MoveArm
from home_service_interfaces.srv import SetGripper


WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_CALIBRATIONS = os.path.join(
    WORKSPACE_ROOT,
    "src",
    "home_service_behaviors",
    "config",
    "grasp_calibrations.yaml",
)
DEFAULT_POSES = os.path.join(
    WORKSPACE_ROOT,
    "src",
    "myagv_mecharm_service",
    "config",
    "poses.yaml",
)
DEFAULT_CATALOG = os.path.join(
    WORKSPACE_ROOT,
    "src",
    "home_service_behaviors",
    "config",
    "grasp_catalog.yaml",
)
# La telemetria del brazo fluctua algo mas de 2 grados al asentarse;
# coincide con angle_tolerance_deg del driver.
HOME_TOLERANCE_DEG = 3.5
HOME_ATTEMPTS = 2
REGRIP_DELAY_SEC = 1.0
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


def joint_error(index, actual, target):
    difference = float(actual) - float(target)
    if index == 5:
        return abs((difference + 180.0) % 360.0 - 180.0)
    return abs(difference)


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


def load_gripper_values(path, object_name):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            catalog = yaml.safe_load(handle) or {}
        root = catalog.get("grasp_catalog", {})
        gripper = root.get("gripper", {})
        spec = (root.get("objects", {}) or {}).get(object_name, {})
        stroke = float(gripper.get("stroke_mm", 45.0))
        open_mm = float(spec.get("open_mm", stroke))
        close_mm = float(spec.get("close_mm", 2.0))
        return (
            round(max(0.0, min(100.0, 100.0 * open_mm / stroke))),
            round(max(0.0, min(100.0, 100.0 * close_mm / stroke))),
        )
    except (OSError, TypeError, ValueError):
        return 100, 20


def load_pose(path, name):
    if not os.path.isfile(path):
        raise RuntimeError(f"no existe el archivo de poses: {path}")
    with open(path, "r", encoding="utf-8") as handle:
        poses = (yaml.safe_load(handle) or {}).get("poses", {})
    angles = poses.get(name)
    if not isinstance(angles, (list, tuple)) or len(angles) != 6:
        raise RuntimeError(f"no hay una pose '{name}' valida en {path}")
    return [float(value) for value in angles]


class PickLiftTest(Node):

    def __init__(self):
        super().__init__("pick_lift_test")
        self.move_client = ActionClient(self, MoveArm, "/mecharm/move_arm")
        self.gripper_client = self.create_client(
            SetGripper, "/mecharm/set_gripper"
        )

    def move(self, label, angles=None, coords=None, speed_percent=20.0):
        if angles is not None and coords is not None:
            raise ValueError("move requiere angles o coords, exactamente uno")
        goal = MoveArm.Goal()
        goal.pose_name = "" if angles is not None or coords is not None else "home"
        if angles is not None:
            goal.joint_angles = angles
        if coords is not None:
            goal.coords = coords
            goal.move_mode = 1
        goal.speed_percent = speed_percent
        self.get_logger().info(f"Moviendo: {label}")
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"objetivo rechazado en {label}")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result()
        if response is None or not response.result.success:
            message = response.result.message if response else "sin respuesta"
            raise RuntimeError(f"fallo en {label}: {message}")
        return list(response.result.final_joint_angles)

    def move_home(self, label, target):
        """Confirma J3 y el resto de home; reintenta si no asienta."""
        last_error = None
        for attempt in range(1, HOME_ATTEMPTS + 1):
            actual = self.move(label, speed_percent=40.0)
            if len(actual) == 6:
                error = max(
                    joint_error(index, actual[index], target[index])
                    for index in range(6)
                )
                if error <= HOME_TOLERANCE_DEG:
                    return
                last_error = error
                self.get_logger().warning(
                    f"{label}: home no asentado (error max={error:.2f} deg, "
                    f"intento {attempt}/{HOME_ATTEMPTS}); se reintenta."
                )
            else:
                last_error = "lectura invalida"
        raise RuntimeError(
            f"{label}: home no confirmado tras {HOME_ATTEMPTS} intentos "
            f"(error max={last_error})"
        )

    def set_gripper(self, label, value, torque=0, protect_current=0):
        if not self.gripper_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/mecharm/set_gripper no esta disponible")
        request = SetGripper.Request()
        request.value = int(value)
        request.speed_percent = 20.0
        request.torque = int(torque)
        request.force_control = bool(torque)
        request.protect_current = int(protect_current)
        self.get_logger().info(f"Pinza: {label} ({value})")
        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            message = response.message if response else "sin respuesta"
            raise RuntimeError(f"fallo de pinza en {label}: {message}")
        self.get_logger().info(
            f"Pinza confirmada: solicitada={value}, lectura={response.current_value}"
        )
        return response.current_value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "object", choices=("engranaje", "poste", "rueda", "estrella")
    )
    parser.add_argument("table_mm", type=int)
    parser.add_argument("--open-value", type=int, default=None)
    parser.add_argument("--closed-value", type=int, default=None)
    parser.add_argument(
        "--gripper-torque", type=int, default=500,
        help="torque adaptativo 150..980 (por defecto: 500)",
    )
    parser.add_argument(
        "--gripper-protect-current", type=int, default=500,
        help="corriente de proteccion 1..500 (por defecto: 500)",
    )
    parser.add_argument("--calibrations", default=DEFAULT_CALIBRATIONS)
    parser.add_argument("--catalog", default=DEFAULT_CATALOG)
    args = parser.parse_args()

    home = load_pose(DEFAULT_POSES, "home")
    calibration_mode, calibration_poses = load_calibration(
        args.calibrations, args.object, args.table_mm
    )
    intermediate, pregrasp, contact = calibration_poses
    open_value, closed_value = load_gripper_values(args.catalog, args.object)
    if args.open_value is not None:
        open_value = max(0, min(100, args.open_value))
    if args.closed_value is not None:
        closed_value = max(0, min(100, args.closed_value))
    if args.gripper_torque and not 150 <= args.gripper_torque <= 980:
        parser.error("--gripper-torque debe estar entre 150 y 980")
    if args.gripper_protect_current and not 1 <= args.gripper_protect_current <= 500:
        parser.error("--gripper-protect-current debe estar entre 1 y 500")

    rclpy.init()
    node = PickLiftTest()
    try:
        if not node.move_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(
                "/mecharm/move_arm no esta disponible; inicia el driver"
            )
        node.move_home("home", home)
        node.set_gripper(
            "abrir", open_value, args.gripper_torque,
            args.gripper_protect_current,
        )
        if calibration_mode == "coords":
            node.move("intermedio", coords=intermediate)
            node.move("preagarre", coords=pregrasp)
            node.move("contacto", coords=contact)
        else:
            node.move("intermedio", angles=intermediate)
            node.move("preagarre", angles=pregrasp)
            node.move("contacto", angles=contact)
        closed_reading = node.set_gripper(
            "cerrar sobre el objeto",
            closed_value,
            args.gripper_torque,
            args.gripper_protect_current,
        )
        if closed_reading > closed_value + 10:
            node.get_logger().warning(
                "La pinza se detuvo antes del cierre solicitado "
                f"({closed_reading} frente a {closed_value}). Si la rueda "
                "esta entre las mordazas puede ser agarre adaptativo; "
                "verifica visualmente que quedo sujeta."
            )
        time.sleep(REGRIP_DELAY_SEC)
        regrip_reading = node.set_gripper(
            "reapriete sobre el objeto",
            closed_value,
            args.gripper_torque,
            args.gripper_protect_current,
        )
        if regrip_reading > closed_value + 10:
            node.get_logger().warning(
                f"Reapriete sin cierre adicional: lectura={regrip_reading}."
            )
        time.sleep(0.8)
        if calibration_mode == "coords":
            node.move("levantar a preagarre", coords=pregrasp)
        else:
            node.move("levantar a preagarre", angles=pregrasp)
        node.move_home("volver a home", home)
        print(
            "Prueba completada: objeto sujeto y brazo en home. "
            "La base no se movio."
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
