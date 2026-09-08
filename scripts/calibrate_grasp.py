#!/usr/bin/env python3
"""Ensena las tres poses articulares de agarre de una pieza.

Guarda en config/grasp_calibrations.yaml, sin alterar grasp_catalog.yaml:
  home -> intermedio -> preagarre -> contacto -> home

Uso dentro del contenedor, con mecharm_driver_node detenido:
  python3 /workspace/scripts/calibrate_grasp.py poste
  python3 /workspace/scripts/calibrate_grasp.py engranaje
  python3 /workspace/scripts/calibrate_grasp.py rueda
"""

import argparse
import datetime as dt
import os
import sys
import tempfile
import time

import yaml

try:
    from pymycobot.mecharm270 import MechArm270
except ImportError:
    print("ERROR: falta pymycobot.")
    sys.exit(1)


DEFAULT_CALIBRATIONS = (
    "/workspace/src/home_service_behaviors/config/grasp_calibrations.yaml"
)
DEFAULT_POSES = "/workspace/src/myagv_mecharm_service/config/poses.yaml"
JOINT_MIN = [-160.0, -75.0, -175.0, -155.0, -115.0, -180.0]
JOINT_MAX = [160.0, 120.0, 65.0, 155.0, 115.0, 180.0]


def read_angles(arm, retries=12):
    for _ in range(retries):
        try:
            angles = arm.get_angles()
        except Exception:  # noqa: BLE001
            angles = None
        if isinstance(angles, (list, tuple)) and len(angles) == 6:
            return [round(float(value), 2) for value in angles]
        time.sleep(0.15)
    return None


def wait_arrival(arm, target, timeout=30.0):
    deadline = time.monotonic() + timeout
    stable = 0
    while time.monotonic() < deadline:
        current = read_angles(arm, retries=1)
        if current is not None:
            error = max(abs(current[i] - target[i]) for i in range(6))
            if error <= 3.0:
                stable += 1
                if stable == 3:
                    return True
            else:
                stable = 0
        time.sleep(0.15)
    return False


def validate(angles):
    for index, value in enumerate(angles):
        if value < JOINT_MIN[index] or value > JOINT_MAX[index]:
            raise ValueError(
                f"J{index + 1}={value:.2f} fuera de limites "
                f"[{JOINT_MIN[index]}, {JOINT_MAX[index]}]"
            )


def capture(arm, name):
    print(f"\n{name.upper()}: sujeta el brazo antes de liberarlo.")
    input("Pulsa ENTER para liberar servos y llevarlo manualmente a la pose... ")
    arm.release_all_servos()
    input("Colocalo, sujetalo firme y pulsa ENTER para fijar y leer los angulos... ")
    arm.power_on()
    time.sleep(0.5)
    angles = read_angles(arm)
    if angles is None:
        raise RuntimeError("no se pudieron leer los angulos tras fijar la pose")
    validate(angles)
    print(f"  {name}: {angles}")
    return angles


def load_home(path):
    with open(path, "r", encoding="utf-8") as handle:
        poses = (yaml.safe_load(handle) or {}).get("poses", {})
    home = poses.get("home")
    if not isinstance(home, (list, tuple)) or len(home) != 6:
        raise RuntimeError(f"no hay una pose home valida en {path}")
    return [float(value) for value in home]


def save_calibration(path, object_name, calibration):
    data = {"calibrations": {}}
    if os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or data
    calibrations = data.setdefault("calibrations", {})
    calibrations[object_name] = calibration
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".grasp_calibration_", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.safe_dump(data, handle, allow_unicode=False, sort_keys=False)
        os.replace(temporary, path)
    except Exception:
        os.unlink(temporary)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("object", choices=("engranaje", "poste", "rueda"))
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--calibrations", default=DEFAULT_CALIBRATIONS)
    parser.add_argument("--poses", default=DEFAULT_POSES)
    args = parser.parse_args()

    print("Deten primero mecharm_driver_node; este script abre el puerto serie.")
    input("Confirma que el area esta despejada y pulsa ENTER para continuar... ")
    home = load_home(args.poses)
    arm = MechArm270(args.port, args.baud)
    time.sleep(0.5)
    arm.power_on()
    try:
        arm.set_gripper_value(100, 40, 1)
    except Exception:  # noqa: BLE001
        pass

    intermediate = capture(arm, "intermedio")
    pregrasp = capture(arm, "preagarre")
    contact = capture(arm, "contacto")
    calibration = {
        "intermediate_joint_angles": intermediate,
        "pregrasp_joint_angles": pregrasp,
        "contact_joint_angles": contact,
        "initial_pose_name": "home",
        "carry_pose": "home",
        "calibrated_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
    }
    save_calibration(args.calibrations, args.object, calibration)
    print(f"\nCalibracion de '{args.object}' guardada en {args.calibrations}.")
    input("Pulsa ENTER para volver a home (transporte)... ")
    arm.send_angles(home, args.speed)
    if wait_arrival(arm, home):
        print("Brazo en home.")
    else:
        print("AVISO: no se confirmo llegada a home; verifica el brazo.")


if __name__ == "__main__":
    main()
