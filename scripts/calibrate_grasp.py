#!/usr/bin/env python3
"""Ensena las tres poses articulares de agarre de una pieza, POR ALTURA
de plataforma.

Cada objeto se agarra distinto segun la altura de la mesa donde esta
apoyado. Este script captura una calibracion (intermedio -> preagarre ->
contacto) para UNA pieza a UNA altura y la guarda anidada:

  calibrations:
    <objeto>:
      "<altura_mm>":
        intermediate_joint_angles: [...]
        pregrasp_joint_angles:     [...]
        contact_joint_angles:      [...]
        table_height_mm: <altura_mm>
        calibrated_at: ...

No toca grasp_catalog.yaml. El object_grasp_server elige la altura con
el parametro 'table_height_mm'.

Uso dentro del contenedor, con mecharm_driver_node detenido
(scripts/tsummit.sh stop, o mata mecharm_driver_node):

  # 4 objetos x 2 alturas = 8 corridas
  python3 /workspace/scripts/calibrate_grasp.py poste     100
  python3 /workspace/scripts/calibrate_grasp.py poste     200
  python3 /workspace/scripts/calibrate_grasp.py engranaje 100
  python3 /workspace/scripts/calibrate_grasp.py engranaje 200
  python3 /workspace/scripts/calibrate_grasp.py rueda     100
  python3 /workspace/scripts/calibrate_grasp.py rueda     200
  python3 /workspace/scripts/calibrate_grasp.py estrella  100
  python3 /workspace/scripts/calibrate_grasp.py estrella  200
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
OBJECTS = ("engranaje", "poste", "rueda", "estrella")
# Alturas de plataforma previstas en el reglamento (mm). Se aceptan
# otras con --force-height por si el jurado cambia la pista.
KNOWN_HEIGHTS = (100, 200)
# Leidos del firmware (get_joint_min/max_angle), coinciden con la URDF.
JOINT_MIN = [-160.0, -75.0, -175.0, -155.0, -115.0, -180.0]
JOINT_MAX = [160.0, 120.0, 65.0, 155.0, 115.0, 180.0]
SEGMENTS = ("intermedio", "preagarre", "contacto")
SEGMENT_KEY = {
    "intermedio": "intermediate_joint_angles",
    "preagarre": "pregrasp_joint_angles",
    "contacto": "contact_joint_angles",
}


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


def _looks_flat(entry):
    """True si 'entry' es una calibracion antigua (sin anidar por altura)."""
    return isinstance(entry, dict) and any(
        k in entry for k in SEGMENT_KEY.values()
    )


def save_calibration(path, object_name, height_mm, calibration):
    data = {"calibrations": {}}
    if os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or data
    calibrations = data.setdefault("calibrations", {})

    entry = calibrations.get(object_name)
    if _looks_flat(entry):
        # Migra una calibracion antigua (formato plano) a la forma
        # anidada, bajo la altura que tuviera anotada o "sin_altura".
        old_height = str(entry.get("table_height_mm", "sin_altura"))
        calibrations[object_name] = {old_height: entry}
        print(
            f"AVISO: la calibracion previa de '{object_name}' estaba en "
            f"formato plano; migrada a la altura '{old_height}'."
        )
    elif not isinstance(entry, dict):
        calibrations[object_name] = {}

    calibrations[object_name][str(height_mm)] = calibration

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
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("object", choices=OBJECTS)
    parser.add_argument(
        "table_mm",
        type=int,
        help="altura de la plataforma en mm (100 o 200)",
    )
    parser.add_argument(
        "--force-height",
        action="store_true",
        help="acepta una altura distinta de 100/200",
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--calibrations", default=DEFAULT_CALIBRATIONS)
    parser.add_argument("--poses", default=DEFAULT_POSES)
    args = parser.parse_args()

    if args.table_mm <= 0:
        parser.error("table_mm debe ser positivo (mm)")
    if args.table_mm not in KNOWN_HEIGHTS and not args.force_height:
        parser.error(
            f"altura {args.table_mm} mm no esperada (validas: "
            f"{', '.join(str(h) for h in KNOWN_HEIGHTS)}). Usa "
            f"--force-height si es a proposito."
        )

    print(
        f"\n== CALIBRACION: {args.object.upper()} sobre plataforma de "
        f"{args.table_mm} mm ==\n"
    )
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

    captured = {}
    for name in SEGMENTS:
        captured[SEGMENT_KEY[name]] = capture(arm, name)

    calibration = {
        "intermediate_joint_angles": captured["intermediate_joint_angles"],
        "pregrasp_joint_angles": captured["pregrasp_joint_angles"],
        "contact_joint_angles": captured["contact_joint_angles"],
        "table_height_mm": args.table_mm,
        "initial_pose_name": "home",
        "carry_pose": "home",
        "calibrated_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
    }
    save_calibration(args.calibrations, args.object, args.table_mm, calibration)
    print(
        f"\nCalibracion de '{args.object}' a {args.table_mm} mm guardada en "
        f"{args.calibrations}."
    )
    input("Pulsa ENTER para volver a home (transporte)... ")
    arm.send_angles(home, args.speed)
    if wait_arrival(arm, home):
        print("Brazo en home.")
    else:
        print("AVISO: no se confirmo llegada a home; verifica el brazo.")


if __name__ == "__main__":
    main()
