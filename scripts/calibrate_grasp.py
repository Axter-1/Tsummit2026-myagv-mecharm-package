#!/usr/bin/env python3
"""Ensena las tres poses de una accion (pick/place), POR ALTURA de plataforma.

Cada objeto se agarra distinto segun la altura de la mesa donde esta
apoyado. Este script reutiliza la pose global ``home`` existente y despues
captura una calibracion (intermedio -> preagarre -> contacto) para UNA pieza
a UNA altura. Con ``--capture-home`` se puede actualizar ``home`` de forma
explicita; ``--capture-global-poses`` tambien ensena todas las poses globales
de ``poses.yaml`` antes de calibrar la pieza.

  calibrations:
    <objeto>:
      "<altura_mm>":
        pick:  # o place; ambos pueden coexistir
          intermediate_coords: [X,Y,Z,RX,RY,RZ]
          pregrasp_coords:     [X,Y,Z,RX,RY,RZ]
          contact_coords:      [X,Y,Z,RX,RY,RZ]
          intermediate_joint_angles: [...]  # diagnostico/respaldo
          pregrasp_joint_angles:     [...]  # diagnostico/respaldo
          contact_joint_angles:      [...]  # diagnostico/respaldo
          table_height_mm: <altura_mm>
          operation: pick
          frame_id: base_link
          gripper: {open_value: 0, close_value: 0, speed_percent: 0}
          calibrated_at: ...

No toca grasp_catalog.yaml. El object_grasp_server elige la altura con
el parametro 'table_height_mm'.

Uso dentro del contenedor, con mecharm_driver_node detenido
(scripts/tsummit.sh stop, o mata mecharm_driver_node):

  python3 /workspace/scripts/calibrate_grasp.py poste 100 --action pick
  python3 /workspace/scripts/calibrate_grasp.py rueda 100 --action place
  python3 /workspace/scripts/calibrate_grasp.py poste 100 --action pick \
      --capture-global-poses
  python3 /workspace/scripts/calibrate_grasp.py --menu
"""

import argparse
import datetime as dt
import os
import shutil
import tempfile
import time

import yaml

try:
    from pymycobot.mecharm270 import MechArm270
except ImportError:
    MechArm270 = None


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
OBJECTS = ("engranaje", "poste", "rueda", "estrella")
# Alturas de plataforma previstas en el reglamento (mm). Se aceptan
# otras con --force-height por si el jurado cambia la pista.
KNOWN_HEIGHTS = (100, 200)
# Leidos del firmware (get_joint_min/max_angle), coinciden con la URDF.
JOINT_MIN = [-160.0, -75.0, -175.0, -155.0, -115.0, -180.0]
JOINT_MAX = [160.0, 120.0, 65.0, 155.0, 115.0, 180.0]
SEGMENTS = ("intermedio", "preagarre", "contacto")
GLOBAL_POSES = (
    "home",
    "safe_navigation",
    "carry",
    "observe",
    "pick_table",
    "place_table",
)
SEGMENT_KEY = {
    "intermedio": "intermediate_joint_angles",
    "preagarre": "pregrasp_joint_angles",
    "contacto": "contact_joint_angles",
}
COORD_SEGMENT_KEY = {
    "intermedio": "intermediate_coords",
    "preagarre": "pregrasp_coords",
    "contacto": "contact_coords",
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


def read_coords(arm, retries=12):
    for _ in range(retries):
        try:
            coords = arm.get_coords()
        except Exception:  # noqa: BLE001
            coords = None
        if isinstance(coords, (list, tuple)) and len(coords) == 6:
            return [round(float(value), 2) for value in coords]
        time.sleep(0.15)
    return None


def wait_arrival(arm, target, timeout=30.0):
    deadline = time.monotonic() + timeout
    stable = 0
    while time.monotonic() < deadline:
        current = read_angles(arm, retries=1)
        if current is not None:
            error = max(
                joint_error(index, current[index], target[index])
                for index in range(6)
            )
            if error <= 3.0:
                stable += 1
                if stable == 3:
                    return True
            else:
                stable = 0
        time.sleep(0.15)
    return False


def joint_error(index, actual, target):
    difference = float(actual) - float(target)
    if index == 5:
        return abs((difference + 180.0) % 360.0 - 180.0)
    return abs(difference)


def send_angles_via_waypoint(arm, target, speed, safe_pose=None):
    """Evita saltos de mas de 180 grados en una articulacion."""
    current = read_angles(arm, retries=2)
    if current is not None:
        waypoint = list(target)
        large_jumps = []
        for index, (actual, desired) in enumerate(zip(current, target)):
            if joint_error(index, actual, desired) > 180.0:
                waypoint[index] = (actual + desired) / 2.0
                large_jumps.append(index + 1)
        if large_jumps:
            if safe_pose is not None:
                waypoint = list(safe_pose)
            print(
                "Ruta segura para volver a home: "
                f"saltos >180 grados en J{large_jumps}."
            )
            arm.send_angles(waypoint, speed)
            if not wait_arrival(arm, waypoint):
                return False
    arm.send_angles(target, speed)
    return wait_arrival(arm, target)


def validate(angles):
    for index, value in enumerate(angles):
        if value < JOINT_MIN[index] or value > JOINT_MAX[index]:
            raise ValueError(
                f"J{index + 1}={value:.2f} fuera de limites "
                f"[{JOINT_MIN[index]}, {JOINT_MAX[index]}]"
            )


def capture(arm, name, include_coords=False):
    while True:
        print(f"\n{name.upper()}: sujeta el brazo antes de liberarlo.")
        input("Pulsa ENTER para liberar servos y llevarlo manualmente a la pose... ")
        arm.release_all_servos(1)
        input("Colocalo, sujetalo firme y pulsa ENTER para fijar y leer los angulos... ")
        arm.power_on()
        arm.clear_error_information()
        time.sleep(0.5)
        angles = read_angles(arm)
        if angles is None:
            raise RuntimeError("no se pudieron leer los angulos tras fijar la pose")
        print(f"  {name}: {angles}")
        try:
            validate(angles)
        except ValueError as exc:
            print(f"  AVISO: {exc}; recoloca la pose y vuelve a intentarlo.")
            continue
        if not include_coords:
            return angles
        coords = read_coords(arm)
        if coords is None:
            raise RuntimeError("no se pudieron leer las coordenadas de la pose")
        print(f"  {name} coords: {coords}")
        return angles, coords


def load_pose(path, name):
    with open(path, "r", encoding="utf-8") as handle:
        poses = (yaml.safe_load(handle) or {}).get("poses", {})
    pose = poses.get(name)
    if not isinstance(pose, (list, tuple)) or len(pose) != 6:
        raise RuntimeError(f"no hay una pose {name} valida en {path}")
    return [float(value) for value in pose]


def load_home(path):
    return load_pose(path, "home")


def save_pose(path, name, angles):
    """Actualiza una pose sin tocar las demas poses del archivo."""
    data = {"poses": {}}
    if os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or data
    poses = data.get("poses")
    if not isinstance(poses, dict):
        poses = {}
        data["poses"] = poses
    poses[name] = [round(float(value), 2) for value in angles]

    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".poses_calibration_", dir=directory)
    try:
        header = (
            "# Poses con nombre del MechArm 270 M5 (grados, [J1..J6]).\n"
            "# Editado por scripts/calibrate_grasp.py y mecharm_console.py\n\n"
        )
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(header)
            yaml.safe_dump(
                {"poses": poses},
                handle,
                allow_unicode=False,
                default_flow_style=True,
                sort_keys=False,
            )
        if os.path.isfile(path):
            shutil.copy2(path, path + ".bak")
        os.replace(temporary, path)
        os.chmod(path, 0o664)
    except Exception:
        os.unlink(temporary)
        raise


def _looks_flat(entry):
    """True si 'entry' es una calibracion antigua (sin anidar por altura)."""
    return isinstance(entry, dict) and any(
        k in entry for k in SEGMENT_KEY.values()
    )


def save_calibration(path, object_name, height_mm, calibration, action="pick"):
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

    calibration = dict(calibration)
    calibration["operation"] = action
    height_key = str(height_mm)
    existing = calibrations[object_name].get(height_key)
    if isinstance(existing, dict) and _looks_flat(existing):
        if action == "pick":
            calibrations[object_name][height_key] = calibration
        else:
            calibrations[object_name][height_key] = {
                "pick": existing,
                "place": calibration,
            }
    elif isinstance(existing, dict):
        # Formato por accion: conserva pick/place independientes.
        entry = dict(existing)
        entry[action] = calibration
        calibrations[object_name][height_key] = entry
    else:
        calibrations[object_name][height_key] = calibration

    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".grasp_calibration_", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.safe_dump(data, handle, allow_unicode=False, sort_keys=False)
        if os.path.isfile(path):
            shutil.copy2(path, path + ".bak")
        os.replace(temporary, path)
        # El script corre como root dentro del contenedor. Dejar el YAML
        # legible y editable desde el host permite revisarlo y subirlo a Git.
        os.chmod(path, 0o664)
    except Exception:
        os.unlink(temporary)
        raise


def load_calibrations(path):
    if not os.path.isfile(path):
        return {"calibrations": {}}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data.get("calibrations", {}), dict):
        raise RuntimeError("'calibrations' debe ser un mapa")
    return data


def save_yaml(path, data):
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".grasp_menu_", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.safe_dump(data, handle, allow_unicode=False, sort_keys=False)
        if os.path.isfile(path):
            shutil.copy2(path, path + ".bak")
        os.replace(temporary, path)
        os.chmod(path, 0o664)
    except Exception:
        os.unlink(temporary)
        raise


def menu_records(data):
    records = []
    for object_name, heights in (data.get("calibrations", {}) or {}).items():
        if not isinstance(heights, dict):
            continue
        for height, entry in heights.items():
            if not isinstance(entry, dict):
                continue
            if _looks_flat(entry):
                records.append((str(object_name), str(height), "pick", entry))
            else:
                for action in ("pick", "place"):
                    if isinstance(entry.get(action), dict):
                        records.append((str(object_name), str(height), action,
                                        entry[action]))
    return records


def choose_record(records):
    if not records:
        print("No hay calibraciones guardadas.")
        return None
    for index, (obj, height, action, _) in enumerate(records, start=1):
        print(f"  {index}. {obj} / {action} / {height} mm")
    try:
        selected = int(input("Selecciona numero (0 cancela): "))
    except ValueError:
        return None
    if selected <= 0 or selected > len(records):
        return None
    return records[selected - 1]


def calibration_menu(path, poses_path, port, baud, speed):
    """Menu de mantenimiento de calibraciones, sin editar YAML a mano."""
    del poses_path
    while True:
        data = load_calibrations(path)
        records = menu_records(data)
        print("\n== CALIBRACIONES ==")
        print("  l) listar   v) ver   e) editar   r) recapturar   d) eliminar   q) salir")
        choice = input("Opcion: ").strip().lower()
        if choice in ("q", "quit", "salir"):
            return
        if choice in ("l", "listar"):
            for index, (obj, height, action, entry) in enumerate(records, 1):
                stamp = entry.get("calibrated_at", "sin fecha")
                print(f"  {index}. {obj} / {action} / {height} mm ({stamp})")
            continue
        selected = choose_record(records)
        if selected is None:
            continue
        object_name, height, action, entry = selected
        if choice in ("v", "ver"):
            print(yaml.safe_dump(entry, allow_unicode=False, sort_keys=False))
        elif choice in ("e", "editar"):
            stage = input("Etapa (intermedio/preagarre/contacto): ").strip()
            if stage not in SEGMENTS:
                print("Etapa invalida.")
                continue
            kind = input("Representacion (joints/coords) [joints]: ").strip().lower()
            if kind not in ("", "joints", "coords"):
                print("Representacion invalida.")
                continue
            kind = kind or "joints"
            key = SEGMENT_KEY[stage] if kind == "joints" else COORD_SEGMENT_KEY[stage]
            raw = input(f"Valores actuales {entry.get(key)}; nuevos 6 numeros: ")
            try:
                values = [float(value) for value in raw.replace(",", " ").split()]
                if len(values) != 6:
                    raise ValueError
                if kind == "joints":
                    validate(values)
            except ValueError:
                print("Valores invalidos o fuera de limites.")
                continue
            entry[key] = [round(value, 2) for value in values]
            save_yaml(path, data)
            print(f"Posicion {kind} actualizada.")
        elif choice in ("d", "eliminar"):
            if input(f"Escribe ELIMINAR para borrar {object_name}/{action}/{height}: ") != "ELIMINAR":
                continue
            heights = data["calibrations"][object_name]
            height_entry = heights[height]
            if _looks_flat(height_entry):
                del heights[height]
            else:
                del height_entry[action]
                if not height_entry:
                    del heights[height]
            if not heights:
                del data["calibrations"][object_name]
            save_yaml(path, data)
            print("Calibracion eliminada.")
        elif choice in ("r", "recapturar"):
            if MechArm270 is None:
                print("No se puede recapturar: falta pymycobot.")
                continue
            stage = input("Etapa (intermedio/preagarre/contacto): ").strip()
            if stage not in SEGMENTS:
                print("Etapa invalida.")
                continue
            input("Area despejada. Pulsa ENTER para abrir el brazo... ")
            arm = MechArm270(port, baud)
            arm.power_on()
            angles, coords = capture(arm, stage, include_coords=True)
            entry[SEGMENT_KEY[stage]] = angles
            entry[COORD_SEGMENT_KEY[stage]] = coords
            entry["calibrated_at"] = dt.datetime.now().astimezone().isoformat(
                timespec="seconds"
            )
            save_yaml(path, data)
            print("Posicion recapturada y guardada.")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("object", nargs="?", choices=OBJECTS)
    parser.add_argument("table_mm", nargs="?", type=int)
    parser.add_argument(
        "--action", choices=("pick", "place"),
        help="accion que se calibrara; obligatoria fuera del menu",
    )
    parser.add_argument(
        "--menu", action="store_true",
        help="gestiona calibraciones existentes sin editar YAML a mano",
    )
    parser.add_argument(
        "--force-height",
        action="store_true",
        help="acepta una altura distinta de 100/200",
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--gripper-open-value", type=int, default=0)
    parser.add_argument("--gripper-close-value", type=int, default=0)
    parser.add_argument("--gripper-speed-percent", type=float, default=0.0)
    parser.add_argument(
        "--capture-global-poses",
        action="store_true",
        help="ensena home, safe_navigation, carry, observe, pick_table y place_table",
    )
    parser.add_argument(
        "--capture-home",
        action="store_true",
        help="actualiza la pose global home antes de calibrar",
    )
    parser.add_argument(
        "--execution-mode",
        choices=("coords", "joints"),
        default=None,
        help="via de ejecucion; rueda usa joints por defecto, las demas coords",
    )
    parser.add_argument("--calibrations", default=DEFAULT_CALIBRATIONS)
    parser.add_argument("--poses", default=DEFAULT_POSES)
    args = parser.parse_args()

    if args.menu:
        calibration_menu(args.calibrations, args.poses, args.port, args.baud,
                         args.speed)
        return
    if args.object is None or args.table_mm is None or args.action is None:
        parser.error("object, table_mm y --action pick|place son obligatorios")

    if args.table_mm <= 0:
        parser.error("table_mm debe ser positivo (mm)")
    if args.table_mm not in KNOWN_HEIGHTS and not args.force_height:
        parser.error(
            f"altura {args.table_mm} mm no esperada (validas: "
            f"{', '.join(str(h) for h in KNOWN_HEIGHTS)}). Usa "
            f"--force-height si es a proposito."
        )

    if MechArm270 is None:
        parser.error("falta pymycobot; solo el menu de lectura no lo necesita")

    print(
        f"\n== CALIBRACION: {args.object.upper()} sobre plataforma de "
        f"{args.table_mm} mm ==\n"
    )
    print("Deten primero mecharm_driver_node; este script abre el puerto serie.")
    input("Confirma que el area esta despejada y pulsa ENTER para continuar... ")
    arm = MechArm270(args.port, args.baud)
    time.sleep(0.5)
    arm.power_on()
    arm.clear_error_information()

    if args.capture_global_poses:
        pose_names = GLOBAL_POSES
    elif args.capture_home:
        pose_names = ("home",)
    else:
        pose_names = ()

    captured_global = {}
    for pose_name in pose_names:
        captured_global[pose_name] = capture(arm, pose_name)
        save_pose(args.poses, pose_name, captured_global[pose_name])
        print(f"  pose global '{pose_name}' guardada en {args.poses}.")

    home = captured_global.get("home")
    if home is None:
        home = load_home(args.poses)
        print(f"  pose global 'home' reutilizada desde {args.poses}.")
    safe_pose = captured_global.get("safe_navigation")
    if safe_pose is None:
        try:
            safe_pose = load_pose(args.poses, "safe_navigation")
        except RuntimeError:
            safe_pose = None

    # La captura de poses se hace con la pinza abierta. Si se proporciona
    # un valor ya calibrado se usa tambien durante la enseñanza; 0 conserva
    # la apertura total segura para una primera calibracion.
    open_value = args.gripper_open_value if args.gripper_open_value > 0 else 100
    try:
        arm.set_gripper_value(open_value, 40, 1)
    except Exception:  # noqa: BLE001
        pass

    captured = {}
    captured_coords = {}
    for name in SEGMENTS:
        angles, coords = capture(arm, name, include_coords=True)
        captured[SEGMENT_KEY[name]] = angles
        captured_coords[COORD_SEGMENT_KEY[name]] = coords

    execution_mode = args.execution_mode or (
        "joints" if args.object == "rueda" else "coords"
    )
    calibration = {
        **captured_coords,
        "intermediate_joint_angles": captured["intermediate_joint_angles"],
        "pregrasp_joint_angles": captured["pregrasp_joint_angles"],
        "contact_joint_angles": captured["contact_joint_angles"],
        "execution_mode": execution_mode,
        "table_height_mm": args.table_mm,
        "initial_pose_name": "home",
        "carry_pose": "carry" if "carry" in captured_global else "home",
        "calibrated_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "frame_id": "base_link",
        "gripper": {
            "open_value": args.gripper_open_value,
            "close_value": args.gripper_close_value,
            "speed_percent": args.gripper_speed_percent,
        },
    }
    save_calibration(
        args.calibrations, args.object, args.table_mm, calibration, args.action
    )
    print(
        f"\nCalibracion de '{args.object}' a {args.table_mm} mm guardada en "
        f"{args.calibrations}."
    )
    input("Pulsa ENTER para volver a home (transporte)... ")
    if send_angles_via_waypoint(arm, home, args.speed, safe_pose):
        print("Brazo en home.")
    else:
        print("AVISO: no se confirmo llegada a home; verifica el brazo.")


if __name__ == "__main__":
    main()
