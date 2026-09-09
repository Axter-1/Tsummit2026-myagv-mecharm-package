#!/usr/bin/env python3
"""Guarda la pose actual del robot junto al mapa, con un nombre.

POR QUE EXISTE
--------------
Las coordenadas de los pasos `navigate` de una mision se median a mano:
llevar el robot al sitio, leer /amcl_pose o tf2_echo, copiar tres
numeros al YAML. Eso es lento, se equivoca de signo, y sobre todo NO
queda atado al mapa: un mapa nuevo invalida las coordenadas viejas sin
que nada avise.

Aqui la pose se guarda EN EL MISMO SITIO que el mapa y con su nombre:

    maps/<mapa>.pgm
    maps/<mapa>.yaml
    maps/<mapa>.poses.yaml     <- esto

Asi la mision pide posiciones POR NOMBRE ('start', 'aruco_0', ...) y el
mission_manager las resuelve contra el fichero del mapa que este
cargado. Cambiar de mapa cambia las posiciones a la vez, o falla de
forma ruidosa si falta alguna, en vez de navegar a un punto de otra
pista.

DE DONDE SALE LA POSE
---------------------
De la TF `map -> base_footprint`, que es la pose que Nav2 usa de verdad.
No de /odom: odom no tiene origen comun con el mapa y deriva.

Con SLAM en vivo el origen de `map` es la pose inicial del robot, o sea
el START. Con AMCL sobre un mapa guardado, el origen es el del mapa. Por
eso las posiciones SOLO valen para el mapa junto al que se guardaron, y
por eso el fichero lleva el nombre del mapa dentro.

USO
    python3 save_pose.py start
    python3 save_pose.py aruco_0 --map pista_reto1
    python3 save_pose.py --list
    python3 save_pose.py posicion_2 --force      # sobrescribir
"""

import argparse
import datetime
import math
import os
import sys

import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener


DEFAULT_MAPS_DIR = os.environ.get(
    "MAPS_DIR",
    "/workspace/maps" if os.path.isdir("/workspace/maps") else
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "maps",
    ),
)


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def resolve_map_name(maps_dir, requested):
    """Nombre del mapa al que se atan las posiciones.

    Sin `--map` se coge el .yaml de mapa mas reciente del directorio.
    Es lo que casi siempre se quiere (acabas de guardar el mapa y ahora
    guardas las poses), pero se dice en voz alta: elegir el mapa
    equivocado en silencio manda al robot a coordenadas de otra pista.
    """
    if requested:
        return requested

    if not os.path.isdir(maps_dir):
        return None

    candidates = [
        f[:-5] for f in os.listdir(maps_dir)
        if f.endswith(".yaml") and not f.endswith(".poses.yaml")
    ]

    if not candidates:
        return None

    candidates.sort(
        key=lambda n: os.path.getmtime(os.path.join(maps_dir, n + ".yaml")),
        reverse=True,
    )

    return candidates[0]


def poses_path(maps_dir, map_name):
    return os.path.join(maps_dir, f"{map_name}.poses.yaml")


def load_poses(path):
    if not os.path.isfile(path):
        return {}

    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    return data.get("poses", {}) or {}


def write_poses(path, map_name, frame_id, poses):
    payload = {
        "map": map_name,
        "frame_id": frame_id,
        "poses": poses,
    }

    header = (
        "# Posiciones alcanzables guardadas junto al mapa.\n"
        "#\n"
        f"# Mapa:  {map_name}.yaml / {map_name}.pgm\n"
        f"# Marco: {frame_id}\n"
        "#\n"
        "# Generado por scripts/save_pose.py. Se puede editar a mano,\n"
        "# pero las coordenadas SOLO valen para ESTE mapa: si vuelves a\n"
        "# mapear, vuelve a guardar las posiciones.\n"
        "#\n"
        "# Uso desde una mision (home_service_mission):\n"
        "#     - type: navigate\n"
        "#       pose: start\n"
        "\n"
    )

    tmp = path + ".tmp"

    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(header)
        yaml.safe_dump(
            payload, handle,
            default_flow_style=False, sort_keys=False, allow_unicode=True,
        )

    os.replace(tmp, path)


class PoseGrabber(Node):

    def __init__(self, frame_id, base_frame, timeout):
        super().__init__("save_pose")

        self.frame_id = frame_id
        self.base_frame = base_frame
        self.timeout = timeout

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def grab(self):
        """(x, y, yaw) del robot en `frame_id`, o None.

        Se insiste durante `timeout` porque el buffer de TF nace vacio:
        pedirlo en el primer ciclo falla siempre, y eso parecia "no hay
        mapa" cuando solo era que el listener aun no habia oido nada.
        """
        deadline = self.get_clock().now() + Duration(seconds=self.timeout)
        last_error = "sin intentos"

        while rclpy.ok() and self.get_clock().now() < deadline:

            rclpy.spin_once(self, timeout_sec=0.1)

            try:
                transform = self.buffer.lookup_transform(
                    self.frame_id, self.base_frame, Time(),
                    timeout=Duration(seconds=0.2),
                )

            except Exception as exc:  # noqa: BLE001
                last_error = f"{type(exc).__name__}: {exc}"
                continue

            t = transform.transform.translation
            q = transform.transform.rotation

            return (
                float(t.x), float(t.y),
                yaw_from_quaternion(q.x, q.y, q.z, q.w),
            )

        self.get_logger().error(
            f"No hay TF {self.frame_id} -> {self.base_frame} tras "
            f"{self.timeout:.0f} s. Ultimo error: {last_error}"
        )
        return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "name", nargs="?",
        help="nombre de la posicion (p.ej. start, aruco_0, finish)",
    )
    parser.add_argument(
        "--map", default="",
        help="mapa al que se atan las poses (por defecto, el mas reciente)",
    )
    parser.add_argument("--maps-dir", default=DEFAULT_MAPS_DIR)
    parser.add_argument("--frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument(
        "--timeout", type=float, default=10.0,
        help="segundos esperando la TF antes de rendirse",
    )
    parser.add_argument(
        "--list", action="store_true",
        help="lista las posiciones guardadas y sale",
    )
    parser.add_argument(
        "--delete", action="store_true",
        help="borra la posicion indicada",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="sobrescribe una posicion que ya existe",
    )
    parser.add_argument(
        "--note", default="",
        help="comentario libre que se guarda con la posicion",
    )

    args = parser.parse_args()

    map_name = resolve_map_name(args.maps_dir, args.map)

    if map_name is None:
        print(
            f"ERROR: no hay ningun mapa en {args.maps_dir}. "
            "Guarda uno antes con 'save-map <nombre>'.",
            file=sys.stderr,
        )
        return 1

    path = poses_path(args.maps_dir, map_name)
    poses = load_poses(path)

    if args.list:
        print(f"Mapa: {map_name}")
        print(f"Fichero: {path}")
        if not poses:
            print("  (sin posiciones guardadas)")
            return 0
        for key in sorted(poses):
            p = poses[key]
            note = f"  # {p['note']}" if p.get("note") else ""
            print(
                f"  {key:20s} x={p['x']:+.3f} y={p['y']:+.3f} "
                f"yaw={p['yaw_deg']:+.1f} deg{note}"
            )
        return 0

    if not args.name:
        parser.error("hace falta el nombre de la posicion (o --list)")

    if args.delete:
        if args.name not in poses:
            print(f"ERROR: '{args.name}' no existe en {path}", file=sys.stderr)
            return 1
        poses.pop(args.name)
        write_poses(path, map_name, args.frame, poses)
        print(f"Borrada '{args.name}' de {path}")
        return 0

    if args.name in poses and not args.force:
        p = poses[args.name]
        print(
            f"ERROR: '{args.name}' ya existe en {path}\n"
            f"       x={p['x']:+.3f} y={p['y']:+.3f} "
            f"yaw={p['yaw_deg']:+.1f} deg\n"
            "       Repite con --force para sobrescribirla.",
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = PoseGrabber(args.frame, args.base_frame, args.timeout)

    try:
        pose = node.grab()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if pose is None:
        print(
            "\nNo se pudo leer la pose. Comprueba que hay SLAM o AMCL en\n"
            "marcha: sin uno de los dos nadie publica map -> odom y el\n"
            "robot no sabe donde esta respecto al mapa.",
            file=sys.stderr,
        )
        return 1

    x, y, yaw = pose

    entry = {
        "x": round(x, 4),
        "y": round(y, 4),
        "yaw_deg": round(math.degrees(yaw), 2),
        "saved_at": datetime.datetime.now().astimezone().isoformat(
            timespec="seconds"
        ),
    }

    if args.note:
        entry["note"] = args.note

    poses[args.name] = entry
    write_poses(path, map_name, args.frame, poses)

    print(
        f"Guardada '{args.name}' en {path}\n"
        f"  x={entry['x']:+.3f}  y={entry['y']:+.3f}  "
        f"yaw={entry['yaw_deg']:+.1f} deg  (marco {args.frame})"
    )
    print(f"  Posiciones en este mapa: {', '.join(sorted(poses))}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
