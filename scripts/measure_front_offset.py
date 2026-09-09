#!/usr/bin/env python3
"""Mide morro -> sensor de una vez, con el scan y una cinta.

POR QUE
-------
`chassis_footprint` y `lidar_to_front_bumper_m` se han estimado tres
veces por tres caminos distintos y han dado tres respuestas:

    0.123   footprint heredado de Nav2 (0.188) menos laser_x (0.065)
    0.081   reconciliando cinta y scan contra el ArUco 2
    0.060   cinta directa

Ninguna es medida de punta a punta. Todas mezclan supuestos: donde cae
base_footprint dentro del chasis, el radio de la carcasa del LiDAR, si
el eco venia del marcador o del fondo.

Lo que el servidor usa de verdad es UNA cantidad:

    salida_del_rayo = front_x - laser_x

o sea, cuanto avanza el haz frontal desde el origen de `laser_frame`
hasta salir del poligono del chasis. Y esa cantidad se mide sin saber
nada del interior del LiDAR ni de donde esta base_footprint:

    salida_del_rayo = rango_del_scan - (morro -> pared, con cinta)

Este script lee el rango del sector frontal exactamente como lo lee el
servidor, le resta tu cinta y dice que valores poner.

USO
    # 1. Robot de frente a una pared PLANA y lisa, a unos 30-40 cm.
    #    Sin el marcador puesto: se quiere la pared, no el carton.
    # 2. Cinta del punto mas saliente del morro a la pared.
    # 3. Con la pila corriendo:
    python3 measure_front_offset.py --morro-pared 0.312
"""

import argparse
import math
import statistics
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class FrontRange(Node):

    def __init__(self, scan_topic, chassis_frame, laser_frame):
        super().__init__("measure_front_offset")

        self.chassis_frame = chassis_frame
        self.laser_frame = laser_frame

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.scans = []
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )

    def _on_scan(self, msg):
        self.scans.append(msg)

    def laser_pose(self, timeout):
        """(x, y, yaw) de laser_frame en el chasis. Igual que el servidor."""
        deadline = self.get_clock().now() + Duration(seconds=timeout)

        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.buffer.lookup_transform(
                    self.chassis_frame, self.laser_frame, Time(),
                    timeout=Duration(seconds=0.2),
                )
            except Exception:
                continue

            return (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                yaw_from_quaternion(tf.transform.rotation),
            )

        return None

    def collect(self, count, timeout):
        deadline = self.get_clock().now() + Duration(seconds=timeout)

        while (
            rclpy.ok()
            and len(self.scans) < count
            and self.get_clock().now() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.scans


def front_ranges(scan, laser_yaw, half_angle_deg):
    """Ecos del sector frontal del ROBOT, no del sensor.

    El montaje del LiDAR lleva yaw = pi, asi que el frente del robot NO
    es el angulo 0 del scan. Se deduce de la TF, igual que hace
    get_lidar_front_angle() en el servidor.
    """
    front_angle = -laser_yaw
    half = math.radians(half_angle_deg)

    out = []

    for index, value in enumerate(scan.ranges):
        if not math.isfinite(value):
            continue
        if value < scan.range_min or value > scan.range_max:
            continue

        angle = scan.angle_min + index * scan.angle_increment
        delta = math.atan2(
            math.sin(angle - front_angle), math.cos(angle - front_angle)
        )

        if abs(delta) <= half:
            out.append(float(value))

    return out


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--morro-pared", type=float, required=True,
        help="metros del punto mas saliente del morro a la pared (cinta)",
    )
    parser.add_argument("--scan-topic", default="/scan_filtered")
    parser.add_argument("--chassis-frame", default="base_footprint")
    parser.add_argument("--laser-frame", default="laser_frame")
    parser.add_argument("--half-angle-deg", type=float, default=6.0)
    parser.add_argument("--scans", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = FrontRange(args.scan_topic, args.chassis_frame, args.laser_frame)

    try:
        pose = node.laser_pose(args.timeout)

        if pose is None:
            print(
                f"ERROR: no hay TF {args.chassis_frame} -> "
                f"{args.laser_frame}. ¿Esta la pila arriba?",
                file=sys.stderr,
            )
            return 1

        laser_x, laser_y, laser_yaw = pose

        scans = node.collect(args.scans, args.timeout)

        if not scans:
            print(
                f"ERROR: no llega nada por {args.scan_topic}.",
                file=sys.stderr,
            )
            return 1

        muestras = []
        for scan in scans:
            sector = front_ranges(scan, laser_yaw, args.half_angle_deg)
            if sector:
                muestras.append(min(sector))

    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not muestras:
        print(
            f"ERROR: el sector frontal (+-{args.half_angle_deg:.0f} deg) "
            "no tiene ni un eco valido. ¿Demasiado cerca, o el LiDAR "
            "midiendo por encima de la pared?",
            file=sys.stderr,
        )
        return 1

    rango = statistics.median(muestras)
    dispersion = max(muestras) - min(muestras)

    salida = rango - args.morro_pared
    front_x = laser_x + salida

    print()
    print(f"  TF {args.chassis_frame} -> {args.laser_frame}")
    print(
        f"      x = {laser_x:+.4f}   y = {laser_y:+.4f}   "
        f"yaw = {math.degrees(laser_yaw):+.1f} deg"
    )
    print()
    print(f"  Sector frontal +-{args.half_angle_deg:.0f} deg, "
          f"{len(muestras)} barridos")
    print(f"      rango (mediana del eco mas cercano) = {rango:.4f} m")
    print(f"      dispersion entre barridos           = {dispersion:.4f} m")
    print()
    print(f"  Cinta morro -> pared                    = "
          f"{args.morro_pared:.4f} m")
    print()
    print("  RESULTADO")
    print(f"      salida del rayo (morro - sensor)    = {salida:+.4f} m")
    print(f"      front_x  (morro en {args.chassis_frame})".ljust(43)
          + f"= {front_x:+.4f} m")
    print()

    if dispersion > 0.01:
        print(
            f"  AVISO: {dispersion*1000:.0f} mm de dispersion entre "
            "barridos. El robot no esta quieto, la pared no es plana, o "
            "el sector coge un borde. Repite."
        )

    if salida <= 0.0:
        print(
            "  AVISO: salida negativa. El sensor quedaria POR DELANTE "
            "del morro, lo que no puede ser. Revisa que la cinta va al "
            "punto mas saliente y que el eco es de la pared."
        )
        return 1

    print("  Para aplicarlo:")
    print(f"      lidar_to_front_bumper_m = {salida:.3f}")
    print(
        "      chassis_footprint       = "
        f"[{front_x:.3f}, ANCHO, {front_x:.3f}, -ANCHO, "
        "TRASERA, -ANCHO, TRASERA, ANCHO]"
    )
    print(
        "      TRASERA = front_x - largo_total_con_cinta "
        f"= {front_x:.3f} - L"
    )
    print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
