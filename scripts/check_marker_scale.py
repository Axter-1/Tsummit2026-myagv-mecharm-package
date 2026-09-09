#!/usr/bin/env python3
"""Reconcilia la distancia que ve la CAMARA con la que mide el LiDAR.

EL SINTOMA
----------
En las corridas del 09-09, con el robot ya bien encarado y centrado
(camara=-0.009, yaw=+0.7 deg), las dos medidas no coincidian:

    geometria (camara)   0.354 m
    LiDAR                0.214 m  ->  0.279 m desde base_footprint
                                      -----------------------------
    diferencia SIN explicar          0.075 m

75 mm, y repetido en dos corridas: es un sesgo sistematico, no ruido.

LAS TRES CAUSAS POSIBLES, Y COMO SE DISTINGUEN
----------------------------------------------
1. marker_length mal. La distancia estimada de un ArUco escala LINEAL
   con el tamano que se le supone: si el marcador real es mas pequeno
   que los 0.08 configurados, la camara lo cree mas lejos. Este script
   despeja el tamano real.

2. El LiDAR no esta midiendo el plano del marcador. El sensor esta a
   z=0.080; si el ArUco va montado mas alto sobre una plataforma o un
   poste, el haz puede estar tocando la BASE del soporte, que sobresale
   hacia el robot. Entonces la camara tiene razon y el LiDAR miente.

3. camera_x mal (0.16 por defecto). Desplaza las dos por igual, asi que
   se ve como un sesgo constante independiente de la distancia.

La CINTA es la que arbitra. Con --cinta se compara contra la verdad:

    la cinta coincide con el LiDAR   -> es (1), marker_length
    la cinta coincide con la camara  -> es (2), el LiDAR mide otra cosa
    la cinta no coincide con ninguna -> es (3), o algo peor

Sin --cinta solo se reporta la discrepancia y el tamano que la
explicaria, que ya es suficiente para saber si merece la pena sacar el
calibre.

USO
    # Robot quieto, de frente al marcador, a 30-50 cm.
    python3 check_marker_scale.py --id 4
    python3 check_marker_scale.py --id 4 --cinta 0.312
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

from home_service_interfaces.msg import ArucoDetectionArray


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ScaleCheck(Node):

    def __init__(self, args):
        super().__init__("check_marker_scale")

        self.args = args

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.scans = []
        self.detections = []

        self.create_subscription(
            LaserScan, args.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            ArucoDetectionArray, args.detections_topic,
            self._on_detections, 10,
        )

    def _on_scan(self, msg):
        self.scans.append(msg)

    def _on_detections(self, msg):
        for det in msg.detections:
            if int(det.id) == self.args.id:
                self.detections.append(det)

    def frame_x(self, frame, timeout):
        """x del frame en el chasis, y su yaw."""
        deadline = self.get_clock().now() + Duration(seconds=timeout)

        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.buffer.lookup_transform(
                    self.args.chassis_frame, frame, Time(),
                    timeout=Duration(seconds=0.2),
                )
            except Exception:
                continue

            return (
                float(tf.transform.translation.x),
                yaw_from_quaternion(tf.transform.rotation),
            )

        return None

    def collect(self, count, timeout):
        deadline = self.get_clock().now() + Duration(seconds=timeout)

        while (
            rclpy.ok()
            and self.get_clock().now() < deadline
            and (len(self.scans) < count or len(self.detections) < count)
        ):
            rclpy.spin_once(self, timeout_sec=0.1)


def front_range(scan, laser_yaw, half_angle_deg):
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

    return min(out) if out else None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument(
        "--cinta", type=float, default=None,
        help="metros del MORRO al plano del marcador (arbitra el empate)",
    )
    parser.add_argument("--scan-topic", default="/scan_filtered")
    parser.add_argument("--detections-topic", default="/aruco/detections")
    parser.add_argument("--chassis-frame", default="base_footprint")
    parser.add_argument("--laser-frame", default="laser_frame")
    parser.add_argument("--camera-frame", default="camera_link")
    parser.add_argument("--half-angle-deg", type=float, default=6.0)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--front-x", type=float, default=0.145,
        help="x del morro en el chasis (chassis_footprint). 0.145 medido "
             "con measure_front_offset.py",
    )
    args = parser.parse_args()

    front_x = args.front_x

    rclpy.init()
    node = ScaleCheck(args)

    try:
        laser = node.frame_x(args.laser_frame, args.timeout)
        camera = node.frame_x(args.camera_frame, args.timeout)

        if laser is None or camera is None:
            print(
                f"ERROR: falta TF de {args.chassis_frame} a "
                f"{args.laser_frame} o {args.camera_frame}.",
                file=sys.stderr,
            )
            return 1

        node.collect(args.samples, args.timeout)

        laser_x, laser_yaw = laser
        camera_x, _camera_yaw = camera

        rangos = [
            r for r in (
                front_range(s, laser_yaw, args.half_angle_deg)
                for s in node.scans
            ) if r is not None
        ]
        zetas = [float(d.distance_z) for d in node.detections]
        tamanos = [float(d.marker_size) for d in node.detections]

    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not rangos:
        print("ERROR: sin ecos validos en el sector frontal.", file=sys.stderr)
        return 1

    if not zetas:
        print(
            f"ERROR: no llego ninguna deteccion del ArUco {args.id} por "
            f"{args.detections_topic}.",
            file=sys.stderr,
        )
        return 1

    rango = statistics.median(rangos)
    z = statistics.median(zetas)
    supuesto = statistics.median(tamanos)

    # morro -> plano = (donde esta el sensor + lo que mide) - donde esta
    # el morro. Todo en el chasis, sin restas cruzadas: la primera
    # version restaba la SALIDA DEL RAYO (front_x - laser_x) despues de
    # haber sumado laser_x, o sea que contaba laser_x dos veces y
    # alargaba las dos cifras 65 mm. La discrepancia entre ellas salia
    # bien porque el error se cancelaba en la resta, pero el veredicto
    # contra la cinta era falso.
    plano_x_lidar = laser_x + rango
    plano_x_camara = camera_x + z

    lidar_chasis = plano_x_lidar - front_x
    camara_chasis = plano_x_camara - front_x
    diferencia = camara_chasis - lidar_chasis

    z_real = plano_x_lidar - camera_x
    tamano_implicado = supuesto * z_real / z if z > 1e-6 else float("nan")

    print()
    print(f"  TF: {args.laser_frame} x={laser_x:+.4f}   "
          f"{args.camera_frame} x={camera_x:+.4f}")
    print(f"  Morro (chassis_footprint) x = {front_x:+.4f} m")
    print()
    print(f"  LiDAR  ({len(rangos):2d} barridos)  rango = {rango:.4f} m"
          f"   -> morro-plano = {lidar_chasis:.4f} m")
    print(f"  Camara ({len(zetas):2d} muestras) distance_z = {z:.4f} m"
          f"   -> morro-plano = {camara_chasis:.4f} m")
    print()
    print(f"  DISCREPANCIA (camara - LiDAR) = {diferencia:+.4f} m")
    print()
    print(f"  marker_length configurado = {supuesto:.4f} m")
    print(f"  tamano que explicaria la discrepancia = "
          f"{tamano_implicado:.4f} m  ({tamano_implicado*100:.1f} cm)")
    print()

    if abs(diferencia) < 0.010:
        print("  Las dos coinciden dentro de 10 mm. No hay nada que corregir.")
        return 0

    if args.cinta is None:
        print("  Repite con --cinta <morro-plano medido> para saber CUAL de")
        print("  las dos miente. Sin eso, esto solo dice que discrepan.")
        return 0

    d_lidar = abs(args.cinta - lidar_chasis)
    d_camara = abs(args.cinta - camara_chasis)

    print(f"  Cinta morro-plano = {args.cinta:.4f} m")
    print(f"      difiere del LiDAR  en {d_lidar:+.4f} m")
    print(f"      difiere de la camara en {d_camara:+.4f} m")
    print()

    if d_lidar < d_camara and d_lidar < 0.015:
        print("  VEREDICTO: manda el LiDAR. La camara esta mal escalada.")
        print("      Mide el cuadrado NEGRO del ArUco con calibre -- el")
        print("      borde negro exterior SI cuenta, la zona blanca NO.")
        print(f"      Si da ~{tamano_implicado*100:.1f} cm, pon "
              f"marker_length:={tamano_implicado:.4f}")
        print("      Si da 8.0 cm clavados, el error esta en camera_x o en")
        print("      la calibracion intrinseca de la CSI, no en el tamano.")
    elif d_camara < d_lidar and d_camara < 0.015:
        print("  VEREDICTO: manda la camara. El LiDAR NO esta midiendo el")
        print("      plano del marcador -- el haz va a z=0.080 y "
              "probablemente")
        print("      toca la base del soporte, que sobresale. NO toques")
        print("      marker_length. Sube el marcador, o estrecha el sector")
        print("      frontal (lidar_sector_half_angle_deg).")
    else:
        print("  VEREDICTO: la cinta no coincide con NINGUNA de las dos.")
        print("      Sospecha de camera_x, del footprint, o de que el robot")
        print("      no estuviera de frente. Repite bien encarado.")

    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
