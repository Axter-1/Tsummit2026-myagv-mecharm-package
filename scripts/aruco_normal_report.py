#!/usr/bin/env python3
"""Informe de calidad de la NORMAL de un ArUco.

Responde a una sola pregunta: con esta camara, a esta altura y contra
este marcador, ¿se puede confiar en la normal del ArUco para alinearse
perpendicular, o su rumbo es ruido amplificado?

El servidor de aproximacion deduce el rumbo perpendicular del eje +Z del
marcador proyectado al plano horizontal. Dos cosas lo estropean:

  * FRACCION HORIZONTAL baja. Si la normal sale casi vertical (camara muy
    baja mirando hacia arriba), su proyeccion horizontal es minuscula y
    atan2 convierte pocos grados de error de pose en decenas de grados de
    rumbo. Se mide como |n_xy| / |n|.

  * AMBIGUEDAD PLANAR. La pose de orientacion de un marcador pequeno
    visto casi de frente tiene dos soluciones simetricas y la estimacion
    salta entre ellas. Se mide con la COHERENCIA: el modulo del vector
    medio de las direcciones horizontales unitarias. 1.00 = todas
    coinciden; ~0.71 = repartidas entre dos ramas a 90 grados, que
    promediadas dan un rumbo a 45 grados de ambas.

Uso:
    python3 scripts/aruco_normal_report.py [id] [segundos]
"""

import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from home_service_interfaces.msg import ArucoDetectionArray


class Report(Node):

    def __init__(self, target):
        super().__init__('aruco_normal_report')
        self.target = target
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.seen = 0
        self.detection = None
        self.create_subscription(
            ArucoDetectionArray, '/aruco/detections', self.cb, 10)

    def cb(self, msg):
        for d in msg.detections:
            if d.id == self.target:
                self.seen += 1
                self.detection = d


def main():
    target = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0

    rclpy.init()
    node = Report(target)

    print(f'Observando ArUco id={target} durante {seconds:.0f} s...\n')

    samples = []      # (heading_deg, horizontal_fraction)
    centers = []
    end = time.time() + seconds

    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

        if node.detection is None:
            continue

        centers.append(node.detection.center_x_normalized)
        node.detection = None

        try:
            tf = node.buffer.lookup_transform(
                'odom', f'aruco_{target}', Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            continue

        q = tf.transform.rotation
        nx = 2.0 * (q.x * q.z + q.w * q.y)
        ny = 2.0 * (q.y * q.z - q.w * q.x)
        nz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)

        planar = math.hypot(nx, ny)
        total = math.sqrt(planar * planar + nz * nz)

        if total < 1e-9 or planar < 1e-9:
            continue

        samples.append((
            math.degrees(math.atan2(ny, nx)),
            planar / total,
        ))

    print(f'detecciones: {node.seen}    normales con TF: {len(samples)}')

    if centers:
        print(f'center_x_normalized: min={min(centers):+.3f} '
              f'max={max(centers):+.3f}  (el centrado usa esto)')

    if not samples:
        print('\nSin normales. ¿Esta el marcador a la vista y corriendo '
              'el detector?')
        rclpy.shutdown()
        return

    horiz = [h for _, h in samples]
    mean_horiz = sum(horiz) / len(horiz)

    sx = sum(math.cos(math.radians(a)) for a, _ in samples)
    sy = sum(math.sin(math.radians(a)) for a, _ in samples)
    coherence = math.hypot(sx, sy) / len(samples)
    mean_heading = math.degrees(math.atan2(sy, sx))

    headings = sorted(a for a, _ in samples)

    print()
    print(f'fraccion horizontal : media {mean_horiz:.2f}   '
          f'min {min(horiz):.2f}   max {max(horiz):.2f}')
    print(f'                      (umbral normal_min_horizontal = 0.50)')
    print(f'coherencia          : {coherence:.2f}   '
          f'(umbral lock_min_coherence = 0.93)')
    print(f'rumbo medio         : {mean_heading:+.1f} deg')
    print(f'rumbos              : min {headings[0]:+.1f}   '
          f'max {headings[-1]:+.1f}   rango {headings[-1] - headings[0]:.1f} deg')

    print()
    if mean_horiz < 0.50:
        print('VEREDICTO: la normal sale casi VERTICAL. Su rumbo es ruido')
        print('  amplificado. Sube la camara o inclinala menos hacia arriba,')
        print('  o deja use_marker_normal:=false y aproximate por centrado.')
    elif coherence < 0.93:
        print('VEREDICTO: AMBIGUEDAD PLANAR. La pose salta entre dos ramas.')
        print('  Marcador mas grande, mas cerca, o mirandolo mas de lado')
        print('  (de frente del todo es el peor caso para la ambiguedad).')
    else:
        print('VEREDICTO: normal UTILIZABLE. La alineacion perpendicular')
        print(f'  deberia funcionar; rumbo objetivo {mean_heading:+.1f} deg.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
