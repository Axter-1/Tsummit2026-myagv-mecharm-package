#!/usr/bin/env python3
"""Saneador del LaserScan del YDLIDAR X2L del myAGV.

PROBLEMA QUE RESUELVE
=====================
El driver del X2L en este robot esta configurado con:

    invalid_range_is_inf: false
    ignore_array: "-50,50"

Con ``invalid_range_is_inf: false`` los haces SIN retorno (espacio libre
mas alla del alcance, o superficies que no reflejan) NO salen como
``+inf``: salen como ``0.0``. Nav2 descarta cualquier lectura fuera de
``[range_min, range_max]``, asi que esos haces **no producen raytracing**
y por lo tanto **no limpian** las celdas que marcaron antes.

Consecuencia exacta del sintoma reportado: al girar, las paredes
marcadas en la pose anterior nunca se borran del costmap porque los
haces que deberian barrerlas son "invalidos". El mapa se llena de
paredes fantasma y el robot elige caminos que no existen.

QUE HACE ESTE NODO
==================
1. **ceros -> +inf** fuera de los sectores ciegos: convierte los "sin
   retorno" en lecturas de alcance infinito, que es lo que Nav2 necesita
   para hacer raytracing y **limpiar** el costmap.
2. **sectores ciegos -> NaN**: los angulos ocluidos por la propia
   estructura del robot (mastil, brazo, cables) se marcan como invalidos
   de verdad. NaN no marca obstaculo y tampoco limpia, que es lo
   correcto: ahi no sabemos nada.
3. **recorte de alcance**: limita el rango util al tamano del escenario
   (el laberinto es de 4.5 x 3.0 m). Lecturas lejanas y rasantes son
   ruido y generan marcas espurias.
4. **filtro de motas (speckle)**: elimina puntos aislados sin vecinos
   coherentes, tipicos de reflejos en aristas. Se convierten en +inf
   para que ademas limpien.
5. **diagnostico de sectores ciegos**: con ``report_blind_sectors:=true``
   publica en el log un histograma de que angulos devuelven cero de forma
   permanente. Sirve para calibrar ``blind_sectors_deg`` sin mover el
   robot.

El resultado se publica en un topic aparte (``/scan_filtered`` por
defecto) para no interferir con nada que ya consuma ``/scan``.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class ScanSanitizer(Node):

    def __init__(self):
        super().__init__('scan_sanitizer')

        # -----------------------------------------------------------------
        # Topics
        # -----------------------------------------------------------------
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')

        # -----------------------------------------------------------------
        # Alcance util
        # -----------------------------------------------------------------
        # Por debajo de range_min son impactos con el propio robot.
        self.declare_parameter('range_min', 0.16)
        # El laberinto mide 4.5 x 3.0 m: la diagonal es ~5.4 m. Limitar
        # a 5.0 m descarta ruido lejano sin perder informacion util.
        self.declare_parameter('range_max', 5.0)

        # -----------------------------------------------------------------
        # Sectores ciegos (grados, en el frame del laser)
        # -----------------------------------------------------------------
        # Pares [inicio, fin] achatados en una sola lista:
        #   [-50.0, 50.0]  -> un sector de -50 a +50 grados
        #   [-50.0, 50.0, 170.0, 190.0] -> dos sectores
        #
        # Por defecto coincide con el 'ignore_array: "-50,50"' del driver.
        # VERIFICA cual es realmente el sector ocluido en tu robot con
        # report_blind_sectors:=true antes de confiar en este valor.
        self.declare_parameter('blind_sectors_deg', [-50.0, 50.0])

        # -----------------------------------------------------------------
        # Conversion de ceros
        # -----------------------------------------------------------------
        self.declare_parameter('zeros_to_inf', True)

        # -----------------------------------------------------------------
        # Filtro de motas
        # -----------------------------------------------------------------
        self.declare_parameter('speckle_filter', True)
        # Numero de vecinos a cada lado que se inspeccionan.
        self.declare_parameter('speckle_window', 2)
        # Un punto sobrevive si algun vecino esta a menos de esta
        # distancia radial (m).
        self.declare_parameter('speckle_threshold', 0.12)

        # -----------------------------------------------------------------
        # Diagnostico
        # -----------------------------------------------------------------
        self.declare_parameter('report_blind_sectors', False)
        self.declare_parameter('report_period_sec', 5.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)

        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.blind_sectors = self._parse_sectors(
            self.get_parameter('blind_sectors_deg').value
        )

        self.zeros_to_inf = bool(self.get_parameter('zeros_to_inf').value)

        self.speckle_filter = bool(
            self.get_parameter('speckle_filter').value
        )
        self.speckle_window = int(
            self.get_parameter('speckle_window').value
        )
        self.speckle_threshold = float(
            self.get_parameter('speckle_threshold').value
        )

        self.report_blind = bool(
            self.get_parameter('report_blind_sectors').value
        )
        self.report_period = float(
            self.get_parameter('report_period_sec').value
        )

        # Mascara de sectores ciegos, cacheada por geometria del scan.
        self._blind_mask = None
        self._mask_signature = None

        # Acumuladores del diagnostico.
        self._zero_hits = None
        self._total_scans = 0

        self.publisher = self.create_publisher(
            LaserScan, self.output_topic, qos_profile_sensor_data
        )

        self.subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        if self.report_blind:
            self.create_timer(self.report_period, self._report)

        self.get_logger().info(
            f'scan_sanitizer: {self.input_topic} -> {self.output_topic}'
        )
        self.get_logger().info(
            f'rango util [{self.range_min:.2f}, {self.range_max:.2f}] m, '
            f'ceros->inf={self.zeros_to_inf}, '
            f'speckle={self.speckle_filter}'
        )
        if self.blind_sectors:
            sectors = ', '.join(
                f'[{math.degrees(a):.0f}, {math.degrees(b):.0f}]'
                for a, b in self.blind_sectors
            )
            self.get_logger().info(f'sectores ciegos (grados): {sectors}')
        else:
            self.get_logger().info('sin sectores ciegos configurados')

    # =====================================================================
    # Utilidades
    # =====================================================================

    @staticmethod
    def _parse_sectors(raw):
        """Convierte una lista plana de grados en pares de radianes."""
        if raw is None:
            return []

        values = [float(v) for v in raw]
        if len(values) % 2 != 0:
            values = values[:-1]

        sectors = []
        for i in range(0, len(values), 2):
            start = math.radians(values[i])
            end = math.radians(values[i + 1])
            if start > end:
                start, end = end, start
            sectors.append((start, end))
        return sectors

    def _build_blind_mask(self, msg):
        """Mascara booleana True donde el haz cae en un sector ciego."""
        count = len(msg.ranges)
        signature = (
            count,
            round(msg.angle_min, 6),
            round(msg.angle_increment, 8),
        )

        if self._mask_signature == signature:
            return self._blind_mask

        angles = msg.angle_min + np.arange(count) * msg.angle_increment
        # Normaliza a [-pi, pi] para comparar con los sectores.
        angles = np.arctan2(np.sin(angles), np.cos(angles))

        mask = np.zeros(count, dtype=bool)
        for start, end in self.blind_sectors:
            mask |= (angles >= start) & (angles <= end)

        self._blind_mask = mask
        self._mask_signature = signature
        return mask

    def _remove_speckles(self, ranges, valid):
        """Marca como invalidos los retornos aislados.

        Un punto valido necesita al menos un vecino (dentro de la
        ventana) cuyo rango difiera menos de speckle_threshold.
        """
        count = ranges.shape[0]
        if count == 0 or self.speckle_window < 1:
            return valid

        keep = np.zeros(count, dtype=bool)

        for shift in range(1, self.speckle_window + 1):
            for signed in (shift, -shift):
                neighbour = np.roll(ranges, signed)
                neighbour_valid = np.roll(valid, signed)
                close = np.abs(ranges - neighbour) < self.speckle_threshold
                keep |= close & neighbour_valid

        return valid & keep

    # =====================================================================
    # Procesado
    # =====================================================================

    def scan_callback(self, msg):
        count = len(msg.ranges)
        if count == 0:
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)

        blind = self._build_blind_mask(msg)

        # Diagnostico: cuenta ceros por haz.
        if self.report_blind:
            if self._zero_hits is None or self._zero_hits.shape[0] != count:
                self._zero_hits = np.zeros(count, dtype=np.int64)
                self._total_scans = 0
            self._zero_hits += (ranges <= 1e-6)
            self._total_scans += 1

        finite = np.isfinite(ranges)

        # "Sin retorno": cero exacto, no finito, o fuera del rango util.
        no_return = (
            (~finite)
            | (ranges <= 1e-6)
            | (ranges > self.range_max)
        )

        # Impacto contra el propio robot: demasiado cerca.
        too_close = finite & (ranges > 1e-6) & (ranges < self.range_min)

        out = ranges.copy()

        # 1. Todo lo que es "sin retorno" pasa a +inf para que Nav2
        #    haga raytracing y LIMPIE el costmap.
        if self.zeros_to_inf:
            out[no_return] = math.inf
        else:
            out[no_return] = np.nan

        # 2. Lo que golpea el propio robot es informacion inutil: NaN.
        #    No marca y no limpia.
        out[too_close] = np.nan

        # 3. Filtro de motas sobre los retornos que siguen siendo validos.
        if self.speckle_filter:
            valid = np.isfinite(out)
            kept = self._remove_speckles(out, valid)
            speckles = valid & ~kept
            # Una mota es ruido, no un obstaculo: dejar que limpie.
            out[speckles] = math.inf

        # 4. Sectores ciegos: NaN SIEMPRE. Nunca marcar ni limpiar ahi,
        #    porque no tenemos informacion real de esa direccion.
        out[blind] = math.nan

        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = float(self.range_min)
        filtered.range_max = float(self.range_max)
        filtered.ranges = out.astype(np.float32).tolist()
        # Las intensidades pierden sentido tras el filtrado.
        filtered.intensities = []

        self.publisher.publish(filtered)

    # =====================================================================
    # Diagnostico de sectores ciegos
    # =====================================================================

    def _report(self):
        if self._zero_hits is None or self._total_scans == 0:
            return

        ratio = self._zero_hits / float(self._total_scans)
        count = ratio.shape[0]

        # Agrupa en cubetas de 10 grados para que el log sea legible.
        bucket_deg = 10.0
        buckets = {}
        for i in range(count):
            angle = math.degrees(
                self._mask_signature[1] + i * self._mask_signature[2]
            )
            angle = (angle + 180.0) % 360.0 - 180.0
            key = int(math.floor(angle / bucket_deg) * bucket_deg)
            buckets.setdefault(key, []).append(ratio[i])

        permanent = [
            (key, float(np.mean(values)))
            for key, values in sorted(buckets.items())
            if float(np.mean(values)) > 0.9
        ]

        if permanent:
            text = ', '.join(
                f'{key}..{key + int(bucket_deg)} deg'
                for key, _ in permanent
            )
            self.get_logger().info(
                f'[diagnostico] sectores con >90% de haces sin retorno '
                f'({self._total_scans} scans): {text}. '
                f'Si son fijos aunque el robot se mueva, son oclusiones '
                f'del propio robot: ponlos en blind_sectors_deg.'
            )
        else:
            self.get_logger().info(
                f'[diagnostico] ningun sector con >90% de haces sin '
                f'retorno en {self._total_scans} scans.'
            )

        self._zero_hits[:] = 0
        self._total_scans = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScanSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
