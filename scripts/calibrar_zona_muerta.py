#!/usr/bin/env python3
"""Mide la ZONA MUERTA real de los motores, con la odometria del robot.

Por que existe: min_lateral_speed y min_linear_speed llevaban rondas
siendo ESTIMACIONES, y todas fallaron. Se dio por hecho tres veces que
la zona muerta era la causa de que el robot no se moviera, y se subieron
los topes a ciegas cada vez.

Medido por fin en el robot: se mueve ya a 0.02 m/s. La zona muerta es
DESPRECIABLE y nunca fue la causa. Que el robot pareciera parado con
mando distinto de cero era otra cosa (mando alternando de signo por el
castañeo del giro, y la direccion torcida por aplicar la zona muerta
eje a eje). Esto esta aqui para que ese numero deje de suponerse.

Esto no lo estima: lo mide. Manda una rampa de velocidades por la MISMA
cadena que usa la aproximacion (/cmd_vel_aruco -> twist_mux -> /cmd_vel)
y mira /odom para ver a partir de que mando se mueve de verdad.

USO
    # con la pila del robot levantada y DISTRIBUTED=1 en la Jetson
    eval "$(WIFI_IFACE=eth3 ROBOT_IP=... LAPTOP_IP=... \\
            ./scripts/tsummit_offboard.sh env)"

    ALLOW_MOTION=1 python3 scripts/calibrar_zona_muerta.py            # los 3 ejes
    ALLOW_MOTION=1 python3 scripts/calibrar_zona_muerta.py --eje avance
    ALLOW_MOTION=1 python3 scripts/calibrar_zona_muerta.py --max 0.40

ESTO MUEVE EL ROBOT. Ruedas en el suelo, medio metro libre por delante
y a los lados, y con Ctrl-C a mano. Al soltarlo se para solo: twist_mux
corta a los 0.5 s sin mensajes y el watchdog de myagv_odometry a los
300 ms.
"""

import argparse
import math
import os
import sys
import time

import rclpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


EJES = ('avance', 'lateral', 'giro')


class Calibrador(Node):

    def __init__(self, cmd_topic, odom_topic):
        super().__init__('calibrar_zona_muerta')

        self.pub = self.create_publisher(Twist, cmd_topic, 10)

        self.create_subscription(
            Odometry, odom_topic, self._odom, qos_profile_sensor_data
        )

        self.odom = None
        self.odom_count = 0

    def _odom(self, msg):
        self.odom = msg
        self.odom_count += 1

    # -----------------------------------------------------------------

    def pose(self):
        """(x, y, yaw) o None."""
        if self.odom is None:
            return None

        p = self.odom.pose.pose
        q = p.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        return p.position.x, p.position.y, yaw

    def publicar(self, eje, valor):
        cmd = Twist()

        if eje == 'avance':
            cmd.linear.x = valor
        elif eje == 'lateral':
            cmd.linear.y = valor
        else:
            cmd.angular.z = valor

        self.pub.publish(cmd)

    def parar(self):
        for _ in range(10):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    # -----------------------------------------------------------------

    def medir(self, eje, valor, duracion, rate=20.0):
        """Manda `valor` durante `duracion` y devuelve el desplazamiento.

        Se mide el DESPLAZAMIENTO integrado, no la velocidad instantanea
        de /odom: el driver puede publicar la velocidad que le pidieron
        en vez de la que consiguio, y entonces la medida diria que se
        mueve cuando las ruedas estan paradas. La posicion no miente.
        """
        inicio = self.pose()

        if inicio is None:
            return None

        period = 1.0 / rate
        fin_t = time.time() + duracion

        while time.time() < fin_t and rclpy.ok():
            self.publicar(eje, valor)
            rclpy.spin_once(self, timeout_sec=period)

        self.parar()

        for _ in range(6):
            rclpy.spin_once(self, timeout_sec=0.05)

        final = self.pose()

        if final is None:
            return None

        if eje == 'giro':
            d = abs(math.atan2(
                math.sin(final[2] - inicio[2]),
                math.cos(final[2] - inicio[2]),
            ))
        else:
            d = math.hypot(final[0] - inicio[0], final[1] - inicio[1])

        return d


def calibrar_eje(nodo, eje, args):
    unidad = 'rad' if eje == 'giro' else 'm'
    vunidad = 'rad/s' if eje == 'giro' else 'm/s'

    paso = args.paso_giro if eje == 'giro' else args.paso
    inicio = args.min_giro if eje == 'giro' else args.min
    tope = args.max_giro if eje == 'giro' else args.max

    print()
    print(f"  === {eje.upper()} ===")

    # Ruido de fondo: cuanto se "mueve" la odometria con el robot quieto.
    quieto = nodo.medir(eje, 0.0, args.ventana)

    if quieto is None:
        print("    ERROR: sin /odom. La pila del robot no esta publicando.")
        return None

    umbral_ruido = max(quieto * 3.0, args.ruido_giro
                       if eje == 'giro' else args.ruido)

    print(f"    ruido con el robot quieto: {quieto:.4f} {unidad}"
          f"  -> umbral {umbral_ruido:.4f} {unidad}")
    print()

    def se_mueve(valor):
        d = nodo.medir(eje, valor, args.ventana)

        if d is None:
            return None, None

        movio = d > umbral_ruido

        print(f"    {valor:6.3f} {vunidad:6s}  ->  {d:7.4f} {unidad}   "
              f"{'SE MUEVE' if movio else '  --    '}")

        return movio, d

    # Acotar de verdad, en las dos direcciones.
    #
    # Antes solo subia desde --min, asi que si se movia ya en el primer
    # escalon daba ESE valor como umbral. No lo es: es solo el primer
    # peldaño de la escalera. Lo unico que dice es que el umbral esta
    # en ese valor o por debajo, y hay que seguir BAJANDO para saberlo.
    movio, _ = se_mueve(inicio)

    if movio is None:
        print("    ERROR: se perdio /odom a mitad.")
        return None

    encontrado = None

    if movio:

        # Se movio a la primera: bajar hasta que deje de moverse.
        print(f"\n    se mueve ya en el primer escalon; bajando\n")

        encontrado = inicio
        valor = inicio - paso

        while valor > 1e-6:

            movio, _ = se_mueve(valor)

            if movio is None:
                return None

            if not movio:
                break

            encontrado = valor
            valor -= paso

        if encontrado <= paso + 1e-9:
            print()
            print(f"    Se mueve hasta el escalon mas bajo probado")
            print(f"    ({encontrado:.3f} {vunidad}). La zona muerta es")
            print(f"    DESPRECIABLE en este eje: no es lo que impide")
            print(f"    que el robot se mueva. Busca la causa en otro")
            print(f"    sitio (mando que llega, direccion, o control).")

    else:

        # No se movio: subir hasta que se mueva.
        valor = inicio + paso

        while valor <= tope + 1e-9:

            movio, _ = se_mueve(valor)

            if movio is None:
                return None

            if movio:
                encontrado = valor
                break

            valor += paso

    print()

    if encontrado is None:
        print(f"    NO se movio hasta {tope:.3f} {vunidad}.")
        print("    Sube --max, o revisa que twist_mux este suscrito a")
        print("    /cmd_vel_aruco y que la bateria no este baja.")
        return None

    margen = encontrado * 1.20

    print(f"    zona muerta medida : {encontrado:.3f} {vunidad}")
    print(f"    valor recomendado  : {margen:.3f} {vunidad}  (+20% de margen)")

    if encontrado <= paso + 1e-9:
        print()
        print("    OJO: esto es un TECHO, no una medida. El barrido no")
        print(f"    bajo de {encontrado:.3f}. Repite con --paso mas fino")
        print("    y --min mas bajo si quieres el numero exacto.")

    return encontrado, margen


def main():
    parser = argparse.ArgumentParser(
        description='Mide la zona muerta real de los motores.'
    )

    parser.add_argument('--eje', choices=EJES + ('todos',), default='todos')
    parser.add_argument('--cmd-topic', default='/cmd_vel_aruco')
    parser.add_argument('--odom-topic', default='/odom')

    parser.add_argument('--min', type=float, default=0.02)
    parser.add_argument('--max', type=float, default=0.30)
    parser.add_argument('--paso', type=float, default=0.01)
    parser.add_argument('--ruido', type=float, default=0.010,
                        help='desplazamiento minimo que cuenta como '
                             'movimiento, en metros')

    parser.add_argument('--min-giro', type=float, default=0.05)
    parser.add_argument('--max-giro', type=float, default=1.20)
    parser.add_argument('--paso-giro', type=float, default=0.05)
    parser.add_argument('--ruido-giro', type=float, default=0.035,
                        help='giro minimo que cuenta como movimiento, '
                             'en radianes')

    parser.add_argument('--ventana', type=float, default=1.5,
                        help='segundos que se manda cada valor')

    args = parser.parse_args()

    if os.environ.get('ALLOW_MOTION') != '1':
        print("ESTO MUEVE EL ROBOT.")
        print("Ruedas en el suelo, medio metro libre alrededor, y con")
        print("Ctrl-C a mano. Cuando lo tengas:")
        print()
        print(f"    ALLOW_MOTION=1 {' '.join(sys.argv)}")
        return 2

    rclpy.init()
    nodo = Calibrador(args.cmd_topic, args.odom_topic)

    print("=" * 62)
    print("  CALIBRACION DE LA ZONA MUERTA")
    print("=" * 62)
    print(f"  mando por  {args.cmd_topic}   (la misma cadena que la")
    print(f"             aproximacion: twist_mux -> /cmd_vel)")
    print(f"  mido por   {args.odom_topic}")
    print()
    print("  OJO: /odom se calcula con los ENCODERS. Mide que las ruedas")
    print("  giren, no que el robot avance. Si patinan, esto dira que se")
    print("  mueve. Miralo tambien con los ojos.")

    print("\n  esperando /odom ...", end='', flush=True)

    espera = time.time() + 10.0

    while nodo.odom is None and time.time() < espera and rclpy.ok():
        rclpy.spin_once(nodo, timeout_sec=0.1)

    if nodo.odom is None:
        print(" NADA.")
        print()
        print("  /odom no publica. Comprueba desde el portatil:")
        print("      ros2 topic info /odom      -> Publisher count >= 1")
        print("  Si sale 0, la Jetson arranco SIN DISTRIBUTED=1 y su DDS")
        print("  esta en loopback.")
        nodo.destroy_node()
        rclpy.shutdown()
        return 1

    print(f" ok ({nodo.odom_count} mensajes)")

    ejes = EJES if args.eje == 'todos' else (args.eje,)
    resultados = {}

    try:
        for eje in ejes:
            r = calibrar_eje(nodo, eje, args)

            if r is not None:
                resultados[eje] = r

    except KeyboardInterrupt:
        print("\n  interrumpido")

    finally:
        nodo.parar()

    print()
    print("=" * 62)

    if resultados:
        print("  PARA EL LAUNCH")
        print()

        nombres = {
            'avance': 'min_linear_speed',
            'lateral': 'min_lateral_speed',
            'giro': 'min_heading_speed',
        }

        topes = {
            'avance': 'max_linear_speed',
            'lateral': 'max_lateral_speed',
            'giro': 'max_heading_speed',
        }

        linea = []

        for eje, (medido, rec) in resultados.items():
            linea.append(f"{nombres[eje]}:={rec:.3f}")

            # El techo tiene que estar MUY por encima del suelo, o el
            # controlador se queda sin margen util: es justo lo que
            # paso con max_linear_speed=0.08 y la zona muerta encima.
            linea.append(f"{topes[eje]}:={max(rec * 2.5, rec + 0.06):.3f}")

        print("    ./scripts/tsummit_offboard.sh run \\")
        print("        " + " \\\n        ".join(linea))
        print()
        print("  Si funciona, commitea estos numeros: son el unico dato")
        print("  de todo esto que no se puede deducir leyendo el codigo.")
    else:
        print("  Sin resultados.")

    print("=" * 62)

    nodo.destroy_node()
    rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
