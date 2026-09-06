#!/usr/bin/env python3
"""Curva de respuesta: velocidad PEDIDA contra velocidad CONSEGUIDA.

Complementa a calibrar_zona_muerta.py, que responde "a partir de que
mando se mueve". Esta responde la que de verdad importa para el control:
"y una vez que se mueve, se mueve a lo que le pido?".

MEDIDO EN EL ROBOT (2026-09-06, ruedas en el suelo, ventana de 1-2 s):

    eje       suelo real     modula desde     mandos que dan el suelo
    giro      0.37 rad/s     ~0.45 rad/s      0.02 a 0.40  (20x)
    avance    0.07 m/s       ~0.12 m/s        0.02 a 0.08  (4x)
    lateral   0.035 m/s      ~0.12 m/s        0.02 a 0.08  (4x)

La base NO baja de esas velocidades. Todo mando por debajo se sirve como
el suelo. O sea que min_heading_speed=0.08 es una ficcion: pidas 0.08 o
pidas 0.30, el robot gira a 0.37.

POR QUE IMPORTA, y es el hallazgo de verdad:

El giro tiene suelo 0.37 rad/s y la tuberia offboard mete ~200 ms de
retardo. En lo que llega la orden de parar el robot ya ha girado
0.37*0.2 = 0.074 rad = 4.2 grados. Con heading_tolerance en 0.08 rad
(4.6 grados), el sobrepasamiento se come la banda ENTERA: el robot no
puede paramerse dentro de la tolerancia, sale por el otro lado y
corrige al reves. Ese es el baile izquierda-derecha, y explica por que
la histeresis lo alivia sin curarlo -- su umbral de salida es
1.5*0.08 = 0.12 rad, del mismo orden que el sobrepasamiento.

La tolerancia tiene que ser MAYOR que el sobrepasamiento inevitable:
retardo (0.074) + un ciclo de control (0.037 a 10 Hz) = 0.111 rad como
minimo absoluto. De ahi el 0.15 que se usa ahora.

El arreglo de fondo seria pulsar el giro (mandar 0.37 una fraccion del
ciclo) para conseguir velocidades efectivas menores. Sin probar.

OJO: /odom sale de los ENCODERS. En el eje lateral las ruedas mecanum
patinan bastante, asi que el suelo lateral medido es optimista.

USO
    python3 scripts/curva_respuesta.py <eje> <mandos> [ventana_s]
    python3 scripts/curva_respuesta.py giro 0.02,0.30,0.45,0.70 2.0
    python3 scripts/curva_respuesta.py avance 0.02,0.05,0.12,0.25 1.0

ESTO MUEVE EL ROBOT. Ruedas en el suelo y espacio libre. En los ejes
lineales alterna el sentido en cada escalon para no escaparse.
"""
import math, time, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

EJE = sys.argv[1] if len(sys.argv) > 1 else 'giro'
MANDOS = [float(x) for x in sys.argv[2].split(',')]
VENTANA = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

def yaw(o):
    return math.atan2(2.0*(o.w*o.z + o.x*o.y), 1.0 - 2.0*(o.y*o.y + o.z*o.z))

class Curva(Node):
    def __init__(self):
        super().__init__('curva_respuesta')
        self.pub = self.create_publisher(Twist, '/cmd_vel_aruco', 10)
        self.odom = None
        self.create_subscription(Odometry, '/odom', self.cb, 10)
    def cb(self, m): self.odom = m

    def espera_odom(self):
        for _ in range(200):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom: return True
        return False

    def estado(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, yaw(p.orientation)

    def parar(self):
        for _ in range(10):
            self.pub.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.6)
        for _ in range(20): rclpy.spin_once(self, timeout_sec=0.02)

    def prueba(self, mando, signo=1.0):
        self.parar()
        x0, y0, a0 = self.estado()
        t0 = time.time()
        msg = Twist()
        if EJE == 'giro':    msg.angular.z = mando * signo
        elif EJE == 'avance': msg.linear.x  = mando * signo
        else:                 msg.linear.y  = mando * signo
        while time.time() - t0 < VENTANA:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        dt = time.time() - t0
        self.parar()
        x1, y1, a1 = self.estado()
        if EJE == 'giro':
            d = abs(math.atan2(math.sin(a1-a0), math.cos(a1-a0)))
        else:
            d = math.hypot(x1-x0, y1-y0)
        return d, d/dt

rclpy.init()
n = Curva()
if not n.espera_odom():
    print("sin /odom"); sys.exit(1)

u = 'rad' if EJE=='giro' else 'm'
print(f"\n  eje: {EJE}   ventana: {VENTANA}s\n")
print(f"  {'pedido':>10}  {'recorrido':>12}  {'conseguido':>12}  {'ratio':>8}")
print("  " + "-"*48)
res = []
for i, m in enumerate(MANDOS):
    d, v = n.prueba(m, 1.0 if i % 2 == 0 else -1.0)
    ratio = v/m if m else float('nan')
    res.append((m, v, ratio))
    print(f"  {m:10.3f}  {d:9.4f} {u}  {v:9.4f} {u}/s  {ratio:7.1f}x")

n.parar()
print()
# El suelo: el mando mas alto cuya velocidad conseguida sigue pegada a
# la de los mandos MUY por debajo. Por debajo del suelo la base no
# obedece, entrega el suelo.
movidos = [r for r in res if r[1] > 0.02]
if len(movidos) >= 3:
    vs = [r[1] for r in movidos]
    suelo = min(vs)
    # cuantos mandos distintos entregan (casi) esa misma velocidad
    planos = [r for r in movidos if r[1] < suelo * 1.25]
    print(f"  suelo de velocidad: {suelo:.3f} {u}/s")
    if len(planos) >= 2:
        lo = min(r[0] for r in planos); hi = max(r[0] for r in planos)
        print(f"  mandos de {lo:.3f} a {hi:.3f} ({hi/lo:.0f}x de rango) entregan todos ~{suelo:.2f} {u}/s")
        print(f"\n  LA BASE NO BAJA DE {suelo:.2f} {u}/s. Todo mando por debajo")
        print(f"  se sirve como {suelo:.2f}. El rango util del control empieza ahi,")
        print(f"  no en min_speed. Un paso de control dura {suelo*0.1:.3f} {u} a 10 Hz.")
    else:
        print("\n  La base modula desde el primer escalon.")
    sigue = [r for r in movidos if r[1] >= suelo*1.25]
    if sigue:
        print(f"  modula de verdad a partir de {min(r[0] for r in sigue):.3f}")
rclpy.shutdown()
