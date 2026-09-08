#!/usr/bin/env python3
"""Planificador de aproximacion a un ArUco para una base HOLONOMA.

Geometria y control puros: ni rclpy, ni TF, ni topics. Todo lo de aqui
se puede probar en un portatil sin robot, que es justo lo que hacia
falta -- el algoritmo viejo solo se podia evaluar en pista.

POR QUE NO PURE PURSUIT TAL CUAL
--------------------------------
Pure Pursuit calcula una CURVATURA de direccion: nacio para vehiculos
no holonomos (Ackermann, diferencial), que no pueden desplazarse de
lado y solo controlan avance y giro. El myAGV es mecanum: controla vx,
vy y wz de forma independiente. Reducirlo a curvatura es tirar un grado
de libertad entero.

Lo que SI vale de Pure Pursuit es el punto de anticipacion: en vez de
apuntar al final del camino, se persigue un punto que se desliza por
delante sobre la trayectoria. Eso da un movimiento suave, sin recortar
esquinas y sin el latigazo de un proporcional puro cerca del objetivo.

Asi que aqui: carrot de Pure Pursuit + accionamiento holonomo de 3 GDL.

EL MARCO DE TRABAJO ES ODOM, NO EL ROBOT
----------------------------------------
El fallo de fondo del control anterior era medir el error RELATIVO al
robot en cada ciclo. La deteccion nace en la Jetson, se comprime, cruza
el WiFi y se procesa en el portatil: cuando llega, describe donde estaba
el marcador hace ~200 ms. Con el robot en movimiento ese retardo se
realimenta y produce sobreoscilacion (el "desface").

Aqui el marcador se fija UNA VEZ en odom y a partir de ahi el robot
navega con su PROPIA odometria, que es local, rapida y continua. Las
detecciones dejan de ser el lazo de control y pasan a ser correcciones
lentas de un estimador. Perder el marcador un segundo deja de importar.

CONVENIO DE NORMALES
--------------------
`get_marker_normal()` y `get_lidar_surface_normal()` del servidor
devuelven la normal en odom apuntando DEL ROBOT HACIA la superficie.
Aqui se trabaja con la normal SALIENTE del marcador (del marcador hacia
el espacio libre), que es la opuesta. `outward_normal()` hace la
conversion; usala siempre en la frontera en vez de repartir signos.
"""

import math
import statistics


# ---------------------------------------------------------------------
# Utilidades
# ---------------------------------------------------------------------

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def angle_lerp(a, b, t):
    """Interpola de a a b por el camino corto.

    Interpolar angulos linealmente es un error clasico: entre 170 y
    -170 grados el camino corto son 20 grados, no 340. Se interpola
    sobre la DIFERENCIA normalizada.
    """
    return normalize_angle(
        a + t * normalize_angle(b - a)
    )


def outward_normal(nx, ny):
    """De 'normal hacia la superficie' a 'normal saliente del marcador'."""
    return -nx, -ny


# ---------------------------------------------------------------------
# Estimador del marcador en odom
# ---------------------------------------------------------------------

class TargetEstimate:
    """Pose del marcador en odom, fusionada a lo largo del tiempo.

    Filtro exponencial, no media simple: las muestras recientes valen
    mas, pero una deteccion suelta y ruidosa no arrastra la estimacion.

    La normal se promedia como VECTOR y se renormaliza, nunca como
    angulo: promediar angulos falla en el cruce de +-pi y ahi es
    justo donde cae un marcador visto de frente segun como quede odom.
    """

    def __init__(
        self,
        alpha_position=0.35,
        alpha_normal=0.25,
        max_normal_jump=None,
        gate_after=5,
        relock_after=12,
    ):
        self.alpha_position = alpha_position
        self.alpha_normal = alpha_normal

        # Rechazo de valores atipicos en la normal. La pose de un ArUco
        # plano es AMBIGUA: para un marcador visto casi de frente hay
        # dos soluciones simetricas respecto a la linea de vision, y el
        # estimador de OpenCV salta entre ellas de un fotograma a otro.
        # Su eje Z -- la normal -- salta con ellas, el rumbo objetivo
        # salta detras, y el robot gira a izquierda y derecha
        # intermitentemente persiguiendolo. Corregir el signo no basta:
        # las dos soluciones apuntan hacia el robot.
        self.max_normal_jump = max_normal_jump

        # No se filtra desde la primera muestra: hasta tener unas
        # cuantas, la estimacion es tan provisional como lo que llega.
        self.gate_after = gate_after

        # Si se rechaza demasiado seguido, la equivocada es la
        # estimacion. Se reinicia en vez de atrincherarse en el error.
        self.relock_after = relock_after

        self.x = None
        self.y = None
        self.nx = None
        self.ny = None

        self.samples = 0
        self.rejected = 0
        self.consecutive_rejected = 0
        self.last_update_ns = None

    @property
    def ready(self):
        return self.samples > 0 and self.x is not None

    def update(self, x, y, nx, ny, stamp_ns=None, alpha_scale=1.0):
        """Incorpora una observacion (marcador en odom, normal saliente).

        `alpha_scale` permite pesar la muestra: el servidor la usa para
        dar mas peso a la normal del LiDAR que a la del ArUco, que es
        mucho mas ruidosa en yaw.
        """
        norm = math.hypot(nx, ny)

        if norm < 1e-9:
            return False

        nx /= norm
        ny /= norm

        if (
            self.ready and
            self.max_normal_jump is not None and
            self.samples >= self.gate_after
        ):

            dot = clamp(self.nx * nx + self.ny * ny, -1.0, 1.0)

            if math.acos(dot) > self.max_normal_jump:

                self.rejected += 1
                self.consecutive_rejected += 1

                # Rechazar sin fin significaria que la equivocada es la
                # estimacion, no las muestras. Reengancharse a la nueva
                # es mejor que atrincherarse en la vieja.
                if self.consecutive_rejected >= self.relock_after:
                    self.x, self.y = float(x), float(y)
                    self.nx, self.ny = nx, ny
                    self.samples = 1
                    self.consecutive_rejected = 0

                    if stamp_ns is not None:
                        self.last_update_ns = stamp_ns

                    return True

                return False

        self.consecutive_rejected = 0

        if not self.ready:
            self.x, self.y = float(x), float(y)
            self.nx, self.ny = nx, ny

        else:
            # Alfa adaptativo. El marcador NO se mueve, asi que al
            # principio lo optimo es una media corriente (1/n): cada
            # muestra pesa lo mismo y el ruido baja como 1/sqrt(n).
            # Pasadas unas cuantas, el alfa fijo hace de suelo para
            # poder seguir correcciones lentas (deriva de odometria,
            # o la normal del LiDAR entrando en juego al acercarse).
            running = 1.0 / (self.samples + 1.0)

            ap = clamp(
                max(self.alpha_position, running) * alpha_scale,
                0.0, 1.0,
            )

            an = clamp(
                max(self.alpha_normal, running) * alpha_scale,
                0.0, 1.0,
            )

            self.x += ap * (float(x) - self.x)
            self.y += ap * (float(y) - self.y)

            mixed_x = self.nx + an * (nx - self.nx)
            mixed_y = self.ny + an * (ny - self.ny)

            mixed_norm = math.hypot(mixed_x, mixed_y)

            # Solo puede anularse si la normal nueva es opuesta a la
            # acumulada. Eso no es ruido, es un cambio de signo: se
            # descarta la muestra en vez de dividir por cero.
            if mixed_norm > 1e-6:
                self.nx = mixed_x / mixed_norm
                self.ny = mixed_y / mixed_norm

        self.samples += 1

        if stamp_ns is not None:
            self.last_update_ns = stamp_ns

        return True

    @property
    def pose(self):
        return self.x, self.y, self.nx, self.ny

    def age_sec(self, now_ns):
        if self.last_update_ns is None:
            return float('inf')

        return (now_ns - self.last_update_ns) / 1e9


# ---------------------------------------------------------------------
# Poses derivadas
# ---------------------------------------------------------------------

def staging_pose(mx, my, nx, ny, standoff):
    """Pose de encare: sobre la normal, a `standoff` del marcador.

    Es el punto desde el que la aproximacion final es una recta
    perpendicular a la superficie. `nx, ny` es la normal SALIENTE.
    El yaw mira HACIA el marcador, o sea a lo largo de -normal.
    """
    return (
        mx + nx * standoff,
        my + ny * standoff,
        math.atan2(-ny, -nx),
    )


def corridor_coords(rx, ry, mx, my, nx, ny):
    """Coordenadas del robot en el marco del marcador.

    Devuelve (avance, lateral):
      avance  = distancia a lo largo de la normal saliente. Es la
                separacion perpendicular real a la superficie.
      lateral = separacion respecto a la recta de la normal, con signo.

    Son las dos magnitudes que de verdad importan para aproximarse a un
    plano, y las que el control anterior mezclaba en un unico "error de
    centrado" normalizado a [-1, 1] sin unidades fisicas.
    """
    dx, dy = rx - mx, ry - my

    return (
        dx * nx + dy * ny,
        -dx * ny + dy * nx,
    )


def build_path(
    rx, ry, mx, my, nx, ny,
    standoff, stop_distance,
    corridor_radius=0.12,
):
    """Camino hasta el marcador, de uno o dos tramos.

    Dos tramos (robot -> encare -> parada) mientras el robot esta fuera
    del pasillo de aproximacion. El segundo va sobre la normal, asi que
    la llegada es perpendicular POR CONSTRUCCION: no hace falta un
    estado aparte que alinee, que era de donde salia el baile
    alinear -> avanzar -> desalinear -> realinear.

    Un solo tramo (recta al final) en cuanto el robot ya esta dentro
    del pasillo: bastante centrado sobre la normal y mas cerca que el
    punto de encare. Sin esto el camino incluiria para siempre el
    rodeo por el punto de encare, `remaining` no bajaria nunca de
    standoff - stop_distance, y la llegada no se declararia jamas.
    """
    sx, sy, _ = staging_pose(mx, my, nx, ny, standoff)

    fx = mx + nx * stop_distance
    fy = my + ny * stop_distance

    along, lateral = corridor_coords(rx, ry, mx, my, nx, ny)

    inside = (
        along <= standoff + 1e-3 and
        abs(lateral) <= corridor_radius
    )

    if inside:
        return [(rx, ry), (fx, fy)]

    if math.hypot(sx - rx, sy - ry) < 1e-3:
        return [(sx, sy), (fx, fy)]

    return [(rx, ry), (sx, sy), (fx, fy)]


# ---------------------------------------------------------------------
# Seguimiento del camino (el carrot de Pure Pursuit)
# ---------------------------------------------------------------------

def path_length(path):
    total = 0.0

    for i in range(len(path) - 1):
        total += math.hypot(
            path[i + 1][0] - path[i][0],
            path[i + 1][1] - path[i][1],
        )

    return total


def project_on_path(path, x, y):
    """Punto del camino mas cercano a (x, y).

    Devuelve (distancia_recorrida, distancia_lateral, (px, py)).
    `distancia_recorrida` se mide desde el inicio del camino, y es lo
    que permite luego avanzar el carrot una longitud de anticipacion.
    """
    best = None
    travelled = 0.0

    for i in range(len(path) - 1):

        ax, ay = path[i]
        bx, by = path[i + 1]

        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy

        if seg_len2 < 1e-12:
            continue

        t = clamp(
            ((x - ax) * dx + (y - ay) * dy) / seg_len2,
            0.0,
            1.0,
        )

        px, py = ax + t * dx, ay + t * dy
        lateral = math.hypot(x - px, y - py)

        seg_len = math.sqrt(seg_len2)
        along = travelled + t * seg_len

        if best is None or lateral < best[1]:
            best = (along, lateral, (px, py))

        travelled += seg_len

    if best is None:
        return 0.0, 0.0, path[0]

    return best


def point_at(path, distance):
    """Punto del camino a `distance` del inicio, saturado en los extremos."""
    if distance <= 0.0:
        return path[0]

    travelled = 0.0

    for i in range(len(path) - 1):

        ax, ay = path[i]
        bx, by = path[i + 1]

        seg_len = math.hypot(bx - ax, by - ay)

        if seg_len < 1e-12:
            continue

        if travelled + seg_len >= distance:
            t = (distance - travelled) / seg_len
            return (ax + t * (bx - ax), ay + t * (by - ay))

        travelled += seg_len

    return path[-1]


def carrot(path, rx, ry, lookahead):
    """Punto de anticipacion y distancia que queda hasta el final.

    El carrot es el corazon de Pure Pursuit: perseguir un punto que se
    desliza por delante en vez del destino final. Cerca del objetivo la
    anticipacion se agota contra el extremo del camino y el carrot pasa
    a ser el destino, con lo que la llegada es limpia y sin latigazo.
    """
    along, lateral, _ = project_on_path(path, rx, ry)

    total = path_length(path)
    remaining = max(0.0, total - along)

    return point_at(path, along + lookahead), remaining, lateral


# ---------------------------------------------------------------------
# Perfil de velocidad
# ---------------------------------------------------------------------

def plane_returns(values, expected, band):
    """Ecos que caen en el plano esperado, descartando lo de detras.

    El sector frontal del LiDAR no mide "el marcador": mide lo que haya
    delante. Si el ArUco esta sobre una caja separada de la pared, y el
    sector es mas ancho que la caja, la MAYORIA de los ecos son de la
    pared. Tomar la mediana entonces devuelve la pared con toda
    confianza, y el robot cree que le falta camino cuando ya ha
    llegado. Medido: sector de +-6 grados a 0.5 m abarca +-5.3 cm, y el
    marcador mide 8 cm.

    La banda es de un solo lado a proposito. Un eco MAS CERCA que lo
    esperado es un obstaculo real y tiene que seguir contando, que para
    eso existe la parada de seguridad. Uno mas lejos es el fondo, y es
    justo lo que hay que tirar.

    `expected` es la distancia geometrica del LiDAR al plano. Con
    expected None no se filtra nada: sin una expectativa no hay forma
    honesta de decidir que sobra.
    """
    if expected is None or band <= 0.0:
        return list(values)
    limite = expected + band
    return [v for v in values if v <= limite]


def robust_nearest(values, cluster_band):
    """Distancia estable del grupo de ecos mas cercano.

    El minimo de un sector cambia de haz cuando el robot se desplaza de
    lado y puede enganchar alternativamente el borde del marcador. Se
    conserva como proteccion el minimo, pero para medir la llegada se usa
    la mediana de los ecos que pertenecen a ese mismo grupo cercano.
    """
    if not values:
        return None

    nearest = min(values)
    cluster = [v for v in values if v <= nearest + cluster_band]

    return float(statistics.median(cluster))


def stopping_distance(speed, latency, period):
    """Cuanto sigue recorriendo tras mandarle parar.

    El mando de cero tarda `latency` en llegar (tuberia + red) y ademas
    el ciclo ya en curso dura `period`. A velocidad constante eso es
    speed * (latency + period).

    Medido en esta base: el suelo de giro es 0.37 rad/s y el retardo
    ~200 ms, o sea 0.093 rad = 5.3 grados que el robot gira DESPUES de
    decidir pararse. Contra una tolerancia de 0.08 rad (4.6 grados) es
    imposible asentarse: sale por el otro lado y corrige al reves. Ese
    fue el baile izquierda-derecha, y no se arregla con ganancias.

    REGLA: ninguna tolerancia puede ser menor que su distancia de
    parada. Si lo es, el objetivo es inalcanzable por construccion.
    """
    return abs(speed) * (latency + period)


def ray_polygon_exit_distance(origin, direction, polygon):
    """Distancia hasta que un rayo sale de un footprint convexo.

    ``origin`` y ``polygon`` estan en el mismo frame. ``direction`` debe
    ser unitario. El rayo empieza dentro del footprint (el sensor esta
    montado dentro del chasis) y se usa para convertir un eco LiDAR en el
    despeje desde el borde real del robot, sin asumir que el sensor mira
    exactamente hacia +X.
    """
    ox, oy = origin
    dx, dy = direction
    if not polygon or len(polygon) < 3:
        return None

    best = None
    for index, (x2, y2) in enumerate(polygon):
        x1, y1 = polygon[index - 1]
        ex = x2 - x1
        ey = y2 - y1
        cross = dx * ey - dy * ex
        if abs(cross) < 1e-9:
            continue
        rx = x1 - ox
        ry = y1 - oy
        distance = (rx * ey - ry * ex) / cross
        edge_fraction = (rx * dy - ry * dx) / cross
        if distance >= -1e-9 and -1e-9 <= edge_fraction <= 1.0 + 1e-9:
            if best is None or distance < best:
                best = max(0.0, distance)
    return best


def tolerance_is_reachable(tolerance, floor_speed, latency, period):
    """La tolerancia, es alcanzable con este suelo y este retardo?"""
    return tolerance >= stopping_distance(floor_speed, latency, period)


def brake_target(control_distance, stop_distance, speed, latency, period,
                 sensor_period=0.0, v_max=0.0, a_max=0.0):
    """Distancia efectiva para la rampa de frenado, con la inercia del
    lazo descontada de forma que NO oscile.

    La rampa v=sqrt(2*a*d) frena como si el mando surtiera efecto al
    instante. No: entre latencia, ciclo y el refresco del LiDAR el robot
    avanza ~v*T mas despues de decidir (T = latency + period +
    sensor_period). Medido en bag: ~0.35 s de coast desde 0.19 m/s.

    En vez de restar `speed_anterior * T` -- que realimentaba y hacia
    que el 52% de los mandos salieran CERO exacto, partiendo por dos la
    velocidad efectiva -- se resuelve la ecuacion de punto fijo:

        v = sqrt(2*a*(d - v*T))    ->    v = -a*T + sqrt(a^2 T^2 + 2 a d)

    que es la velocidad que, mandada AHORA, deja al robot frenando por
    la rampa correcta cuando el mando llega. Es funcion monotona y suave
    de d: baja segun el robot se acerca, sin escalones, sin ceros
    intercalados. Tiende a 0 en d=0 por construccion.

    Se devuelve la distancia equivalente v^2/(2a) para que
    `profile_speed` reproduzca esa v. Sin a_max se cae al modelo simple
    (compatibilidad).
    """
    d = control_distance - stop_distance
    if d <= 0.0:
        return d

    t_total = latency + period + sensor_period

    if a_max > 0.0:
        v_comp = -a_max * t_total + math.sqrt(
            (a_max * t_total) ** 2 + 2.0 * a_max * d
        )
        v_comp = max(0.0, v_comp)
        if v_max > 0.0:
            v_comp = min(v_comp, v_max)
        return (v_comp * v_comp) / (2.0 * a_max)

    return d - abs(speed) * t_total

def profile_speed(
    remaining, v_max, a_max,
    v_min=0.0,
    tolerance=0.0,
    stop_margin=0.0,
):
    """Rampa de frenado: v = sqrt(2*a*d), saturada a v_max.

    Es el perfil trapezoidal de toda la vida. Sustituye al proporcional
    puro del control anterior, cuyo problema era estructural: v = kp*e
    se hace infinitesimal cerca del objetivo y cae bajo la zona muerta
    de los motores, asi que el robot se paraba ANTES de llegar y el
    estado no cerraba nunca.

    `v_min` no es una zona muerta: es el SUELO de la base. Medido en
    este robot, por debajo de el la base NO MODULA -- pedirle 0.02 o
    pedirle 0.07 produce lo mismo, 0.07. O sea que el conjunto de
    velocidades alcanzables no es un intervalo continuo sino

        {0}  union  [v_min, v_max]

    y por debajo de v_min la rampa de frenado NO EXISTE. Ahi solo se
    puede elegir entre el suelo y parar, asi que se elige por distancia
    de parada: si lo que queda cabe en lo que el robot recorreria antes
    de detenerse, se manda cero; si no, el suelo.

    Sin esto el planificador cree ir a la mitad de lo que va y frena
    tarde. Con los valores de hoy la rampa solo baja del suelo por
    debajo de 1 cm, muy dentro de la tolerancia de 3 cm, asi que no se
    llega a notar. Pero v_min, a_max y la tolerancia se ajustan desde
    el launch, y el dia que se toquen esto deja de ser inofensivo --
    la misma leccion que la elipse de la zona muerta.
    """
    # `remaining` puede ser la distancia equivalente devuelta por
    # `brake_target`, no la distancia geometrica al objetivo. En ese caso
    # usar `tolerance` aqui manda cero antes de tiempo: el siguiente eco
    # puede volver a poner el valor por encima de la tolerancia y el mando
    # alterna entre cero y el minimo de las ruedas. `stop_margin` ya
    # representa la distancia que el robot necesita para detenerse; solo
    # usar la tolerancia cuando no se proporciona ese margen (compatibilidad
    # con llamadas directas al planificador).
    stop_threshold = (
        stop_margin if stop_margin > 0.0 else tolerance
    )

    if remaining <= stop_threshold:
        return 0.0

    v = min(v_max, math.sqrt(max(0.0, 2.0 * a_max * remaining)))

    if v_min > 0.0 and v < v_min:

        if remaining <= stop_margin:
            return 0.0

        return min(v_min, v_max)

    return min(v, v_max)


def apply_deadband(value, minimum, tolerance_reached=False):
    """Saca un mando de la zona muerta de los motores.

    Por debajo de `minimum` las ruedas no giran: el mando se publica y
    no pasa nada. Se eleva al minimo conservando el signo, salvo que ya
    estemos dentro de tolerancia, donde lo correcto es cero.
    """
    if tolerance_reached or value == 0.0:
        return 0.0

    if abs(value) < minimum:
        return math.copysign(minimum, value)

    return value


def deadband_floor(dx, dy, min_linear, min_lateral):
    """Radio de la elipse de zona muerta en la direccion (dx, dy) unitaria.

    La elipse es (vx/a)^2 + (vy/b)^2 = 1 con a=min_linear y
    b=min_lateral. Sustituyendo el punto (r*dx, r*dy) y despejando:

        r = 1 / sqrt((dx/a)^2 + (dy/b)^2)

    NO es hypot(a*dx, b*dy). Esa expresion parametriza la elipse por la
    direccion de la PREIMAGEN en el circulo unidad, no por la del rayo
    que se pide, y por Cauchy-Schwarz siempre sobrepasa. Con a y b
    parecidos la diferencia es del 1% y no se nota; en cuanto divergen
    se dispara, y son ajustables desde el launch:

        a=0.03 b=0.035   ->  hasta x1.01   (los valores de hoy)
        a=0.03 b=0.20    ->  hasta x3.41 a 45 grados

    Un suelo 3.4 veces mas alto del pedido es exactamente el tiron que
    bajar los minimos pretende evitar.

    Con cualquiera de los dos semiejes a cero la elipse degenera y no
    hay suelo que aplicar: se devuelve 0 y el mando pasa tal cual.
    """
    if min_linear <= 0.0 or min_lateral <= 0.0:
        return 0.0

    return 1.0 / math.sqrt(
        (dx / min_linear) ** 2 +
        (dy / min_lateral) ** 2
    )


# ---------------------------------------------------------------------
# Ley de control holonoma
# ---------------------------------------------------------------------

def desired_heading(rx, ry, mx, my, nx, ny, remaining, blend_distance):
    """Yaw objetivo: lejos mira al marcador, cerca se pone perpendicular.

    Mirar al marcador mientras se navega es lo que lo mantiene DENTRO
    del campo de vision -- perderlo era el modo de fallo dominante del
    control anterior. Y al llegar al punto de encare las dos referencias
    coinciden solas, porque encarar el marcador desde la normal ES
    estar perpendicular. La mezcla evita el salto entre ambas.
    """
    normal_yaw = math.atan2(-ny, -nx)

    dx, dy = mx - rx, my - ry

    if math.hypot(dx, dy) < 1e-6:
        return normal_yaw

    bearing_yaw = math.atan2(dy, dx)

    if blend_distance <= 0.0:
        return normal_yaw

    weight = clamp(remaining / blend_distance, 0.0, 1.0)

    return angle_lerp(normal_yaw, bearing_yaw, weight)


def holonomic_command(
    rx, ry, ryaw,
    carrot_xy,
    target_yaw,
    remaining,
    limits,
    yaw_settled=False,
):
    """Velocidades en el marco del ROBOT hacia el carrot.

    vx y vy se calculan SIEMPRE juntos: corregir un eje de traslacion
    cada vez era lo que hacia que el control anterior se persiguiera la
    cola (corregir el lateral cambia el rumbo, corregir el rumbo cambia
    el lateral).

    El giro, en cambio, NO puede acompañarlos: la placa no acepta los
    tres ejes a la vez y se queda quieta. Ver el bloque de
    holonomic_command donde se anula la traslacion, con las medidas.

    `limits` es un dict con: max_linear, max_lateral, max_angular,
    min_linear, min_lateral, min_angular, accel, distance_tolerance,
    yaw_tolerance.
    """
    cx, cy = carrot_xy

    dx, dy = cx - rx, cy - ry

    # A cuerpo: girar el error del mundo por -yaw del robot.
    cos_y, sin_y = math.cos(-ryaw), math.sin(-ryaw)

    ex = dx * cos_y - dy * sin_y
    ey = dx * sin_y + dy * cos_y

    norm = math.hypot(ex, ey)

    reached = remaining <= limits['distance_tolerance']

    speed = profile_speed(
        remaining,
        limits['max_linear'],
        limits['accel'],
        v_min=limits.get('min_linear', 0.0),
        tolerance=limits['distance_tolerance'],
        stop_margin=limits.get('stop_margin', 0.0),
    )

    if norm > 1e-9 and speed > 0.0:
        ux, uy = ex / norm, ey / norm
    else:
        ux, uy = 0.0, 0.0

    vx = speed * ux
    vy = speed * uy

    # El tope lateral es mas bajo que el frontal (mas friccion en
    # mecanum al desplazarse). Se escala el VECTOR completo, no cada
    # componente por separado: recortar solo vy torceria la direccion
    # del movimiento y el robot dejaria de seguir el camino.
    scale = 1.0

    if abs(vx) > limits['max_linear']:
        scale = min(scale, limits['max_linear'] / abs(vx))

    if abs(vy) > limits['max_lateral']:
        scale = min(scale, limits['max_lateral'] / abs(vy))

    vx *= scale
    vy *= scale

    # Zona muerta SOBRE EL VECTOR, no eje por eje.
    #
    # Aplicarla por separado a vx y a vy destroza la DIRECCION del
    # movimiento: si el objetivo esta muy a un lado, vy es grande y vx
    # minusculo, pero el minimo de avance eleva ese vx a min_linear y
    # el robot sale en diagonal en vez de de lado. Medido: hasta 26
    # grados de desvio, y el robot abandonando el camino.
    #
    # La zona muerta es una ELIPSE en el plano (vx, vy), con semiejes
    # min_linear y min_lateral, no un rectangulo. Se escala el vector
    # entero hasta el borde de esa elipse en la direccion pedida, con
    # lo que el modulo sube y la direccion se conserva exacta.
    if reached:
        vx = vy = 0.0

    else:

        speed_norm = math.hypot(vx, vy)

        if speed_norm > 1e-9:

            dx, dy = vx / speed_norm, vy / speed_norm

            floor_speed = deadband_floor(
                dx, dy,
                limits['min_linear'],
                limits['min_lateral'],
            )

            if speed_norm < floor_speed:
                vx = floor_speed * dx
                vy = floor_speed * dy

        else:
            vx = vy = 0.0

    yaw_error = normalize_angle(target_yaw - ryaw)

    # Histeresis (disparador Schmitt) en el giro.
    #
    # Sin ella el giro castañea: la zona muerta obliga a mandar
    # min_angular en cuanto se sale de tolerancia, ese minimo se pasa
    # de largo, y al ciclo siguiente hay que corregir al otro lado. El
    # robot gira a izquierda y derecha sin asentarse nunca.
    #
    # Con dos umbrales: se ENTRA en asentado con la tolerancia fina, y
    # solo se SALE si el error supera un umbral bastante mayor. Entre
    # los dos no se manda nada, que es justo lo que hay que hacer
    # cuando el error es menor que el escalon minimo que sabes dar.
    release = (
        limits['yaw_tolerance'] *
        limits.get('yaw_hysteresis', 2.5)
    )

    if yaw_settled:
        if abs(yaw_error) > release:
            yaw_settled = False

    elif abs(yaw_error) <= limits['yaw_tolerance']:
        yaw_settled = True

    if yaw_settled:
        wz = 0.0

    else:
        wz = clamp(
            limits['kp_angular'] * yaw_error,
            -limits['max_angular'],
            limits['max_angular'],
        )

        wz = apply_deadband(wz, limits['min_angular'])

        # LA PLACA NO ACEPTA LOS TRES EJES A LA VEZ.
        #
        # Medido en el robot, ventanas de 2 s sobre /odom:
        #
        #     solo giro                    0.773 rad
        #     solo avance                  0.212 m
        #     solo lateral                 0.180 m
        #     avance + giro        0.302 m / 0.273 rad
        #     lateral + giro       0.217 m / 0.277 rad
        #     avance + lateral     0.271 m
        #     LOS TRES             0.000 m / 0.000 rad   <-- ni un mm
        #     los tres, a la mitad         0.000
        #     los tres, a un cuarto        0.000
        #
        # Uno o dos ejes siempre se mueve; tres, cero absoluto a
        # CUALQUIER magnitud, asi que no es saturacion. Verificado que
        # el mando llega entero a /cmd_vel, o sea que no es twist_mux:
        # es la placa. En el frame de writeSpeed
        # (fe fe 01 0b [x] [y] [rot] [check], cada eje int(v*100)+128)
        # todos los casos que se mueven tienen exactamente UN byte de
        # eje a 0x80; los de tres ejes no tienen ninguno. Firmware
        # cerrado: hay que convivir con ello.
        #
        # Esto es lo que hacia que el robot "no avanzara", y tambien lo
        # que hacia que SOLO se aproximara de frente: de frente el error
        # de rumbo es ~0, wz sale 0 y quedan dos ejes, que si funcionan.
        # En oblicuo los tres son no nulos y la base se congela.
        #
        # Se sacrifica la SIMULTANEIDAD del giro con la traslacion, que
        # la placa no puede dar de todas formas. NO se separa vx de vy:
        # esos dos siguen yendo SIEMPRE juntos, que es lo que conserva
        # la direccion del movimiento (ver la elipse de zona muerta mas
        # arriba). Separarlos era el "perseguirse la cola" del control
        # viejo.
        #
        # La puerta es la histeresis que ya hay, no un umbral nuevo:
        # fuera de banda el ciclo es giro puro, dentro es traslacion
        # plena. Cerca del objetivo vx,vy tienden a 0 y el rumbo domina,
        # asi que salen ciclos de giro puro y la llegada se asienta
        # perpendicular.
        vx = 0.0
        vy = 0.0

    return vx, vy, wz, yaw_error, reached, yaw_settled


# ---------------------------------------------------------------------
# Compensacion de latencia
# ---------------------------------------------------------------------

def predict_pose(x, y, yaw, vx, vy, wz, dt):
    """Avanza la pose por integracion con las velocidades mandadas.

    Sirve para dos cosas:

      1. Anclar una deteccion que llega con retardo a la pose que el
         robot TENIA cuando se tomo la imagen, no a la de ahora
         (dt negativo).
      2. Adelantar la pose actual al instante en que el mando hara
         efecto (dt positivo).

    Integracion de primer orden: a 20 Hz y a 0.08 m/s el error de
    truncamiento es de micras, muy por debajo del ruido de la
    odometria. No merece la pena algo mas fino.
    """
    nyaw = normalize_angle(yaw + wz * dt)

    mid = normalize_angle(yaw + 0.5 * wz * dt)

    cos_y, sin_y = math.cos(mid), math.sin(mid)

    nx = x + (vx * cos_y - vy * sin_y) * dt
    ny = y + (vx * sin_y + vy * cos_y) * dt

    return nx, ny, nyaw


def compose(base_x, base_y, base_yaw, local_x, local_y):
    """Lleva un punto del marco del robot a odom."""
    cos_y, sin_y = math.cos(base_yaw), math.sin(base_yaw)

    return (
        base_x + local_x * cos_y - local_y * sin_y,
        base_y + local_x * sin_y + local_y * cos_y,
    )
