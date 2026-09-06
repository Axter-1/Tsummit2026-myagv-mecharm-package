#!/usr/bin/env python3

import math
import time
import threading

import numpy as np

import rclpy

from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener

from home_service_behaviors import approach_planner as planner

from home_service_interfaces.msg import ArucoDetectionArray
from home_service_interfaces.action import ArucoApproach


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (
        q.w * q.z +
        q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y +
        q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


class ArucoLidarApproachServer(Node):

    def __init__(self):

        super().__init__(
            'aruco_lidar_approach_server'
        )

        self.callback_group = (
            ReentrantCallbackGroup()
        )

        # =========================================================
        # Topics
        # =========================================================

        self.declare_parameter(
            'detections_topic',
            '/aruco/detections'
        )

        self.declare_parameter(
            'scan_topic',
            '/scan'
        )

        self.declare_parameter(
            'odom_topic',
            '/odom'
        )

        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel_aruco'
        )

        self.declare_parameter(
            'action_name',
            '/aruco_lidar_approach'
        )

        self.declare_parameter(
            'odom_frame',
            'odom'
        )

        # =========================================================
        # Search
        # =========================================================

        self.declare_parameter(
            'search_angular_speed',
            0.45
        )

        # Busqueda PASO-Y-MIRA. Girando en continuo el marcador no se
        # llegaba a detectar nunca: entre el desenfoque de movimiento de
        # la CSI y la latencia de la tuberia (JPEG -> WiFi -> portatil),
        # el ArUco cruzaba el campo de vision sin dejar un solo fotograma
        # nitido y quieto. Ahora gira un paso corto y se PARA a mirar.
        self.declare_parameter(
            'search_step_sec',
            0.45
        )

        self.declare_parameter(
            'search_dwell_sec',
            0.70
        )

        # =========================================================
        # Lock target normal
        # =========================================================

        # Alinearse perpendicular al plano del marcador es bonito sobre
        # el papel y fragil en este robot: la camara va a 7 cm y mira los
        # marcadores desde muy abajo, asi que la normal estimada sale
        # casi VERTICAL y su proyeccion horizontal -- la unica parte que
        # da rumbo -- es minuscula. Un error de pocos grados en la pose
        # se convierte en decenas de grados de rumbo. Sumado a la
        # ambiguedad planar del ArUco, el resultado medido en el robot
        # fue un giro sistematico de ~45 grados hacia un rumbo inventado.
        #
        # Se INTENTA por defecto, porque alinearse perpendicular es el
        # comportamiento que se quiere. Lo que protege del giro de 45
        # grados no es desactivarlo, son los dos filtros de abajo:
        # normal_min_horizontal descarta las normales casi verticales
        # (cuyo rumbo es ruido amplificado) y lock_min_coherence descarta
        # los lotes de muestras que no se ponen de acuerdo entre si. Si
        # los filtros rechazan el lock, se cae a centrado + avance en vez
        # de girar hacia un rumbo inventado.
        #
        # Usa 'analyze' de tsummit_offboard.sh para ver, con un marcador
        # delante, si esta geometria da normales utilizables.
        self.declare_parameter(
            'use_marker_normal',
            True
        )

        # Fraccion horizontal minima de la normal para creersela.
        # 0.5 = la normal debe estar a menos de 60 grados de la
        # horizontal. Por debajo, su rumbo es ruido amplificado.
        self.declare_parameter(
            'normal_min_horizontal',
            0.5
        )

        self.declare_parameter(
            'lock_duration',
            1.20
        )

        self.declare_parameter(
            'lock_min_samples',
            15
        )

        # Dispersion maxima admisible entre las muestras del normal.
        #
        # La pose de orientacion de un ArUco pequeno visto casi de
        # frente sufre AMBIGUEDAD PLANAR: la solucion salta entre dos
        # ramas simetricas. Promediar dos ramas separadas 90 grados da
        # un rumbo a 45 grados de ambas, que es justo el error que se
        # observo en el robot. La longitud del vector medio mide eso:
        # 1.0 = muestras identicas, ~0.7 = reparto entre dos ramas a 90
        # grados. Por debajo del umbral NO se confia en el normal y se
        # cae al modo de centrado directo.
        self.declare_parameter(
            'lock_min_coherence',
            0.93
        )

        # Cuanto insistir en lograr un lock coherente antes de rendirse
        # y aproximarse solo por el centrado de la camara.
        self.declare_parameter(
            'lock_max_attempts',
            3
        )

        # =========================================================
        # Heading
        # =========================================================

        self.declare_parameter(
            'kp_heading',
            1.5
        )

        self.declare_parameter(
            'max_heading_speed',
            0.30
        )

        # Velocidad angular minima EFECTIVA. Por debajo de esto los
        # motores del myAGV no vencen la friccion estatica: el mando se
        # publica, el robot no se mueve, el error no baja y el control
        # proporcional entra en ciclo limite (tiron, pasada, tiron al
        # otro lado). Cualquier wz no nulo se eleva a este valor.
        self.declare_parameter(
            'min_heading_speed',
            0.12
        )

        # 0.02 rad = 1.15 grados era inalcanzable: en ese borde wz vale
        # 0.03 rad/s, muy por debajo de min_heading_speed. 0.12 rad = 7
        # grados sobra para una aproximacion perpendicular.
        self.declare_parameter(
            'heading_tolerance',
            0.12
        )

        # 0.05 rad = 2.9 grados: el desplazamiento lateral en mecanum
        # deriva mas que eso, asi que ALIGNING_LATERAL rebotaba a
        # ALIGN_HEADING_TO_ARUCO en cuanto empezaba a moverse.
        self.declare_parameter(
            'heading_realign_threshold',
            0.25
        )

        # =========================================================
        # Lateral alignment
        # =========================================================

        self.declare_parameter(
            'kp_lateral',
            0.08
        )

        self.declare_parameter(
            'max_lateral_speed',
            0.06
        )

        # Zona muerta lateral. Desplazarse de lado en mecanum exige MAS
        # par que girar: las cuatro ruedas empujan en diagonal y la
        # friccion transversal de los rodillos se suma. Un vy de 0.01 m/s
        # se publica y no mueve nada, y el centrado entraba en el mismo
        # ciclo limite de tirones que tenia el rumbo.
        self.declare_parameter(
            'min_lateral_speed',
            0.035
        )

        self.declare_parameter(
            'lateral_tolerance',
            0.04
        )

        self.declare_parameter(
            'lateral_realign_threshold',
            0.15
        )

        # Ganancia del rodeo hasta el eje normal. El error va en
        # radianes (no en pixeles como kp_lateral), asi que necesita su
        # propia ganancia: 0.15 da ~0.05 m/s con 20 grados de desvio.
        self.declare_parameter(
            'kp_axis',
            0.15
        )

        self.declare_parameter(
            'axis_tolerance_deg',
            8.0
        )

        # =========================================================
        # Forward movement
        # =========================================================

        self.declare_parameter(
            'kp_linear',
            0.5
        )

        self.declare_parameter(
            'max_linear_speed',
            0.08
        )

        # Zona muerta hacia delante, hermana de min_lateral_speed.
        # Sin esto el robot se para a unos centimetros del objetivo sin
        # llegar a cumplir la condicion de parada.
        self.declare_parameter(
            'min_linear_speed',
            0.05
        )

        # 0.01 m no es alcanzable con la latencia de la tuberia
        # (JPEG -> WiFi -> portatil -> cmd_vel -> WiFi -> motores): a
        # 0.05 m/s el robot recorre ~1.5 cm solo en lo que llega la
        # orden de parar. Con 0.03 se para dentro de la ventana.
        self.declare_parameter(
            'distance_tolerance',
            0.03
        )

        # =========================================================
        # LiDAR geometry
        # =========================================================

        # Medido en el robot: camera_link a x=0.16, laser_frame a
        # x=0.065 respecto de base_link. La diferencia es 0.095, no
        # 0.16: ese 0.16 era la x de la camara, no la separacion.
        self.declare_parameter(
            'camera_x_minus_lidar_x',
            0.095
        )

        self.declare_parameter(
            'lidar_sector_half_angle_deg',
            6.0
        )

        # Angulo del FRENTE del robot medido EN EL FRAME DEL LASER.
        #
        # No es 0. El YDLidar va montado girado 180 grados
        # (base_link -> laser_frame tiene yaw = pi), asi que el 0 del
        # scan apunta a la TRASERA del robot, justo contra el chasis:
        # medido en el robot, el sector 0 +-4 no devuelve NI UN punto
        # valido ni siquiera en /scan crudo, mientras que 180 +-4 da 11
        # puntos a 0.91 m. El servidor promediaba el sector 0 y por eso
        # se quedaba en "Waiting for lidar" para siempre.
        #
        # 999.0 = deducirlo de la TF base_link -> laser_frame (lo
        # correcto: sobrevive a que alguien remonte el sensor).
        # Cualquier otro valor lo fija a mano.
        self.declare_parameter(
            'lidar_front_angle_deg',
            999.0
        )

        # =========================================================
        # Normal por LIDAR  (fuente preferente)
        # =========================================================
        #
        # La normal sacada de la POSE del ArUco es el punto debil de
        # todo esto: ambiguedad planar, y con la camara a 7 cm sale casi
        # vertical, asi que su proyeccion horizontal -- la unica que da
        # rumbo -- es ruido amplificado.
        #
        # El lidar no tiene ninguno de esos dos problemas. El marcador
        # esta pegado a una superficie plana, el lidar VE esa superficie,
        # y una recta ajustada a esos puntos da la orientacion del plano
        # directamente en horizontal y en metros de verdad. El ArUco se
        # usa para lo que es bueno: DECIR CUAL es el objetivo y en que
        # direccion esta. El lidar, para la geometria.
        self.declare_parameter(
            'use_lidar_normal',
            True
        )

        # Sector alrededor del marcador donde buscar la superficie.
        # 30 grados abarcaba pared del marcador Y pared contigua: el
        # ajuste por SVD encajaba una recta perfecta (coherencia 1.00,
        # residuo de milimetros) sobre la superficie EQUIVOCADA, y salia
        # una normal a ~99 grados de la linea de vision. La recta era
        # buena, la pared no. 15 grados deja fuera la pared vecina.
        self.declare_parameter(
            'lidar_normal_half_angle_deg',
            15.0
        )

        # Banda de profundidad alrededor del punto mas cercano del
        # sector: descarta la pared del fondo y los objetos sueltos que
        # caen en el mismo angulo.
        self.declare_parameter(
            'lidar_normal_depth_band',
            0.30
        )

        self.declare_parameter(
            'lidar_normal_min_points',
            8
        )

        # Residuo maximo del ajuste de recta. Si la nube no es una
        # recta (esquina, objeto curvo, dos superficies), no hay un
        # plano al que ponerse perpendicular.
        self.declare_parameter(
            'lidar_normal_max_residual',
            0.02
        )

        # Oblicuidad maxima entre la normal fijada y la direccion al
        # marcador. Si el robot VE el ArUco, no puede estar mirando su
        # superficie de canto: por encima de ~60 grados el marcador
        # dejaria de detectarse. Una normal a 80 grados de la linea de
        # vision significa que el ajuste cogio OTRA superficie (una
        # pared lateral, el borde de un mueble), no la del marcador.
        #
        # Sin esta comprobacion el robot fijaba rumbos de -68 a -90
        # grados con coherencia 1.00 -- el ajuste era perfecto, solo que
        # de la superficie equivocada -- giraba 90 grados, el marcador
        # se le salia del encuadre y volvia a buscar. En bucle, 11 veces
        # seguidas.
        self.declare_parameter(
            'normal_max_obliquity_deg',
            60.0
        )

        # =========================================================
        # Freshness
        # =========================================================

        # 0.6 s: la deteccion en la Nano ronda 4-6 Hz (0.17-0.25 s) y
        # con picos de carga se salta algun frame. 0.35 abortaba con
        # TARGET_LOST en falso; 0.6 aguanta un par de frames perdidos
        # sin dejar de reaccionar a que el marcador desaparezca de
        # verdad.
        self.declare_parameter(
            'detection_timeout',
            0.6
        )

        # Cuanto aguantar sin ver el marcador durante el centrado antes
        # de volver a buscarlo. Al girar hacia la perpendicular se sale
        # del encuadre un momento y vuelve.
        self.declare_parameter(
            'lost_marker_timeout',
            3.0
        )

        self.declare_parameter(
            'scan_timeout',
            0.35
        )

        self.declare_parameter(
            'control_rate',
            20.0
        )


        # -------------------------------------------------------------
        # Aproximacion con punto de encare y carrot (approach_planner)
        # -------------------------------------------------------------

        # Distancia del punto de encare al marcador, sobre su normal.
        # Desde ahi la aproximacion final es una recta perpendicular.
        self.declare_parameter(
            'staging_standoff',
            0.45
        )

        # Anticipacion del carrot. Mas alto = mas suave y mas lento en
        # reaccionar; mas bajo = mas ceñido al camino y mas nervioso.
        self.declare_parameter(
            'lookahead_distance',
            0.25
        )

        # Semiancho del pasillo de aproximacion. Dentro de el se va
        # recto al objetivo en vez de rodear por el punto de encare.
        self.declare_parameter(
            'corridor_radius',
            0.12
        )

        # Frenada del perfil trapezoidal: v = sqrt(2*a*d).
        self.declare_parameter(
            'linear_accel',
            0.25
        )

        # Filtro del estimador. Suelo del alfa adaptativo; al principio
        # manda la media corriente 1/n porque el marcador no se mueve.
        self.declare_parameter(
            'estimate_alpha_position',
            0.20
        )

        self.declare_parameter(
            'estimate_alpha_normal',
            0.10
        )

        # Sin detecciones durante mas de esto, se avisa: se sigue
        # navegando a ciegas por odometria, que deriva.
        self.declare_parameter(
            'estimate_max_age',
            3.0
        )

        # Parada de seguridad por LiDAR frontal.
        self.declare_parameter(
            'min_front_clearance',
            0.12
        )

        # =========================================================
        # State
        # =========================================================

        self.lock = threading.Lock()

        self.latest_detections = {}

        self.latest_scan = None
        self.latest_scan_time_ns = None

        # Cache del frente deducido de la TF (ver get_lidar_front_angle).
        self._lidar_front_angle = None

        # Cache de base_link <- laser_frame: (x, y, yaw). Es estatica.
        self._laser_to_base = None

        # Motivo del ultimo fallo del lidar. WAITING_LIDAR era una caja
        # negra: no distinguia "el scan no llega" de "llega pero el
        # sector que miro esta vacio", que piden arreglos opuestos.
        self._lidar_fail = None

        self.latest_odom = None

        # =========================================================
        # TF
        # =========================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # =========================================================
        # Subscribers
        # =========================================================

        self.create_subscription(
            ArucoDetectionArray,
            self.get_parameter(
                'detections_topic'
            ).value,
            self.detections_callback,
            10,
            callback_group=self.callback_group
        )

        self.create_subscription(
            LaserScan,
            self.get_parameter(
                'scan_topic'
            ).value,
            self.scan_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )

        self.create_subscription(
            Odometry,
            self.get_parameter(
                'odom_topic'
            ).value,
            self.odom_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )

        # =========================================================
        # Velocity
        # =========================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter(
                'cmd_vel_topic'
            ).value,
            10
        )

        # =========================================================
        # Action
        # =========================================================

        self.action_server = ActionServer(
            self,
            ArucoApproach,
            self.get_parameter(
                'action_name'
            ).value,
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            'ArUco + LiDAR geometric '
            'approach server started'
        )

    # =============================================================
    # Helpers
    # =============================================================

    def pf(self, name):
        return float(
            self.get_parameter(name).value
        )

    def detections_callback(self, msg):

        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        with self.lock:

            for detection in msg.detections:

                self.latest_detections[
                    int(detection.id)
                ] = (
                    detection,
                    now_ns
                )

    def scan_callback(self, msg):

        with self.lock:

            self.latest_scan = msg

            self.latest_scan_time_ns = (
                self.get_clock()
                .now()
                .nanoseconds
            )

    def odom_callback(self, msg):

        with self.lock:
            self.latest_odom = msg

    # =============================================================
    # Robot pose
    # =============================================================

    def get_robot_pose(self):

        with self.lock:
            odom = self.latest_odom

        if odom is None:
            return None

        x = float(
            odom.pose.pose.position.x
        )

        y = float(
            odom.pose.pose.position.y
        )

        yaw = yaw_from_quaternion(
            odom.pose.pose.orientation
        )

        return x, y, yaw

    # =============================================================
    # Current detection
    # =============================================================

    def get_detection(self, target_id):

        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        with self.lock:

            data = self.latest_detections.get(
                target_id
            )

        if data is None:
            return None

        detection, stamp_ns = data

        age = (
            now_ns - stamp_ns
        ) / 1e9

        if age > self.pf(
            'detection_timeout'
        ):
            return None

        return detection

    # =============================================================
    # Marker normal in odom
    #
    # IMPORTANT:
    # We use marker ORIENTATION, not marker distance.
    #
    # The physical marker size can therefore be wrong without
    # affecting the distance controller.
    # =============================================================

    def get_marker_normal(
        self,
        target_id
    ):

        robot_pose = self.get_robot_pose()

        if robot_pose is None:
            return None

        robot_x, robot_y, _ = robot_pose

        odom_frame = self.get_parameter(
            'odom_frame'
        ).value

        marker_frame = (
            f'aruco_{target_id}'
        )

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    odom_frame,
                    marker_frame,
                    Time(),
                    timeout=Duration(
                        seconds=0.1
                    )
                )
            )

        except Exception:
            return None

        q = transform.transform.rotation

        # Third column of rotation matrix:
        # marker local +Z axis transformed into odom.
        #
        # This is the normal vector of the ArUco plane.

        nx = 2.0 * (
            q.x * q.z +
            q.w * q.y
        )

        ny = 2.0 * (
            q.y * q.z -
            q.w * q.x
        )

        nz = 1.0 - 2.0 * (
            q.x * q.x +
            q.y * q.y
        )

        norm = math.hypot(
            nx,
            ny
        )

        # Descartar normales casi verticales ANTES de normalizar en 2D.
        # Normalizar borra la prueba de que la direccion horizontal era
        # despreciable: una normal a 5 grados de la vertical produce un
        # vector unitario con toda la pinta de ser fiable y un rumbo que
        # es puro ruido.
        horizontal = norm / max(
            1e-9,
            math.sqrt(
                norm * norm +
                nz * nz
            )
        )

        if horizontal < self.pf(
            'normal_min_horizontal'
        ):
            return None

        if norm < 1e-6:
            return None

        nx /= norm
        ny /= norm

        # ---------------------------------------------------------
        # Choose the sign pointing ROBOT -> MARKER.
        #
        # Marker translation may have wrong magnitude if
        # marker_size is wrong, but its direction is sufficient
        # here simply to choose ±normal.
        # ---------------------------------------------------------

        marker_x = (
            transform.transform.translation.x
        )

        marker_y = (
            transform.transform.translation.y
        )

        to_marker_x = (
            marker_x - robot_x
        )

        to_marker_y = (
            marker_y - robot_y
        )

        dot = (
            nx * to_marker_x +
            ny * to_marker_y
        )

        if dot < 0.0:
            nx = -nx
            ny = -ny

        return nx, ny

    # =============================================================
    # Pose completa del marcador en odom
    # =============================================================

    def get_marker_pose_odom(self, target_id):
        """(mx, my, nx, ny) del marcador en odom, normal SALIENTE.

        Exige deteccion fresca por el mismo motivo que
        get_marker_bearing(): el buffer de TF guarda 10 s y, perdido el
        marcador, lookup_transform(..., Time()) sigue devolviendo la
        ultima transformada como si nada.

        Se consulta con Time() a proposito. tf2 evalua la cadena
        odom -> base_link -> camera -> aruco_N en el instante comun mas
        reciente, que lo limita el eslabon mas viejo: el del marcador.
        O sea que compone la odometria de CUANDO se tomo la imagen, no
        la de ahora. Para un marcador quieto eso da su posicion en odom
        ya libre del retardo de la vision, que es justo lo que se
        buscaba: el desfase deja de realimentarse en el lazo.
        """
        if self.get_detection(target_id) is None:
            return None

        normal = self.get_marker_normal(target_id)

        if normal is None:
            return None

        odom_frame = self.get_parameter('odom_frame').value

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    odom_frame,
                    f'aruco_{target_id}',
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
            )

        except Exception:
            return None

        mx = float(transform.transform.translation.x)
        my = float(transform.transform.translation.y)

        # get_marker_normal devuelve ROBOT -> superficie; el
        # planificador trabaja con la SALIENTE del marcador.
        nx, ny = planner.outward_normal(normal[0], normal[1])

        return mx, my, nx, ny

    # =============================================================
    # Front LiDAR
    # =============================================================

    def get_lidar_front_angle(
        self,
        scan
    ):

        override = self.pf(
            'lidar_front_angle_deg'
        )

        if abs(override) <= 180.0:
            return math.radians(override)

        if self._lidar_front_angle is not None:
            return self._lidar_front_angle

        # El frente del robot es +X de base_link. Expresado en el frame
        # del laser, ese eje queda rotado por -yaw(base_link->laser).
        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    scan.header.frame_id,
                    'base_link',
                    Time(),
                    timeout=Duration(
                        seconds=0.2
                    )
                )
            )

        except Exception:
            return 0.0

        yaw = yaw_from_quaternion(
            transform.transform.rotation
        )

        self._lidar_front_angle = (
            normalize_angle(yaw)
        )

        self.get_logger().info(
            'Frente del robot en el frame '
            f'{scan.header.frame_id}: '
            f'{math.degrees(self._lidar_front_angle):+.1f} '
            'grados (deducido de la TF)'
        )

        return self._lidar_front_angle

    def get_marker_bearing(
        self,
        target_id
    ):
        """Direccion al marcador vista desde base_link, en radianes.

        Solo la DIRECCION. La distancia del ArUco depende de que
        marker_size sea correcto; la direccion, no.

        EXIGE deteccion fresca. El buffer de TF guarda 10 s: si el
        marcador se pierde, `lookup_transform(..., Time())` sigue
        devolviendo la ultima transformada tan campante. Y como esa
        transformada cuelga de camera_optical_frame, el rumbo en
        base_link NO cambia aunque el robot gire. El resultado era un
        rumbo congelado, un error de giro constante, y el robot dando
        vueltas indefinidamente con state=ALIGN_HEADING_TO_ARUCO y
        center_error=0.0 -- girando como si buscara, pero sin buscar.

        La frescura se mide con get_detection(), que sella por hora de
        RECEPCION. La cabecera de la TF viene sellada por la Jetson y
        compararla con el reloj del portatil traeria el desfase de
        relojes por la puerta de atras.
        """

        if self.get_detection(target_id) is None:
            return None

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'base_link',
                    f'aruco_{target_id}',
                    Time(),
                    timeout=Duration(
                        seconds=0.1
                    )
                )
            )

        except Exception:
            return None

        return math.atan2(
            transform.transform.translation.y,
            transform.transform.translation.x
        )

    def normal_obliquity(
        self,
        heading,
        target_id
    ):
        """Angulo entre la normal fijada y la linea de vision al marcador.

        `heading` va en ODOM (es lo que consume heading_control). El
        rumbo al marcador se mide en el cuerpo y se pasa a odom con el
        yaw del robot. Devuelve None si falta alguno de los dos.
        """

        bearing = self.get_marker_bearing(
            target_id
        )

        pose = self.get_robot_pose()

        if bearing is None or pose is None:
            return None

        bearing_odom = normalize_angle(
            pose[2] + bearing
        )

        return abs(
            normalize_angle(
                heading - bearing_odom
            )
        )

    def get_lidar_surface_normal(
        self,
        target_id
    ):
        """Normal de la superficie donde esta el marcador, via lidar.

        Devuelve (nx, ny) unitario EN ODOM apuntando del ROBOT HACIA la
        superficie, o None si la nube no describe un plano fiable.

        OJO con el marco. El calculo se hace en base_link, porque el
        scan llega en el frame del laser y lo natural es pasarlo al
        cuerpo. Pero quien consume esto es heading_control, que compara
        contra el yaw del robot EN ODOM. Devolver el vector en base_link
        hacia que el error de rumbo fuera exactamente el yaw acumulado
        del robot: el automata giraba en direccion contraria al marcador,
        y tanto mas cuanto mas hubiera girado buscandolo. Medido: yaw
        -21.2 deg -> error +21.2 deg. Por eso el ultimo paso rota a odom.
        """

        bearing = self.get_marker_bearing(
            target_id
        )

        if bearing is None:
            return None

        with self.lock:
            scan = self.latest_scan
            stamp_ns = self.latest_scan_time_ns

        if scan is None or stamp_ns is None:
            return None

        age = (
            self.get_clock().now().nanoseconds -
            stamp_ns
        ) / 1e9

        if age > self.pf('scan_timeout'):
            return None

        laser_to_base = (
            self.get_laser_to_base()
        )

        if laser_to_base is None:
            return None

        offset_x, offset_y, laser_yaw = (
            laser_to_base
        )

        half = math.radians(
            self.pf(
                'lidar_normal_half_angle_deg'
            )
        )

        # Puntos del scan pasados a base_link, quedandonos con los que
        # caen en el sector angular alrededor del marcador.
        points = []

        for i, distance in enumerate(
            scan.ranges
        ):

            if not math.isfinite(distance):
                continue

            if (
                distance < scan.range_min or
                distance > scan.range_max
            ):
                continue

            angle = (
                scan.angle_min +
                i * scan.angle_increment
            )

            px = (
                offset_x +
                distance * math.cos(
                    angle + laser_yaw
                )
            )

            py = (
                offset_y +
                distance * math.sin(
                    angle + laser_yaw
                )
            )

            point_bearing = math.atan2(
                py,
                px
            )

            if abs(
                normalize_angle(
                    point_bearing - bearing
                )
            ) > half:
                continue

            points.append(
                (
                    px,
                    py,
                    math.hypot(px, py)
                )
            )

        if not points:
            return None

        # Quedarse con la superficie MAS CERCANA del sector: el
        # marcador esta en ella, no en la pared del fondo.
        nearest = min(
            p[2] for p in points
        )

        band = self.pf(
            'lidar_normal_depth_band'
        )

        selected = [
            (px, py)
            for px, py, r in points
            if r <= nearest + band
        ]

        min_points = int(
            self.get_parameter(
                'lidar_normal_min_points'
            ).value
        )

        if len(selected) < min_points:
            return None

        # Ajuste de recta por componentes principales (minimos
        # cuadrados totales: no privilegia ningun eje, a diferencia de
        # un ajuste y = mx + b, que revienta con superficies casi
        # paralelas al eje Y).
        data = np.array(
            selected,
            dtype=float
        )

        centroid = data.mean(axis=0)
        centred = data - centroid

        try:
            _, singular, vectors = (
                np.linalg.svd(
                    centred,
                    full_matrices=False
                )
            )
        except np.linalg.LinAlgError:
            return None

        direction = vectors[0]
        normal = vectors[1]

        # Residuo cuadratico medio respecto de la recta ajustada.
        residual = float(
            singular[1] /
            math.sqrt(len(selected))
        )

        if residual > self.pf(
            'lidar_normal_max_residual'
        ):

            self.get_logger().warn(
                'Superficie no plana junto al '
                f'marcador (residuo {residual:.3f} m '
                f'con {len(selected)} puntos)'
            )

            return None

        # La superficie debe tener extension: una nube corta y
        # apretada ajusta cualquier recta.
        extent = float(
            singular[0] /
            math.sqrt(len(selected))
        )

        if extent < 2.0 * max(
            residual,
            1e-3
        ):
            return None

        nx = float(normal[0])
        ny = float(normal[1])

        norm = math.hypot(nx, ny)

        if norm < 1e-6:
            return None

        nx /= norm
        ny /= norm

        # Signo: del ROBOT hacia la superficie. En base_link el robot
        # esta en el origen, asi que el centroide ES el vector hacia
        # la superficie.
        if (
            nx * centroid[0] +
            ny * centroid[1]
        ) < 0.0:
            nx = -nx
            ny = -ny

        del direction

        # A ODOM: rotar por el yaw del robot. Sin esto el rumbo
        # resultante es de cuerpo y heading_control lo trata como de
        # mundo (ver el docstring).
        pose = self.get_robot_pose()

        if pose is None:
            return None

        yaw = pose[2]

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        return (
            nx * cos_yaw - ny * sin_yaw,
            nx * sin_yaw + ny * cos_yaw,
        )

    def get_laser_to_base(self):

        if self._laser_to_base is not None:
            return self._laser_to_base

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'base_link',
                    'laser_frame',
                    Time(),
                    timeout=Duration(
                        seconds=0.2
                    )
                )
            )

        except Exception:
            return None

        self._laser_to_base = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw_from_quaternion(
                transform.transform.rotation
            ),
        )

        return self._laser_to_base

    def get_front_lidar_range(self):

        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        with self.lock:

            scan = self.latest_scan
            stamp_ns = (
                self.latest_scan_time_ns
            )

        if scan is None or stamp_ns is None:
            self._lidar_fail = 'SIN_SCAN'
            return None

        age = (
            now_ns - stamp_ns
        ) / 1e9

        if age > self.pf(
            'scan_timeout'
        ):
            self._lidar_fail = (
                f'SCAN_VIEJO({age:.2f}s)'
            )
            return None

        half_angle = math.radians(
            self.pf(
                'lidar_sector_half_angle_deg'
            )
        )

        front_angle = (
            self.get_lidar_front_angle(
                scan
            )
        )

        values = []

        for i, distance in enumerate(
            scan.ranges
        ):

            angle = (
                scan.angle_min +
                i * scan.angle_increment
            )

            # Diferencia angular CON ENVOLVENTE. El 'abs(angle)' de
            # antes se rompia en la frontera de +-pi, que es justo donde
            # cae el frente de este robot: +179 y -179 grados son vecinos
            # y la resta cruda los separa 358.
            if abs(
                normalize_angle(
                    angle - front_angle
                )
            ) > half_angle:
                continue

            if not math.isfinite(
                distance
            ):
                continue

            if distance < scan.range_min:
                continue

            if distance > scan.range_max:
                continue

            values.append(
                float(distance)
            )

        if not values:
            self._lidar_fail = (
                'SECTOR_VACIO(frente='
                f'{math.degrees(front_angle):+.0f}deg)'
            )
            return None

        self._lidar_fail = None

        return float(
            np.median(values)
        )

    # =============================================================
    # Commands
    # =============================================================

    def publish_cmd(
        self,
        vx=0.0,
        vy=0.0,
        wz=0.0
    ):

        msg = Twist()

        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)

        self.cmd_pub.publish(msg)

    def face_marker_control(
        self,
        target_id
    ):
        """Giro que mantiene el marcador centrado en la camara.

        El error ES el rumbo al marcador en el cuerpo: anularlo deja al
        robot encarado a el. Reutiliza heading_control pasandole el
        rumbo ya convertido a odom, para heredar sus limites y su zona
        muerta.
        """

        bearing = self.get_marker_bearing(
            target_id
        )

        pose = self.get_robot_pose()

        if bearing is None or pose is None:
            return None, None

        return self.heading_control(
            normalize_angle(
                pose[2] + bearing
            )
        )

    def axis_error(
        self,
        normal_heading,
        target_id
    ):
        """Angulo con signo entre la linea de vision y la normal fijada.

        Vale cero exactamente cuando el robot esta sobre el eje normal
        del marcador, que es la posicion desde la que se puede atacar de
        frente. Positivo = el robot esta desplazado a la derecha del eje.
        """

        if normal_heading is None:
            return None

        bearing = self.get_marker_bearing(
            target_id
        )

        pose = self.get_robot_pose()

        if bearing is None or pose is None:
            return None

        bearing_odom = normalize_angle(
            pose[2] + bearing
        )

        return normalize_angle(
            bearing_odom - normal_heading
        )

    def apply_lateral_deadband(
        self,
        vy
    ):

        minimum = self.pf(
            'min_lateral_speed'
        )

        if 0.0 < abs(vy) < minimum:
            return math.copysign(
                minimum,
                vy
            )

        return vy

    def apply_linear_deadband(
        self,
        vx
    ):
        """Lo mismo que apply_lateral_deadband, pero hacia delante.

        Faltaba, y es la mitad del problema: cerca del objetivo
        vx = kp_linear * error se hace minusculo (a 4 cm del goal,
        0.5 * 0.04 = 0.02 m/s) y cae bajo la zona muerta. El robot deja
        de avanzar ANTES de cumplir la condicion de parada: ni llega ni
        termina, se va por timeout. El cero exacto se respeta porque es
        una orden de parar, no un mando pequeno.
        """
        minimum = self.pf(
            'min_linear_speed'
        )

        if 0.0 < abs(vx) < minimum:
            return math.copysign(
                minimum,
                vx
            )

        return vx

    def stop_robot(self):

        for _ in range(3):

            self.publish_cmd()

            time.sleep(0.02)

    # =============================================================
    # Heading controller
    # =============================================================

    def heading_control(
        self,
        desired_heading
    ):

        # desired_heading None = no se pudo fijar un normal fiable.
        # Se renuncia a la perpendicularidad y la aproximacion se apoya
        # solo en el centrado de camara, que es estable: center_x es un
        # centroide en pixeles, no una pose 3D ambigua.
        if desired_heading is None:
            return 0.0, 0.0

        pose = self.get_robot_pose()

        if pose is None:
            return None, None

        _, _, current_yaw = pose

        error = normalize_angle(
            desired_heading -
            current_yaw
        )

        wz = (
            self.pf(
                'kp_heading'
            ) *
            error
        )

        wz = clamp(
            wz,
            -self.pf(
                'max_heading_speed'
            ),
            self.pf(
                'max_heading_speed'
            )
        )

        # Zona muerta de los motores: un wz de 0.03 rad/s se publica
        # pero no mueve el robot. Se eleva al minimo efectivo para que
        # el mando que se envia sea el mando que se ejecuta.
        min_wz = self.pf(
            'min_heading_speed'
        )

        if 0.0 < abs(wz) < min_wz:
            wz = math.copysign(
                min_wz,
                wz
            )

        return error, wz

    # =============================================================
    # Feedback
    # =============================================================

    def send_feedback(
        self,
        goal_handle,
        state,
        distance,
        center_error,
        elapsed
    ):

        feedback = (
            ArucoApproach.Feedback()
        )

        feedback.state = state
        feedback.distance = float(
            distance
        )

        feedback.center_error = float(
            center_error
        )

        feedback.elapsed_sec = float(
            elapsed
        )

        goal_handle.publish_feedback(
            feedback
        )

    # =============================================================
    # Action
    # =============================================================

    def execute_callback(
        self,
        goal_handle
    ):
        """Aproximacion con punto de encare y carrot, en el marco odom.

        Sustituye a la maquina de estados secuencial anterior
        (ALIGN_HEADING -> ALIGNING_LATERAL -> APPROACHING), que corregia
        un grado de libertad cada vez contra el error instantaneo. En
        una base mecanum eso se persigue la cola: corregir el
        desplazamiento lateral cambia el rumbo, corregir el rumbo
        cambia el lateral. De ahi el baile
        "Target found -> Heading aligned -> Marcador perdido" de los
        registros.

        Aqui solo hay dos estados de verdad:

          SEARCHING  no hay estimacion todavia: paso-y-mira.
          PURSUING   hay estimacion: se navega hacia ella con las tres
                     velocidades a la vez.

        El marcador se fija en ODOM y el robot navega con su propia
        odometria, que es local y sin retardo. Las detecciones pasan de
        ser el lazo de control a ser correcciones de un estimador, asi
        que perder el marcador un rato ya no rompe nada.
        """

        target_id = int(
            goal_handle.request.target_id
        )

        stop_distance = float(
            goal_handle.request.stop_distance
        )

        timeout_sec = float(
            goal_handle.request.timeout_sec
        )

        estimate = planner.TargetEstimate(
            alpha_position=self.pf(
                'estimate_alpha_position'
            ),
            alpha_normal=self.pf(
                'estimate_alpha_normal'
            ),
        )

        use_lidar = bool(
            self.get_parameter(
                'use_lidar_normal'
            ).value
        )

        state = 'SEARCHING'

        start_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        search_phase_start_ns = start_ns
        search_moving = True

        last_detection_ns = None
        stale_warned = False

        final_distance = -1.0
        center_error = 0.0

        period = 1.0 / max(
            1.0,
            self.pf('control_rate')
        )

        yaw_tolerance = math.radians(
            self.pf('heading_tolerance')
        )

        self.get_logger().info(
            f'Starting target ID {target_id} '
            f'(encare a {self.pf("staging_standoff"):.2f} m, '
            f'parada a {stop_distance:.2f} m)'
        )

        while rclpy.ok():

            now_ns = (
                self.get_clock()
                .now()
                .nanoseconds
            )

            elapsed = (
                now_ns - start_ns
            ) / 1e9

            # =====================================================
            # Cancelacion
            # =====================================================

            if goal_handle.is_cancel_requested:

                self.stop_robot()
                goal_handle.canceled()

                result = ArucoApproach.Result()

                result.success = False
                result.status = 'CANCELED'
                result.message = 'Goal canceled'
                result.final_distance = final_distance

                return result

            # =====================================================
            # Tiempo agotado
            # =====================================================

            if elapsed > timeout_sec:

                self.stop_robot()
                goal_handle.abort()

                result = ArucoApproach.Result()

                result.success = False
                result.status = 'TIMEOUT'
                result.message = (
                    f'Timeout tras {elapsed:.1f} s '
                    f'en estado {state}'
                )
                result.final_distance = final_distance

                return result

            # =====================================================
            # Estimador: la deteccion CORRIGE, no pilota
            # =====================================================

            detection = self.get_detection(target_id)

            if detection is not None:

                center_error = float(
                    detection.center_x_normalized
                )

                pose = self.get_marker_pose_odom(target_id)

                if pose is not None:

                    mx, my, nx, ny = pose

                    estimate.update(
                        mx, my, nx, ny,
                        stamp_ns=now_ns,
                    )

                    last_detection_ns = now_ns
                    stale_warned = False

                    # La normal del LiDAR es bastante mejor que la del
                    # ArUco, que en yaw es ruidosa y ambigua de perfil.
                    # Se mete como muestra de mas peso en vez de
                    # sustituir: si el ajuste engancha una pared vecina,
                    # el promedio lo diluye en lugar de creerselo.
                    if use_lidar:

                        lidar_normal = (
                            self.get_lidar_surface_normal(
                                target_id
                            )
                        )

                        if lidar_normal is not None:

                            lnx, lny = planner.outward_normal(
                                lidar_normal[0],
                                lidar_normal[1],
                            )

                            estimate.update(
                                mx, my, lnx, lny,
                                stamp_ns=now_ns,
                                alpha_scale=2.0,
                            )

            # =====================================================
            # SEARCHING: sin estimacion no hay a donde ir
            # =====================================================

            if not estimate.ready:

                state = 'SEARCHING'

                phase_elapsed = (
                    now_ns - search_phase_start_ns
                ) / 1e9

                # Paso-y-mira: girando en continuo no queda ni un
                # fotograma nitido y quieto del marcador.
                if search_moving:

                    if phase_elapsed >= self.pf('search_step_sec'):

                        self.stop_robot()
                        search_moving = False
                        search_phase_start_ns = now_ns

                    else:

                        self.publish_cmd(
                            wz=self.pf('search_angular_speed')
                        )

                else:

                    self.publish_cmd()

                    if phase_elapsed >= self.pf('search_dwell_sec'):

                        search_moving = True
                        search_phase_start_ns = now_ns

                self.send_feedback(
                    goal_handle, state,
                    final_distance, center_error, elapsed,
                )

                time.sleep(period)
                continue

            # =====================================================
            # PURSUING
            # =====================================================

            robot_pose = self.get_robot_pose()

            if robot_pose is None:

                # Sin odometria no se puede navegar en odom. Parar es
                # lo unico honesto: seguir seria integrar a ciegas.
                self.stop_robot()

                self.get_logger().warn(
                    'Sin odometria; no puedo navegar.',
                    throttle_duration_sec=2.0,
                )

                self.send_feedback(
                    goal_handle, state,
                    final_distance, center_error, elapsed,
                )

                time.sleep(period)
                continue

            state = 'PURSUING'

            rx, ry, ryaw = robot_pose
            mx, my, nx, ny = estimate.pose

            standoff = self.pf('staging_standoff')

            path = planner.build_path(
                rx, ry, mx, my, nx, ny,
                standoff,
                stop_distance,
                corridor_radius=self.pf('corridor_radius'),
            )

            carrot_xy, remaining, off_path = planner.carrot(
                path, rx, ry,
                self.pf('lookahead_distance'),
            )

            along, lateral = planner.corridor_coords(
                rx, ry, mx, my, nx, ny
            )

            target_yaw = planner.desired_heading(
                rx, ry, mx, my, nx, ny,
                remaining,
                standoff * 2.0,
            )

            limits = {
                'max_linear': self.pf('max_linear_speed'),
                'max_lateral': self.pf('max_lateral_speed'),
                'max_angular': self.pf('max_heading_speed'),
                'min_linear': self.pf('min_linear_speed'),
                'min_lateral': self.pf('min_lateral_speed'),
                'min_angular': self.pf('min_heading_speed'),
                'kp_angular': self.pf('kp_heading'),
                'accel': self.pf('linear_accel'),
                'distance_tolerance': self.pf('distance_tolerance'),
                'yaw_tolerance': yaw_tolerance,
            }

            vx, vy, wz, yaw_error, reached = (
                planner.holonomic_command(
                    rx, ry, ryaw,
                    carrot_xy,
                    target_yaw,
                    remaining,
                    limits,
                )
            )

            # -------------------------------------------------
            # El LiDAR manda en la distancia
            #
            # `along` sale de la posicion del marcador por TF, que
            # depende de que marker_length sea correcto. El LiDAR mide
            # el plano de verdad, asi que decide la llegada y es lo que
            # se reporta. Con el tamaño del marcador mal, la geometria
            # se equivoca y esto lo salva.
            # -------------------------------------------------

            front = self.get_front_lidar_range()

            if front is not None:
                final_distance = front
            else:
                final_distance = along

            aligned = abs(yaw_error) <= yaw_tolerance

            centred = abs(lateral) <= self.pf('lateral_tolerance')

            if (
                front is not None and
                front <= stop_distance + self.pf('distance_tolerance') and
                aligned and
                centred
            ):
                reached = True

            # -------------------------------------------------
            # Parada de seguridad
            # -------------------------------------------------

            clearance = self.pf('min_front_clearance')

            if (
                front is not None and
                front < clearance and
                vx > 0.0
            ):

                self.stop_robot()
                goal_handle.abort()

                result = ArucoApproach.Result()

                result.success = False
                result.status = 'BLOCKED'
                result.message = (
                    f'Obstaculo a {front:.3f} m '
                    f'(minimo {clearance:.3f} m)'
                )
                result.final_distance = front

                self.get_logger().error(result.message)

                return result

            # -------------------------------------------------
            # Llegada
            # -------------------------------------------------

            if reached and aligned:

                self.stop_robot()
                goal_handle.succeed()

                result = ArucoApproach.Result()

                result.success = True
                result.status = 'REACHED'
                result.message = (
                    f'Llegada: lidar={final_distance:.3f} m, '
                    f'geometria={along:.3f} m, '
                    f'lateral={lateral:+.3f} m, '
                    f'yaw={math.degrees(yaw_error):+.1f} deg, '
                    f'{elapsed:.1f} s'
                )
                result.final_distance = final_distance

                self.get_logger().info(result.message)

                return result

            # -------------------------------------------------
            # Aviso de estimacion vieja
            #
            # No se vuelve a SEARCHING: la gracia de estimar en odom es
            # justo poder seguir con el marcador tapado un rato. Pero
            # la odometria deriva, asi que hay que decirlo.
            # -------------------------------------------------

            if last_detection_ns is not None:

                stale = (
                    now_ns - last_detection_ns
                ) / 1e9

                if (
                    stale > self.pf('estimate_max_age') and
                    not stale_warned
                ):

                    stale_warned = True

                    self.get_logger().warn(
                        f'Sin detecciones desde hace {stale:.1f} s; '
                        'navegando por odometria.'
                    )

            self.publish_cmd(vx, vy, wz)

            self.send_feedback(
                goal_handle, state,
                final_distance, center_error, elapsed,
            )

            time.sleep(period)

        # =========================================================
        # Apagado
        # =========================================================

        self.stop_robot()

        result = ArucoApproach.Result()

        result.success = False
        result.status = 'SHUTDOWN'
        result.message = 'ROS shutdown'
        result.final_distance = final_distance

        return result


def main(args=None):

    rclpy.init(args=args)

    node = (
        ArucoLidarApproachServer()
    )

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:

        # Al recibir SIGTERM rclpy ya ha invalidado el contexto, asi que
        # este ultimo intento de parar el robot lanza RCLError y ensucia
        # el log con una traza que parece un fallo y no lo es.
        try:
            node.stop_robot()
        except Exception:
            pass

        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
