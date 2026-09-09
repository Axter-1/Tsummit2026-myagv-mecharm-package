#!/usr/bin/env python3

import math
import time
import threading
import traceback

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
        self.declare_parameter('chassis_frame', 'base_footprint')

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

        # Buscar indefinidamente tampoco vale: si el marcador no esta a
        # la vista, o el detector esta caido, o la Nano esta saturada y
        # no llega ni una deteccion, girar sobre el sitio hasta el
        # timeout no ayuda a nadie. Pasado esto se rinde con un estado
        # claro para que el operador sepa que revisar.
        self.declare_parameter(
            'search_giveup_sec',
            45.0
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
            0.60
        )

        # Velocidad angular minima EFECTIVA. Por debajo de esto los
        # motores del myAGV no vencen la friccion estatica: el mando se
        # publica, el robot no se mueve, el error no baja y el control
        # proporcional entra en ciclo limite (tiron, pasada, tiron al
        # otro lado). Cualquier wz no nulo se eleva a este valor.
        # El suelo real de la base son 0.37 rad/s: pidas 0.08 o pidas
        # 0.30, gira a 0.37. Se deja en el valor medido para que el
        # planificador razone con la velocidad que va a conseguir de
        # verdad, no con una que la base no sabe dar.
        self.declare_parameter(
            'min_heading_speed',
            0.37
        )

        # 0.02 rad = 1.15 grados era inalcanzable: en ese borde wz vale
        # 0.03 rad/s, muy por debajo de min_heading_speed. 0.12 rad = 7
        # grados sobra para una aproximacion perpendicular.
        # MEDIDO en el robot: el suelo de giro de la base es 0.37 rad/s
        # (mandos de 0.02 a 0.40 entregan todos lo mismo). Con los
        # ~200 ms de retardo de la tuberia offboard, en lo que llega la
        # orden de parar el robot ya ha girado 0.37*0.2 = 0.074 rad, mas
        # 0.037 de un ciclo de control a 10 Hz. Son 0.111 rad de
        # sobrepasamiento INEVITABLE.
        #
        # Con 0.08 el sobrepasamiento se comia la banda entera: el robot
        # no podia pararse dentro de la tolerancia, salia por el otro
        # lado y corregia al reves. Ese es el baile izquierda-derecha, y
        # por eso la histeresis lo aliviaba sin curarlo (su umbral de
        # salida, 1.5*0.08 = 0.12, es del mismo orden que el
        # sobrepasamiento). La tolerancia tiene que ser MAYOR que el
        # sobrepasamiento; 0.15 deja un 35% de margen.
        #
        # Ojo: la llegada usa esta misma banda a proposito (un criterio
        # mas estricto que la histeresis produce bloqueo), asi que
        # subirla afloja tambien la perpendicularidad de llegada.
        # Medir con scripts/curva_respuesta.py.
        self.declare_parameter(
            'heading_tolerance',
            0.15
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
            0.14
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
            0.18
        )

        # Zona muerta hacia delante, hermana de min_lateral_speed.
        # Sin esto el robot se para a unos centimetros del objetivo sin
        # llegar a cumplir la condicion de parada.
        # MEDIDO: el suelo de avance de la base son 0.07 m/s (mandos de
        # 0.02 a 0.08 entregan todos ~0.07). Con 0.03 el planificador
        # creia ir a menos de la mitad de la velocidad real y se
        # pasaba de largo al llegar. El lateral, medido en 0.035, ya
        # coincidia con lo que habia.
        self.declare_parameter(
            'min_linear_speed',
            0.07
        )

        # Limites del tramo final. Son los minimos fisicamente alcanzables
        # de esta base; pedir menos genera tirones o inmovilidad.
        self.declare_parameter('final_slow_distance', 0.30)
        self.declare_parameter('final_max_linear_speed', 0.07)
        self.declare_parameter('final_max_lateral_speed', 0.035)
        self.declare_parameter('final_max_angular_speed', 0.37)

        # Congela la referencia filtrada antes de que el ArUco llegue al
        # borde optico y las esquinas degraden la estimacion PnP.
        self.declare_parameter('angular_freeze_distance', 0.50)
        self.declare_parameter('angular_freeze_min_samples', 8)
        self.declare_parameter('angular_freeze_tolerance', 0.15)

        # 0.01 m no es alcanzable con la latencia de la tuberia
        # (JPEG -> WiFi -> portatil -> cmd_vel -> WiFi -> motores): a
        # 0.05 m/s el robot recorre ~1.5 cm solo en lo que llega la
        # orden de parar. Con 0.03 se para dentro de la ventana.
        self.declare_parameter(
            'distance_tolerance',
            0.045
        )

        # La tolerancia amplia de control absorbe la latencia de la base,
        # pero no debe decidir la llegada: con 45 mm el robot podia aceptar
        # 0.239 m al pedir 0.200 m. El criterio final es mas estricto y el
        # LiDAR debe seguir mandando hasta entrar en esta banda.
        self.declare_parameter(
            'final_distance_tolerance',
            0.020
        )

        # Sesgo empirico de frenada final. Compensa el avance que queda por
        # latencia y velocidad minima sin alterar la distancia fisica
        # reportada ni la calibracion lidar->bumper.
        self.declare_parameter(
            'final_braking_bias',
            0.055
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

        # Compatibilidad con configuraciones antiguas. La seguridad actual
        # no resta este valor: calcula el borde de salida del rayo contra
        # chassis_footprint usando la TF completa del sensor.
        #
        # 0.081 = bumper -> CENTRO DE GIRO del LiDAR, que es el origen de
        # laser_frame y desde donde el sensor mide sus rangos.
        #
        # Confirmado por dos caminos independientes que coinciden en 1 mm:
        #
        #   a) Cinta + scan. Bumper a 0.430 m del plano del ArUco 2, eco
        #      frontal crudo a 0.511 desde el sensor -> 0.081.
        #   b) Cinta al BORDE del LiDAR: 0.052 del bumper al primer punto
        #      de la circunferencia. Sumando el radio del YDLIDAR X2
        #      (~0.0298, carcasa de 59.5 mm) -> 0.082.
        #
        # El 0.09 anterior era un valor heredado sin medir.
        #
        # Este valor es el que destapo que chassis_footprint estaba mal:
        # el footprint ponia el morro en x=+0.188, la TF pone laser_frame
        # en x=0.065, y su diferencia (0.123) contradecia estos 0.081.
        # Ver el bloque de chassis_footprint.
        self.declare_parameter(
            'lidar_to_front_bumper_m',
            0.081
        )

        # Cuanto mas lejos que el plano esperado se acepta un eco del
        # sector frontal. Por encima es el fondo -- tipicamente la pared
        # detras del marcador -- y falsea la distancia hacia arriba.
        #
        # La banda es de UN SOLO LADO: un eco mas cerca de lo esperado
        # es un obstaculo de verdad y tiene que seguir contando, porque
        # es lo unico que dispara la parada de seguridad.
        #
        # DESACTIVADA POR DEFECTO (0.0), y no por precaucion vaga.
        #
        # La expectativa sale de la geometria de la CAMARA, y la camara
        # esta mal escalada ahora mismo. Medido con cinta: bumper a
        # 0.43 m del plano del ArUco 2. Reconciliando con el LiDAR crudo
        # (0.511 desde el sensor) sale base_link->marcador = 0.576,
        # mientras la camara reporta 0.449. Se queda corta 0.127 m.
        #
        # Como la distancia estimada de un ArUco escala con
        # marker_length, esa proporcion dice que el marcador REAL mide
        # unos 10.3 cm, no los 0.08 configurados.
        #
        # Con esa expectativa mala la banda calcula un limite de 0.464 y
        # RECHAZA el eco bueno de 0.511: tira la medida correcta por
        # fiarse de la equivocada. Encenderla antes de recalibrar
        # empeora las cosas.
        #
        # Orden correcto: medir el marcador con calibre, corregir
        # marker_length y SOLO entonces poner esto a un valor pequeno. Hasta
        # ahi, si el sector
        # midiera el fondo, el aborto por STALLED lo dice en 2 s.
        self.declare_parameter(
            'lidar_front_depth_band',
            0.0
        )

        # Banda del grupo de ecos cercanos que representa la misma cara.
        # El minimo puro cambia de haz al corregir lateralmente y mueve la
        # distancia final varios centimetros aunque el robot no avance.
        self.declare_parameter(
            'lidar_nearest_cluster_band',
            0.04
        )

        # Un haz aislado mas cercano limita la seguridad, pero no debe
        # cambiar la frenada nominal salvo que este separado del grupo de
        # la cara por mas que esta banda.
        self.declare_parameter(
            'lidar_safety_obstacle_margin',
            0.04
        )

        # Por debajo de esta distancia se deja de exigir ver el
        # marcador. No es una concesion: a 0.29 m un ArUco de 8 cm ya no
        # cabe en el encuadre, medido en pista. Exigir vision hasta el
        # final vuelve inalcanzable cualquier parada corta.
        #
        # El marcador esta fijado en odom, asi que la odometria sabe
        # donde esta sin verlo, y el LiDAR sigue midiendo el plano. Se
        # pierde la correccion, no la posicion.
        self.declare_parameter(
            'blind_endgame_distance',
            0.35
        )

        # Cuanto se admite recorrer a ciegas. En METROS y no en
        # segundos: la deriva de odometria crece con la distancia
        # recorrida, y un robot parado esperando no deriva nada. Con el
        # limite en tiempo, quedarse quieto un rato abortaba una
        # aproximacion sana.
        self.declare_parameter(
            'max_blind_travel',
            0.25
        )

        # Cuanto se tolera que el robot este parado sin haber declarado
        # llegada antes de abortar con diagnostico. Dos segundos son
        # cuarenta ciclos a 20 Hz: de sobra para distinguir un bloqueo
        # real de un ciclo de giro puro con traslacion nula.
        self.declare_parameter(
            'stall_timeout',
            2.0
        )

        # El ArUco debe seguir centrado al aceptar el rango LiDAR final:
        # asi se evita parar ante otra superficie del sector frontal.
        self.declare_parameter(
            'final_camera_center_tolerance',
            0.12
        )

        # Ganancia del centrado lateral final usando la posicion del ArUco
        # en la imagen. Cuando el LiDAR ya esta en la distancia de parada,
        # no se debe seguir avanzando solo porque la geometria TF discrepe;
        # se corrige de lado hasta devolver el marcador al centro.
        self.declare_parameter(
            'final_camera_lateral_kp',
            0.12
        )

        # Criterio final contra la normal del plano. No usar
        # `yaw_settled` como criterio de llegada: ese estado tambien puede
        # significar que se dejo de girar para priorizar la traslacion.
        self.declare_parameter(
            'final_heading_tolerance',
            0.12
        )

        # El rumbo debe permanecer dentro de la banda durante varios ciclos
        # para evitar aceptar una lectura favorable aislada.
        self.declare_parameter(
            'final_heading_settle_sec',
            0.30
        )

        self.declare_parameter('final_distance_settle_sec', 0.25)

        # No se acepta una buena distancia mientras aun hay movimiento. La
        # odometria se usa como comprobacion de la velocidad real y el mando
        # como comprobacion inmediata del controlador.
        self.declare_parameter('final_linear_velocity_tolerance', 0.015)
        self.declare_parameter('final_angular_velocity_tolerance', 0.03)
        self.declare_parameter('final_velocity_settle_sec', 0.20)

        # Distancia de entrada a la fase final: aqui se detiene la
        # traslacion cuando el rumbo aun no esta asentado y se gira en
        # sitio usando la normal fresca del LiDAR.
        self.declare_parameter(
            'final_alignment_distance',
            0.35
        )

        self.declare_parameter(
            'final_lidar_normal_max_age',
            0.75
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
        # Normal por LiDAR (opcional; para comparar contra ArUco)
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
            False
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
        # Profundidad que se acepta por detras del punto mas cercano.
        # TIENE QUE SER MENOR QUE LA SEPARACION entre el marcador y lo
        # que haya detras, o la banda se traga las dos superficies y la
        # SVD ajusta una recta a traves de las DOS.
        #
        # Medido: con el ArUco sobre una caja a 13 cm de la pared y la
        # banda en 0.30, salian 360 avisos de 'superficie no plana' en
        # una sola corrida, con residuos de 0.023 a 0.086 m contra un
        # umbral de 0.02 -- justo el orden de una mezcla de dos planos
        # separados 13 cm. La normal se descartaba casi siempre y el
        # servidor caia a la del ArUco, que es la que sufre la
        # ambiguedad planar: rumbo objetivo saltando y robot sin
        # asentarse nunca.
        #
        # 0.08 deja fuera la pared con margen y sigue muy por encima
        # del ruido de alcance del LiDAR (1-2 cm). Si el montaje cambia
        # y el marcador queda mas pegado a la pared, hay que bajarlo
        # mas: el criterio es la SEPARACION, no un valor fijo.
        self.declare_parameter(
            'lidar_normal_depth_band',
            0.08
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
            # 0.05, no 0.12. El pasillo marca cuanto se admite estar
            # fuera del eje de la normal al entrar en el tramo recto, y
            # ese desvio se paga como un giro en seco al final, cuando
            # el marcador ya llena el encuadre.
            #
            # 0.12 a 0.346 m del marcador eran 19 grados: el robot
            # giraba de golpe, el ArUco se salia de la imagen y acababa
            # torcido respecto al marcador. Medido en pista, con salto
            # del centro normalizado de 0.55.
            #
            # 0.05 = final * tan(heading_tolerance), o sea el desvio
            # mas ancho que la tolerancia de rumbo puede absorber sin
            # pedir un giro. check_tolerances lo comprueba al arrancar.
            #
            # 0.04 y no 0.05 porque el limite depende de
            # lidar_to_front_bumper_m: con el 0.081 medido con cinta el
            # punto final queda mas cerca del marcador, el mismo desvio
            # lateral abarca mas angulo, y el pasillo tiene que
            # estrecharse con el. Lo canta el arranque.
            0.04
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

        # ...y sin detecciones durante mas de ESTO, se ABORTA. Estimar
        # en odom permite coastear con el marcador tapado un rato -- esa
        # es la gracia. Pero la odometria deriva: pasado cierto punto la
        # estimacion ya no dice donde esta el marcador, solo donde
        # estaba menos la deriva acumulada, y seguir es conducir a
        # ciegas. Antes se agotaba el timeout entero (180 s) navegando
        # contra una estimacion muerta, o declarando una llegada falsa.
        self.declare_parameter(
            'estimate_abort_age',
            8.0
        )

        # =========================================================
        # ALIGN_PERPENDICULAR
        #
        # Etapa previa a la aproximacion: ponerse de frente al PLANO del
        # marcador y sobre su eje normal, antes de avanzar hacia el.
        #
        # POR QUE UNA ETAPA APARTE, SI build_path YA LLEGA PERPENDICULAR
        # ---------------------------------------------------------------
        # El camino de dos tramos deja la llegada perpendicular POR
        # CONSTRUCCION, y eso sigue siendo cierto. Lo que no da es
        # perpendicularidad al EMPEZAR: el primer tramo se recorre
        # mirando al marcador (desired_heading mezcla rumbo y normal con
        # `remaining`), asi que el robot ataca el pasillo oblicuo y toda
        # la correccion de encare se paga al final, ya cerca, donde el
        # margen de maniobra y el encuadre son minimos.
        #
        # Alinear ANTES cuesta unos segundos parados y quita ese pago
        # tardio. No sustituye al camino: lo alimenta ya encarado.
        #
        # NO ES EL VIEJO ALIGN_HEADING -> ALIGNING_LATERAL
        # ------------------------------------------------
        # Aquella maquina corregia UN grado de libertad cada vez contra
        # el error instantaneo de camara, y en mecanum eso se persigue
        # la cola. Aqui los dos lazos trabajan contra la estimacion
        # fijada en ODOM, la traslacion sale siempre como vector (vx e
        # vy juntos, en diagonal si hace falta) y la unica multiplexacion
        # es giro-o-traslacion, que la impone la placa (ver la tabla de
        # tres ejes en holonomic_command), no el diseño.
        # =========================================================

        self.declare_parameter('align_enabled', True)

        # Tolerancia de PERPENDICULARIDAD. No confundir con
        # center_keep_margin, que mide centrado en imagen: son errores
        # distintos y se calculan por separado (perpendicular_errors).
        #
        # El suelo de giro son 0.37 rad/s y el lazo tarda
        # command_latency + periodo en reaccionar: 0.37 * 0.32 = 0.118
        # rad de sobrepasamiento. Por debajo de eso la tolerancia es
        # inalcanzable y el robot oscila. check_tolerances lo verifica.
        self.declare_parameter('align_yaw_tolerance', 0.13)
        self.declare_parameter('align_yaw_hysteresis', 1.6)

        # Tolerancia de CENTRADO sobre el eje normal, en metros.
        self.declare_parameter('align_lateral_tolerance', 0.05)
        self.declare_parameter('align_lateral_hysteresis', 1.6)

        # Si la etapa regula tambien la separacion al plano o la deja
        # entera para la aproximacion. Con True se coloca en el punto de
        # encare; la banda es ancha a proposito para no pelearse con el
        # perfil de frenado de APPROACH.
        self.declare_parameter('align_regulate_distance', True)
        self.declare_parameter('align_standoff_tolerance', 0.10)

        self.declare_parameter('align_kp_angular', 1.2)
        self.declare_parameter('align_kp_linear', 0.6)
        self.declare_parameter('align_kp_lateral', 0.9)

        self.declare_parameter('align_max_angular_speed', 0.45)
        self.declare_parameter('align_max_linear_speed', 0.09)
        self.declare_parameter('align_max_lateral_speed', 0.10)

        # Cuanto tiempo seguido tienen que estar los dos errores en
        # banda para dar la alineacion por buena. Sin esto se declara
        # alineado en el cruce por cero de una oscilacion.
        self.declare_parameter('align_settle_sec', 0.35)

        # Presupuesto de la etapa. Agotado, se pasa a APPROACH con lo
        # que haya: la aproximacion sabe corregir, solo que mas tarde.
        self.declare_parameter('align_timeout_sec', 25.0)

        # Tras completar la alineacion, ventana en la que APPROACH NO
        # puede volver a tocar el rumbo por centrado de camara. Sin
        # ella, el bloque "girar lo menos posible" reevalua yaw_settled
        # con center_x en el primer ciclo y deshace el encare recien
        # conseguido con un giro en seco.
        self.declare_parameter('align_handoff_grace_sec', 1.5)

        # Por debajo de esta distancia al plano no se alinea: el
        # marcador ya no cabe en el encuadre y el endgame del
        # aproximador tiene sus propios criterios, mas finos.
        self.declare_parameter('align_min_distance', 0.30)

        # ---- Tolerancia a perdida de vision durante la alineacion ----

        # Edad maxima de la ultima pose fiable para seguir corrigiendo
        # sin ver el marcador.
        self.declare_parameter('align_max_pose_age', 2.0)

        # ...y limite en METROS recorridos sin verlo. En metros porque
        # la deriva de odometria crece con la distancia, no con la
        # espera: un robot parado no deriva.
        self.declare_parameter('align_max_blind_travel', 0.15)

        # Calidad minima de la estimacion (TargetEstimate.quality) para
        # fiarse de ella a ciegas. Una pose reciente tomada en mitad de
        # una racha de rechazos no vale aunque sea reciente.
        self.declare_parameter('align_min_quality', 0.5)

        # Recuperacion: girar para volver a meter el marcador en el
        # encuadre, apuntando a la posicion RECORDADA en odom. No es una
        # busqueda a ciegas: se sabe hacia donde mirar.
        self.declare_parameter('align_recovery_angular_speed', 0.40)
        self.declare_parameter('align_recovery_timeout_sec', 6.0)
        self.declare_parameter('align_max_attempts', 3)

        # ---- Vuelta de APPROACH a ALIGN_PERPENDICULAR ----

        # Error de perpendicularidad que obliga a realinear en mitad de
        # la aproximacion. Muy por encima de align_yaw_tolerance: la
        # separacion entre los dos umbrales ES la histeresis que evita
        # el pinponeo entre etapas.
        self.declare_parameter('realign_yaw_threshold', 0.35)

        # ...y cuanto tiene que persistir. Un pico de un ciclo es ruido
        # del estimador, no un desvio real.
        self.declare_parameter('realign_persist_sec', 0.6)
        self.declare_parameter('max_realign_cycles', 2)

        # Periodo del volcado de diagnostico de la etapa, en segundos.
        # 0.0 lo apaga. Es un throttle: el lazo va a 20 Hz y sacar una
        # linea por ciclo tapa cualquier otro mensaje.
        self.declare_parameter('align_log_period', 0.5)

        # Histeresis del giro. La tolerancia fina hace de umbral de
        # ENTRADA en asentado, y esta por el de SALIDA. Sin los dos
        # umbrales el giro castañea: la zona muerta obliga a mandar el
        # minimo en cuanto se sale de tolerancia, ese minimo se pasa de
        # largo, y al ciclo siguiente hay que corregir al otro lado.
        # Cuanto se deja alejar el marcador del centro del encuadre
        # antes de gastar un ciclo en girar. En unidades de
        # center_x_normalized, que va de -1 a +1. La aproximacion debe
        # salir ya alineada con el ArUco; un margen grande deja avanzar
        # oblicuo y obliga al LiDAR a corregir demasiado tarde.
        #
        # Subirlo = menos giros y aproximacion mas rapida, pero mas
        # riesgo de perder el marcador. Bajarlo = lo contrario.
        self.declare_parameter(
            'center_keep_margin',
            0.12
        )

        # Distancia por debajo de la cual el rumbo vuelve a mandar, para
        # que la llegada quede perpendicular. Por encima se prioriza
        # avanzar; por debajo ya casi no queda traslacion que perder.
        self.declare_parameter(
            'yaw_free_until',
            0.35
        )

        self.declare_parameter(
            'yaw_hysteresis',
            1.5
        )

        # Salto maximo admisible en la normal entre muestras. La pose de
        # un ArUco plano es AMBIGUA: hay dos soluciones simetricas
        # respecto a la linea de vision y OpenCV salta entre ellas de un
        # fotograma a otro, con lo que la normal salta y el robot gira a
        # izquierda y derecha persiguiendola.
        self.declare_parameter(
            'max_normal_jump_deg',
            35.0
        )

        # El marcador es fijo en odom. Un salto grande de su POSICION no
        # es movimiento real: suele ser una solucion de pose ArUco mala o
        # una TF con marca temporal inconsistente. No debe cambiar el
        # carrot en un solo ciclo.
        self.declare_parameter(
            'max_position_jump',
            0.12
        )

        # Retardo del lazo: mando (portatil) -> actuacion. MEDIDO con
        # rosbag durante una aproximacion: escalon de /cmd_vel_aruco a
        # /odom = 0.223 s SIN la red de vuelta; con Tailscale portatil->
        # Jetson por delante, ~0.25-0.30. Se usa 0.27.
        self.declare_parameter(
            'command_latency',
            0.27
        )

        # Tasa a la que llegan los ecos frontales del LiDAR. El lazo va
        # a control_rate (20 Hz) pero el scan a ~8, asi que la distancia
        # con la que se decide frenar puede estar hasta 1/8 s rancia.
        # Entra en la compensacion de inercia (brake_target).
        self.declare_parameter(
            'lidar_rate_hint_hz',
            8.0
        )

        # Por debajo de esta distancia al marcador (geometria), la
        # distancia la manda el eco MAS CERCANO del sector frontal en
        # vez de la mediana: cerca y de frente, lo mas cercano es el
        # plano del marcador y no lo enturbia el fondo.
        self.declare_parameter(
            'lidar_nearest_below',
            0.60
        )

        # Parada de seguridad por LiDAR frontal. Debe quedar por debajo de
        # la parada calibrada mas corta (rueda: 0.09 m), para permitir que
        # el controlador termine de alinear dentro de su tolerancia.
        self.declare_parameter(
            'min_front_clearance',
            0.07
        )

        # Umbral operativo adelantado para compensar el retardo entre el
        # scan, el controlador y el frenado de la base. No reduce el minimo
        # fisico: hace que la parada normal se solicite antes de alcanzarlo.
        self.declare_parameter(
            'chassis_clearance_stop_margin',
            0.015
        )

        # Por debajo de este despeje ya no se considera una parada normal:
        # hay que informar BLOCKED y no permitir ninguna continuacion.
        self.declare_parameter(
            'emergency_chassis_clearance',
            0.040
        )

        # El nombre antiguo era ambiguo: esta magnitud es el despeje entre
        # el eco y el borde del chasis, no la distancia del LiDAR a la pared.
        # Se conserva como alias para configuraciones existentes.
        self.declare_parameter(
            'min_chassis_clearance',
            -1.0
        )

        # Poligono del myAGV en chassis_frame. Aqui se intersecta el rayo
        # real del LiDAR con sus vertices, incluyendo x/y/yaw del montaje
        # del sensor.
        #
        # MORRO: 0.147, NO 0.188
        # ======================
        # El 0.188 heredado de la configuracion de Nav2 ponia el morro
        # 4.2 cm mas adelante de donde esta. Cadena de medidas que lo
        # corrige, sin ninguna suposicion sobre donde cae base_footprint
        # dentro del chasis:
        #
        #   TF leida en el robot        base_footprint -> laser_frame
        #                               x = 0.065  (z = 0.080, yaw = pi)
        #   Cinta al borde del LiDAR    morro -> primer punto de la
        #                               circunferencia = 0.052
        #   Radio del YDLIDAR X2        0.0298 (carcasa de 59.5 mm)
        #
        #   morro = 0.065 + 0.052 + 0.0298 = 0.147
        #
        # Corrobora el 0.081 de morro->centro del LiDAR que ya salia de
        # reconciliar cinta y scan (ver lidar_to_front_bumper_m): dos
        # caminos independientes, 1 mm de diferencia.
        #
        # TRASERA: -0.183
        # Largo total medido con cinta = 0.330 -> 0.147 - 0.330.
        # O sea que base_footprint NO esta en el centro geometrico del
        # chasis, sino 1.8 cm por delante. Medir media eslora (0.165) y
        # asumir simetria daba 0.100 de morro->LiDAR, que contradice las
        # dos medidas directas.
        #
        # POR QUE IMPORTA -- ERA LA CAUSA DE LAS PARADAS CORTAS
        # =====================================================
        # chassis_clearance_for_scan() calcula el despeje intersectando
        # el eco con ESTE poligono. Con el morro 4.2 cm adelantado, el
        # despeje sale 4.2 cm MENOR que el real y choca contra el suelo
        # de min_front_clearance (0.07) mucho antes de tiempo: parada
        # segura, o STALLED, con el robot todavia lejos. Es el sintoma
        # que se venia observando como "por debajo de 0.07 de despeje la
        # aproximacion falla siempre".
        #
        # ANCHO: 0.130 SIN VERIFICAR. Es el valor de Nav2 y no se ha
        # medido. No afecta a una aproximacion frontal, pero si al
        # laberinto.
        #
        # DIVERGE DE NAV2 A PROPOSITO: el footprint de nav2_maze.yaml
        # lleva ~1.5 cm de margen incorporado, que para inflar un
        # costmap esta bien. Aqui hace falta el poligono REAL, porque el
        # margen operativo ya lo pone chassis_clearance_stop_margin
        # (0.015). Sumar los dos era contar el margen dos veces.
        self.declare_parameter(
            'chassis_footprint',
            [0.147, 0.130, 0.147, -0.130,
             -0.183, -0.130, -0.183, 0.130]
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

        # Caches de base_link/footprint <- laser_frame: (x, y, yaw).
        self._laser_to_base = None
        self._laser_to_chassis = None
        raw_footprint = self.get_parameter('chassis_footprint').value
        try:
            if len(raw_footprint) < 6 or len(raw_footprint) % 2:
                raise ValueError
            self.chassis_footprint = [
                (float(raw_footprint[i]), float(raw_footprint[i + 1]))
                for i in range(0, len(raw_footprint), 2)
            ]
        except (TypeError, ValueError):
            raise RuntimeError(
                'chassis_footprint debe ser una lista plana de pares x,y'
            )

        # Motivo del ultimo fallo del lidar. WAITING_LIDAR era una caja
        # negra: no distinguia "el scan no llega" de "llega pero el
        # sector que miro esta vacio", que piden arreglos opuestos.
        self._lidar_fail = None

        # Contexto del goal actual. Se usa exclusivamente para que una
        # excepcion inesperada no termine en el resultado ROS generico y
        # vacio que publica ActionServer cuando el callback revienta.
        self._active_goal_context = {}

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

        # Al ARRANCAR, no al aceptar el primer goal: una configuracion
        # imposible tiene que verse ya, sin tener que lanzar nada.
        self.check_tolerances(
            1.0 / max(1.0, self.pf('control_rate'))
        )
        self.check_detection_freshness()

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

    def get_robot_velocity(self):
        with self.lock:
            odom = self.latest_odom
        if odom is None:
            return None
        twist = odom.twist.twist
        return (
            math.hypot(float(twist.linear.x), float(twist.linear.y)),
            abs(float(twist.angular.z)),
        )

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

        normal = None
        if self.get_parameter('use_marker_normal').value:
            normal = self.get_marker_normal(target_id)

        if normal is None:
            # Sin normal fiable, conservar la posicion y aproximar por la
            # linea de vision actual. La normal saliente apunta marcador ->
            # robot; asi se avanza hacia el ArUco sin inventar un rumbo a
            # partir de una pose plana ambigua.
            robot_pose = self.get_robot_pose()
            if robot_pose is None:
                return None
            dx = robot_pose[0] - mx
            dy = robot_pose[1] - my
            distance = math.hypot(dx, dy)
            if distance < 1e-6:
                return None
            return mx, my, dx / distance, dy / distance

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

    def get_laser_to_chassis(self):
        """TF del sensor al frame donde estan definidos los vertices."""
        if self._laser_to_chassis is not None:
            return self._laser_to_chassis
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('chassis_frame').value,
                'laser_frame',
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return None
        self._laser_to_chassis = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw_from_quaternion(transform.transform.rotation),
        )
        return self._laser_to_chassis

    def planner_stop_distance(self, lidar_distance, normal=None):
        """Convierte distancia LiDAR-pared en una distancia de planificacion.

        Este valor solo se usa para el camino geométrico. La llegada la
        decide ``stop_distance`` en el frame LiDAR y la seguridad el
        footprint, porque una conversion escalar no vale para un sensor
        descentrado u orientado.
        """
        laser_to_base = self.get_laser_to_base()
        if laser_to_base is None:
            return lidar_distance
        laser_x, laser_y, _ = laser_to_base
        if normal is None:
            # Solo para la comprobacion de configuracion antigua.
            return lidar_distance + self.pf('lidar_to_front_bumper_m') + laser_x
        nx, ny = normal
        # stop_distance es LiDAR->pared. La base debe quedar en la
        # posicion que produce ese rango segun la proyeccion real del
        # sensor sobre la normal, no segun una resta fija frontal.
        return lidar_distance - (laser_x * nx + laser_y * ny)

    def chassis_clearance_for_scan(self, scan, index, distance):
        """Despeje del chasis para un haz concreto del LiDAR."""
        laser_to_base = self.get_laser_to_chassis()
        if laser_to_base is None:
            return None
        offset_x, offset_y, laser_yaw = laser_to_base
        angle = scan.angle_min + index * scan.angle_increment
        direction = (
            math.cos(angle + laser_yaw),
            math.sin(angle + laser_yaw),
        )
        exit_distance = planner.ray_polygon_exit_distance(
            (offset_x, offset_y), direction, self.chassis_footprint
        )
        if exit_distance is None:
            return None
        return max(0.0, float(distance) - exit_distance)

    def get_front_lidar_observation(
        self, expected=None, nearest=False, robust_nearest_mode=False
    ):
        """Devuelve ``(distancia_lidar, despeje_chasis)`` coherentes.

        La primera magnitud es el rango medido desde el LiDAR hasta la
        pared. La segunda usa el mismo haz y la interseccion de ese haz con
        el footprint, por lo que no depende de una resta fija.
        """
        now_ns = self.get_clock().now().nanoseconds
        with self.lock:
            scan = self.latest_scan
            stamp_ns = self.latest_scan_time_ns
        if scan is None or stamp_ns is None:
            return None
        if (now_ns - stamp_ns) / 1e9 > self.pf('scan_timeout'):
            return None

        half_angle = math.radians(self.pf('lidar_sector_half_angle_deg'))
        front_angle = self.get_lidar_front_angle(scan)
        values = []
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(normalize_angle(angle - front_angle)) > half_angle:
                continue
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            values.append((float(distance), index))
        if not values:
            return None

        gated_values = planner.plane_returns(
            [value for value, _ in values],
            expected,
            self.pf('lidar_front_depth_band'),
        )
        if not gated_values:
            return None
        if nearest:
            selected_range = min(gated_values)
        elif robust_nearest_mode:
            selected_range = planner.robust_nearest(
                gated_values, self.pf('lidar_nearest_cluster_band')
            )
        else:
            selected_range = float(np.median(gated_values))

        selected_distance, selected_index = min(
            values, key=lambda item: abs(item[0] - selected_range)
        )
        clearance = self.chassis_clearance_for_scan(
            scan, selected_index, selected_distance
        )
        return selected_distance, clearance

    def get_front_lidar_range(
        self,
        expected=None,
        nearest=False,
        robust_nearest_mode=False,
    ):

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

        # El sector mide lo que haya delante, no "el marcador". Con el
        # ArUco sobre una caja y la pared detras, la mayoria de los ecos
        # son de la pared y la MEDIANA se va con ellos: en pista dio
        # 0.511 m (pared) donde la geometria decia 0.384 (marcador), y
        # el goal expiro creyendo que faltaban 11.6 cm ya recorridos.
        # Un 'min' habria acertado de chiripa; la mediana falla siempre.
        gated = planner.plane_returns(
            values,
            expected,
            self.pf('lidar_front_depth_band'),
        )

        if not gated:
            # No inventar un numero: que el llamante sepa que el plano
            # esperado no esta y decida el con la geometria.
            self._lidar_fail = (
                f'SIN_PLANO(esperado={expected:.3f}m, '
                f'mas cerca={min(values):.3f}m)'
            )
            return None

        self._lidar_fail = None

        if nearest:
            # Modo de seguridad: si la geometria de la camara esta mal, el
            # eco mas cercano sigue siendo un limite valido para no avanzar
            # contra un obstaculo o el plano del marcador.
            return float(min(gated))

        if robust_nearest_mode:
            return planner.robust_nearest(
                gated,
                self.pf('lidar_nearest_cluster_band'),
            )

        return float(np.median(gated))

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

    def log_stage_diagnostics(
        self,
        state,
        phase,
        detection,
        reliable,
        detection_age,
        blind_travel,
        yaw_error,
        lateral_error,
        along,
        vx, vy, wz,
        lidar_distance,
        loss_reason,
        transition_reason,
    ):
        """Una linea por volcado con todo lo que hace falta para juzgar.

        Sale por throttle (align_log_period), no por ciclo: el lazo va a
        20 Hz y una linea por ciclo tapa cualquier otro mensaje del
        nodo, que es como se perdieron los avisos de LiDAR en pista.
        """
        period = self.pf('align_log_period')

        if period <= 0.0:
            return

        pose_txt = (
            f'({reliable.x:+.3f}, {reliable.y:+.3f}) '
            f'n=({reliable.nx:+.2f}, {reliable.ny:+.2f}) '
            f'q={reliable.quality:.2f} n_muestras={reliable.samples}'
            if reliable is not None else '<sin pose fiable>'
        )

        actual_txt = (
            f'centro={detection.center_x_normalized:+.3f} '
            f'z={detection.distance_z:.3f} m'
            if detection is not None else '<sin deteccion>'
        )

        edad_txt = (
            f'{detection_age:.2f} s'
            if detection_age != float('inf') else 'nunca'
        )

        self.get_logger().info(
            f'[{state}/{phase}] '
            f'aruco_actual: {actual_txt} | '
            f'ultima_pose_fiable(odom): {pose_txt} | '
            f'edad_deteccion={edad_txt} '
            f'recorrido_ciego={blind_travel:.3f} m | '
            f'err_angular={math.degrees(yaw_error):+.1f} deg '
            f'err_lateral={lateral_error:+.3f} m '
            f'perpendicular={along:.3f} m | '
            f'cmd=({vx:+.3f}, {vy:+.3f}, {wz:+.3f}) | '
            f'lidar={lidar_distance:.3f} m | '
            f'perdida={loss_reason or "-"} | '
            f'ultima_transicion={transition_reason or "-"}',
            throttle_duration_sec=period,
        )

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

    def check_tolerances(self, period):
        """Avisa si alguna tolerancia es inalcanzable por aritmetica.

        La base tiene un SUELO de velocidad por debajo del cual no
        modula. A ese suelo, entre el retardo del lazo y el ciclo en
        curso, el robot recorre una distancia DESPUES de decidir
        pararse. Si la tolerancia es menor que eso, el objetivo es
        inalcanzable: sale por el otro lado y corrige al reves.

        Ese fue exactamente el baile izquierda-derecha del giro
        (suelo 0.37 rad/s, retardo 200 ms -> 0.093 rad de
        sobrepasamiento contra una tolerancia de 0.08). Costo varias
        sesiones y una prueba de pista descubrirlo. Comprobarlo al
        arrancar cuesta cuatro lineas.
        """
        latency = self.pf('command_latency')
        lidar_dt = 1.0 / max(1.0, self.pf('lidar_rate_hint_hz'))

        for nombre, tol, suelo, unidad, extra in (
            ('heading_tolerance',
             math.radians(0.0) + self.pf('heading_tolerance'),
             self.pf('min_heading_speed'), 'rad', 0.0),
            # La distancia lleva compensacion de inercia (brake_target),
            # asi que el robot NO se pasa por la latencia -- pero la
            # frenada se decide con un eco de LiDAR que puede estar
            # lidar_dt rancio, y ESO no se compensa. Ese es el residuo.
            ('distance_tolerance',
             self.pf('distance_tolerance'),
             self.pf('min_linear_speed'), 'm', lidar_dt),
            ('lateral_tolerance',
             self.pf('lateral_tolerance'),
             self.pf('min_lateral_speed'), 'm', 0.0),
            # La etapa de alineacion tiene sus propias tolerancias y el
            # mismo problema aritmetico: por debajo del recorrido
            # residual el lazo no puede asentarse, solo oscilar.
            ('align_yaw_tolerance',
             self.pf('align_yaw_tolerance'),
             self.pf('min_heading_speed'), 'rad', 0.0),
            ('align_lateral_tolerance',
             self.pf('align_lateral_tolerance'),
             self.pf('min_lateral_speed'), 'm', 0.0),
        ):

            parada = suelo * (
                lidar_dt + period if extra > 0.0
                else latency + period
            )

            if tol < parada:

                self.get_logger().error(
                    f'{nombre}={tol:.3f} {unidad} es INALCANZABLE: a su '
                    f'suelo de {suelo:.3f} el robot recorre '
                    f'{parada:.3f} {unidad} tras mandarle parar. '
                    f'Subelo por encima de {parada:.3f} o baja el suelo, '
                    'o el control oscilara sin asentarse nunca.'
                )

        # -------------------------------------------------
        # El pasillo tiene que caber en la tolerancia de rumbo
        #
        # Dentro del pasillo se va RECTO al objetivo, y cerca del final
        # el rumbo deseado pasa de "apuntar al marcador" a "apuntar por
        # la normal". Si el pasillo admite estar muy fuera del eje, esas
        # dos direcciones difieren mucho y el cambio es un giro en seco
        # justo cuando el marcador ya llena el encuadre: se sale de la
        # imagen, se pierde, y el robot acaba torcido respecto al
        # marcador.
        #
        # Medido en pista: pasillo 0.12 a 0.346 m del marcador son 19
        # grados de giro, que mueven el centro normalizado 0.64. Se
        # observo un salto de 0.55 y la deteccion se perdio.
        # -------------------------------------------------

        corridor = self.pf('corridor_radius')
        final = self.planner_stop_distance(
            self.pf('default_stop_distance')
            if self.has_parameter('default_stop_distance')
            else 0.20
        )
        maximo = final * math.tan(self.pf('heading_tolerance'))

        if corridor > maximo:

            desvio = math.degrees(math.atan2(corridor, final))

            self.get_logger().error(
                f'corridor_radius={corridor:.3f} m es DEMASIADO ANCHO '
                f'para heading_tolerance={self.pf("heading_tolerance"):.3f} '
                f'rad: permite entrar al tramo recto {desvio:.1f} grados '
                f'fuera de la normal, y ese error se paga como un giro '
                f'en seco al final que saca el marcador del encuadre. '
                f'Bajalo a {maximo:.3f} o menos.'
            )

    def check_detection_freshness(self):
        """Avisa si el abort por estimacion rancia no puede dispararse.

        get_detection() descarta una deteccion en cuanto pasa de
        detection_timeout, asi que ESE es el tiempo real que tarda el
        servidor en darse cuenta de que ha perdido el marcador. Los
        cortes por estimacion vieja (estimate_max_age para el aviso,
        estimate_abort_age para el aborto) se miden desde la ULTIMA
        deteccion aceptada, luego solo tienen sentido si son mayores
        que detection_timeout. Si alguien sube detection_timeout por
        encima de estimate_abort_age, get_detection sigue dando la
        deteccion vieja por fresca, last_detection_ns se refresca sola,
        y el aborto no salta nunca -- se vuelve al fallo que este mismo
        corte venia a arreglar. Misma familia que check_tolerances.
        """
        timeout = self.pf('detection_timeout')

        for nombre, valor in (
            ('estimate_max_age', self.pf('estimate_max_age')),
            ('estimate_abort_age', self.pf('estimate_abort_age')),
        ):

            if valor <= timeout:

                self.get_logger().error(
                    f'{nombre}={valor:.2f} s <= detection_timeout='
                    f'{timeout:.2f} s: get_detection() descarta la '
                    'deteccion antes de que este corte pueda medir '
                    'nada, asi que no disparara. Subelo por encima de '
                    f'{timeout:.2f} o baja detection_timeout.'
                )

    # =============================================================
    # Action
    # =============================================================

    def execute_callback(
        self,
        goal_handle
    ):
        """Captura errores inesperados y devuelve un resultado diagnostico."""
        request = goal_handle.request
        context = {
            "action": "/aruco_lidar_approach",
            "target_id": int(getattr(request, "target_id", 0)),
            "stop_distance": float(getattr(request, "stop_distance", 0.0)),
            "state": "STARTING",
            "final_distance": -1.0,
            "last_detection_age": None,
            "lidar_failure": self._lidar_fail,
        }
        self._active_goal_context = context
        try:
            return self._execute_callback(goal_handle)
        except Exception as exc:  # noqa: BLE001
            try:
                self.stop_robot()
            except Exception as stop_exc:  # noqa: BLE001
                context["stop_error"] = (
                    f"{type(stop_exc).__name__}: {stop_exc!r}"
                )

            try:
                goal_handle.abort()
            except Exception as abort_exc:  # noqa: BLE001
                context["abort_error"] = (
                    f"{type(abort_exc).__name__}: {abort_exc!r}"
                )

            exception_detail = f"{type(exc).__name__}: {exc!r}"
            traceback_text = traceback.format_exc()
            self.get_logger().error(
                "Excepcion no controlada en execute_callback de "
                f"{context['action']}: {exception_detail}\n"
                f"Contexto: {context}\n{traceback_text}"
            )

            result = ArucoApproach.Result()
            result.success = False
            result.status = "INTERNAL_ERROR"
            final_distance = context.get("final_distance", -1.0)
            result.final_distance = float(final_distance)
            result.final_chassis_clearance = -1.0
            age = context.get("last_detection_age")
            age_text = "desconocido" if age is None else f"{age:.3f} s"
            result.message = (
                f"stage=APPROACH; exception_type={type(exc).__name__}; "
                f"exception={exc!r}; state={context.get('state')}; "
                f"target_id={context.get('target_id')}; "
                f"requested_stop={context.get('stop_distance'):.3f} m; "
                f"last_lidar_distance={result.final_distance:.3f} m; "
                f"last_detection_age={age_text}; "
                f"lidar_failure={context.get('lidar_failure') or '<none>'}; "
                "traceback=log del nodo aruco_lidar_approach_server"
            )
            return result
        finally:
            self._active_goal_context = {}

    def _execute_callback(
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

        self._active_goal_context.update(
            target_id=target_id,
            stop_distance=float(goal_handle.request.stop_distance),
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
            max_normal_jump=math.radians(
                self.pf('max_normal_jump_deg')
            ),
            max_position_jump=self.pf('max_position_jump'),
        )

        use_lidar = bool(
            self.get_parameter(
                'use_lidar_normal'
            ).value
        )

        state = 'SEARCHING'

        # ---- ALIGN_PERPENDICULAR ----
        align_enabled = bool(
            self.get_parameter('align_enabled').value
        )
        # Se entra en la etapa en cuanto haya estimacion, no antes: sin
        # saber donde esta el plano no hay nada con que alinearse.
        align_stage = align_enabled
        align_start_ns = None
        align_yaw_settled = False
        align_translation_settled = False
        align_settle_since_ns = None
        align_attempts = 0
        align_reference_yaw = None
        align_handoff_until_ns = None
        reacquire_start_ns = None
        realign_since_ns = None
        realign_cycles = 0

        # Ultima pose del marcador con calidad suficiente para fiarse de
        # ella a ciegas. Es un ReliablePose en ODOM, no un tvec: por eso
        # sigue valiendo cuando el robot se mueve sin ver el marcador --
        # la odometria actualiza la relacion robot-marcador sola.
        reliable = None
        loss_reason = None
        transition_reason = None

        # Ciclos seguidos sin mando y sin llegada declarada. Ver el
        # bloque "Ni avanza ni llega" mas abajo.
        stalled = 0

        # Velocidad de avance del ciclo anterior. Alimenta la
        # compensacion de inercia de la frenada: el robot sigue
        # avanzando ~v*(latency+period+lidar_dt) tras decidir parar, y
        # eso es lo que le hacia pasarse 4 cm (pedir 0.15, quedarse a
        # 0.11).
        last_forward_speed = 0.0

        start_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        search_phase_start_ns = start_ns
        search_moving = True

        last_detection_ns = None
        # Pose del robot en la ultima deteccion. La deriva de odometria
        # crece con la DISTANCIA recorrida, no con el tiempo parado, asi
        # que el tramo a ciegas se acota por metros, no por segundos.
        last_detection_xy = None
        stale_warned = False
        yaw_settled = False

        final_distance = -1.0
        center_error = 0.0
        final_heading_since_ns = None
        final_distance_since_ns = None
        final_velocity_since_ns = None
        last_lidar_outward_normal = None
        last_lidar_normal_ns = None
        frozen_target = None
        frozen_yaw = None

        period = 1.0 / max(
            1.0,
            self.pf('control_rate')
        )

        yaw_tolerance = self.pf('heading_tolerance')

        self.check_tolerances(period)

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
            self._active_goal_context.update(
                state=state,
                final_distance=final_distance,
                lidar_failure=self._lidar_fail,
                last_detection_age=(
                    (now_ns - last_detection_ns) / 1e9
                    if last_detection_ns is not None else None
                ),
            )

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
                result.final_chassis_clearance = -1.0

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
                result.final_chassis_clearance = -1.0

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
                    loss_reason = None

                    # La pose se pide aqui explicitamente: este bloque
                    # corre tambien en SEARCHING, donde rx/ry todavia no
                    # existen (se desempaquetan mas abajo, ya en
                    # PURSUING). Usarlas aqui reventaba el goal con
                    # UnboundLocalError y un abort de estado vacio.
                    pose_ahora = self.get_robot_pose()
                    if pose_ahora is not None:
                        last_detection_xy = (pose_ahora[0], pose_ahora[1])

                    # La normal del LiDAR es bastante mejor que la del
                    # ArUco, que en yaw es ruidosa y ambigua de perfil.
                    # Se mete como muestra de mas peso en vez de
                    # sustituir.
                    #
                    # Pero el peso doble NO diluye una normal mala: el
                    # alfa del estimador tiene SUELO (max(alpha, 1/n)),
                    # asi que pasadas unas muestras es un filtro
                    # exponencial fijo: 0.25 con el valor por defecto de
                    # TargetEstimate, o 0.10 con el que pasa este
                    # servidor (estimate_alpha_normal), y por 2 al meter
                    # alpha_scale. Da igual cual: en los dos casos el
                    # sesgo gana (+73.7 y +67.0 grados). Y el
                    # fallo que vimos en el robot no es ruido: la SVD
                    # enganchaba la pared CONTIGUA con coherencia 1.00 y
                    # residuo de milimetros -- misma respuesta erronea
                    # cada ciclo mientras el robot no se mueva. Un sesgo
                    # sistematico a peso doble no se promedia, gana:
                    # simulado con TargetEstimate, una normal a 90 grados
                    # deja la estimacion en +73.7 grados en UN ciclo.
                    #
                    # Por eso vuelve la guarda de oblicuidad, que es lo
                    # que salvo la unica aproximacion completa (rechazo
                    # una normal a 82-99 grados y siguio con la camara).
                    # Una superficie que el robot VE no puede tener su
                    # normal casi perpendicular a la linea de vision.
                    if use_lidar:

                        lidar_normal = (
                            self.get_lidar_surface_normal(
                                target_id
                            )
                        )

                        if lidar_normal is not None:

                            obliquity = self.normal_obliquity(
                                math.atan2(
                                    lidar_normal[1],
                                    lidar_normal[0],
                                ),
                                target_id,
                            )

                            max_obliquity = math.radians(
                                self.pf(
                                    'normal_max_obliquity_deg'
                                )
                            )

                            if (
                                obliquity is not None and
                                obliquity > max_obliquity
                            ):

                                self.get_logger().warn(
                                    'Normal del LiDAR descartada: '
                                    'oblicuidad '
                                    f'{math.degrees(obliquity):.1f} deg '
                                    '> '
                                    f'{self.pf("normal_max_obliquity_deg"):.0f}'
                                    ' deg. Probablemente la pared '
                                    'contigua. Sigo con la del ArUco.'
                                )

                            else:

                                lnx, lny = planner.outward_normal(
                                    lidar_normal[0],
                                    lidar_normal[1],
                                )

                                estimate.update(
                                    mx, my, lnx, lny,
                                    stamp_ns=now_ns,
                                    alpha_scale=2.0,
                                )
                                last_lidar_outward_normal = (lnx, lny)
                                last_lidar_normal_ns = now_ns

            # =====================================================
            # ULTIMA POSE FIABLE
            #
            # Se refresca SOLO mientras la estimacion tiene calidad
            # suficiente. Guardar la ultima sin mas seria guardar
            # tambien la lectura aberrante de justo antes de perder el
            # marcador, que es precisamente la que no hay que recordar:
            # el modo de fallo tipico es que la pose se degrada unos
            # ciclos (marcador de perfil, desenfoque de movimiento) y
            # DESPUES desaparece.
            #
            # El marco es odom. Eso es lo que hace que la referencia
            # siga siendo valida al moverse sin ver el marcador: no se
            # guarda un tvec de camara -- que caduca en cuanto la base
            # se desplaza -- sino un punto fijo del mundo, y es la
            # odometria (via TF odom->base_link) la que actualiza la
            # relacion robot-marcador ciclo a ciclo, sin tocar la pose
            # almacenada.
            # =====================================================

            snapshot = estimate.snapshot(now_ns)

            if (
                snapshot is not None and
                snapshot.quality >= self.pf('align_min_quality')
            ):
                reliable = snapshot

            detection_age = (
                (now_ns - last_detection_ns) / 1e9
                if last_detection_ns is not None else float('inf')
            )

            # =====================================================
            # SEARCHING: sin estimacion no hay a donde ir
            # =====================================================

            if not estimate.ready:

                state = 'SEARCHING'

                # Rendirse si se lleva demasiado buscando sin haber
                # conseguido una sola estimacion. En este estado
                # last_detection_ns es siempre None (a PURSUING no se
                # vuelve), asi que esto es literalmente "nunca vi el
                # marcador".
                if elapsed > self.pf('search_giveup_sec'):

                    self.stop_robot()
                    goal_handle.abort()

                    result = ArucoApproach.Result()

                    result.success = False
                    result.status = 'NO_MARKER'
                    result.message = (
                        f'Marcador {target_id} no encontrado en '
                        f'{elapsed:.1f} s de busqueda. Si deberia estar '
                        'a la vista: revisar el detector (¿publica '
                        '/aruco/detections?) y la carga de CPU.'
                    )
                    result.final_distance = -1.0

                    self.get_logger().error(result.message)

                    return result

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

            rx, ry, ryaw = robot_pose
            estimate_pose = estimate.pose

            # Metros recorridos desde la ultima deteccion aceptada. La
            # deriva de odometria crece con la DISTANCIA, no con el
            # tiempo: un robot parado esperando no deriva nada, asi que
            # el limite del tramo sin vision se mide en metros.
            blind_travel = (
                math.hypot(rx - last_detection_xy[0],
                           ry - last_detection_xy[1])
                if last_detection_xy is not None else 0.0
            )

            # =====================================================
            # ALIGN_PERPENDICULAR
            #
            # Ponerse de frente al PLANO del marcador y sobre su eje
            # normal ANTES de avanzar hacia el. Ver el bloque de
            # parametros align_* para el porque de la etapa y en que se
            # diferencia de la vieja maquina secuencial.
            # =====================================================

            if align_stage:

                state = 'ALIGN_PERPENDICULAR'

                if align_start_ns is None:
                    align_start_ns = now_ns
                    align_settle_since_ns = None
                    self.get_logger().info(
                        'ALIGN_PERPENDICULAR: alineando con el plano del '
                        f'marcador {target_id} '
                        f'(tol {math.degrees(self.pf("align_yaw_tolerance")):.1f} '
                        f'deg / {self.pf("align_lateral_tolerance") * 100:.0f} cm)'
                    )

                align_elapsed = (now_ns - align_start_ns) / 1e9

                # -------------------------------------------------
                # Que referencia se usa: la pose fiable si existe, y si
                # no la estimacion viva. Las dos estan en odom, asi que
                # la eleccion no cambia el marco, solo la confianza.
                # -------------------------------------------------
                if reliable is not None:
                    amx, amy, anx, any_ = reliable.pose
                    align_quality = reliable.quality
                    align_pose_age = (
                        (now_ns - reliable.stamp_ns) / 1e9
                        if reliable.stamp_ns is not None
                        else float('inf')
                    )
                else:
                    amx, amy, anx, any_ = estimate_pose
                    align_quality = estimate.quality
                    align_pose_age = detection_age

                # -------------------------------------------------
                # PERDIDA VISUAL: ni abortar de golpe ni seguir con el
                # ultimo mando. Se juzga la referencia por TRES cosas
                # -- calidad, edad y metros recorridos a ciegas -- y
                # solo si las tres aguantan se sigue corrigiendo.
                # -------------------------------------------------
                pose_usable = (
                    reliable is not None and
                    align_quality >= self.pf('align_min_quality') and
                    align_pose_age <= self.pf('align_max_pose_age') and
                    blind_travel <= self.pf('align_max_blind_travel')
                )

                if detection is None and not pose_usable:

                    if align_quality < self.pf('align_min_quality'):
                        loss_reason = (
                            f'calidad {align_quality:.2f} < '
                            f'{self.pf("align_min_quality"):.2f}'
                        )
                    elif align_pose_age > self.pf('align_max_pose_age'):
                        loss_reason = (
                            f'pose de hace {align_pose_age:.2f} s '
                            f'(limite {self.pf("align_max_pose_age"):.2f})'
                        )
                    else:
                        loss_reason = (
                            f'{blind_travel:.3f} m a ciegas '
                            f'(limite {self.pf("align_max_blind_travel"):.3f})'
                        )

                    # -------------------------------------------------
                    # REACQUIRING: girar para volver a meter el marcador
                    # en el encuadre. NO es una busqueda a ciegas: se
                    # apunta a la posicion RECORDADA en odom, que la
                    # odometria mantiene actualizada aunque no se vea.
                    # -------------------------------------------------
                    if reacquire_start_ns is None:
                        align_attempts += 1
                        reacquire_start_ns = now_ns
                        self.stop_robot()
                        self.get_logger().warn(
                            f'Marcador perdido en ALIGN_PERPENDICULAR: '
                            f'{loss_reason}. Intento de recuperacion '
                            f'{align_attempts}/'
                            f'{int(self.pf("align_max_attempts"))}.'
                        )

                    reacquire_elapsed = (
                        now_ns - reacquire_start_ns
                    ) / 1e9

                    agotado = (
                        align_attempts > int(self.pf('align_max_attempts')) or
                        reacquire_elapsed >
                        self.pf('align_recovery_timeout_sec')
                    )

                    if agotado:

                        # La referencia ya no es de fiar. Se tira y se
                        # vuelve a buscar de cero, que es mas honesto
                        # que seguir corrigiendo contra una pose que ha
                        # dejado de describir el mundo.
                        transition_reason = (
                            'referencia descartada -> SEARCHING '
                            f'({loss_reason})'
                        )

                        self.get_logger().warn(
                            'Recuperacion agotada tras '
                            f'{reacquire_elapsed:.1f} s e intento '
                            f'{align_attempts}: descarto la estimacion y '
                            'vuelvo a buscar.'
                        )

                        self.stop_robot()

                        estimate = planner.TargetEstimate(
                            alpha_position=self.pf(
                                'estimate_alpha_position'
                            ),
                            alpha_normal=self.pf('estimate_alpha_normal'),
                            max_normal_jump=math.radians(
                                self.pf('max_normal_jump_deg')
                            ),
                            max_position_jump=self.pf('max_position_jump'),
                        )

                        reliable = None
                        last_detection_ns = None
                        last_detection_xy = None
                        align_start_ns = None
                        align_yaw_settled = False
                        align_translation_settled = False
                        align_settle_since_ns = None
                        align_attempts = 0
                        reacquire_start_ns = None
                        search_phase_start_ns = now_ns
                        search_moving = True
                        state = 'SEARCHING'

                        self.send_feedback(
                            goal_handle, state,
                            final_distance, center_error, elapsed,
                        )

                        time.sleep(period)
                        continue

                    state = 'REACQUIRING'

                    recovery_yaw = planner.reacquire_heading(
                        rx, ry, amx, amy
                    )

                    wz = 0.0

                    if recovery_yaw is not None:

                        recovery_error = normalize_angle(
                            recovery_yaw - ryaw
                        )

                        wz = clamp(
                            self.pf('align_kp_angular') * recovery_error,
                            -self.pf('align_recovery_angular_speed'),
                            self.pf('align_recovery_angular_speed'),
                        )

                        wz = planner.apply_deadband(
                            wz,
                            self.pf('min_heading_speed'),
                            tolerance_reached=(
                                abs(recovery_error) <=
                                self.pf('align_yaw_tolerance')
                            ),
                        )

                    # Giro puro y nada mas: moverse sin ver el marcador
                    # y sin referencia fresca es justo el movimiento a
                    # ciegas que hay que evitar. Rotar no cambia la
                    # posicion, asi que no acumula deriva de traslacion.
                    self.publish_cmd(wz=wz)

                    self.log_stage_diagnostics(
                        state, 'RECOVERY', detection, reliable,
                        detection_age, blind_travel,
                        0.0, 0.0, -1.0,
                        0.0, 0.0, wz,
                        final_distance, loss_reason, transition_reason,
                    )

                    self.send_feedback(
                        goal_handle, state,
                        final_distance, center_error, elapsed,
                    )

                    time.sleep(period)
                    continue

                # Vision recuperada (o nunca perdida).
                if reacquire_start_ns is not None and detection is not None:
                    self.get_logger().info(
                        'Marcador recuperado tras '
                        f'{(now_ns - reacquire_start_ns) / 1e9:.1f} s; '
                        'sigo alineando.'
                    )
                    reacquire_start_ns = None
                    align_attempts = 0
                    loss_reason = None

                # -------------------------------------------------
                # Ley de control de la etapa
                # -------------------------------------------------
                align_standoff = max(
                    self.pf('staging_standoff'),
                    self.planner_stop_distance(
                        stop_distance, (anx, any_)
                    ) + 0.10,
                )

                align_limits = {
                    'kp_angular': self.pf('align_kp_angular'),
                    'kp_linear': self.pf('align_kp_linear'),
                    'kp_lateral': self.pf('align_kp_lateral'),
                    'max_angular': self.pf('align_max_angular_speed'),
                    'max_linear': self.pf('align_max_linear_speed'),
                    'max_lateral': self.pf('align_max_lateral_speed'),
                    'min_angular': self.pf('min_heading_speed'),
                    'min_linear': self.pf('min_linear_speed'),
                    'min_lateral': self.pf('min_lateral_speed'),
                    'yaw_tolerance': self.pf('align_yaw_tolerance'),
                    'yaw_hysteresis': self.pf('align_yaw_hysteresis'),
                    'lateral_tolerance': self.pf('align_lateral_tolerance'),
                    'lateral_hysteresis': self.pf(
                        'align_lateral_hysteresis'
                    ),
                    'standoff_tolerance': self.pf(
                        'align_standoff_tolerance'
                    ),
                }

                align_cmd = planner.alignment_command(
                    rx, ry, ryaw,
                    amx, amy, anx, any_,
                    align_standoff,
                    align_limits,
                    yaw_settled=align_yaw_settled,
                    translation_settled=align_translation_settled,
                    regulate_distance=bool(
                        self.get_parameter(
                            'align_regulate_distance'
                        ).value
                    ),
                )

                align_yaw_settled = align_cmd.yaw_settled
                align_translation_settled = align_cmd.translation_settled

                # -------------------------------------------------
                # Demasiado cerca para alinear: a partir de aqui el
                # marcador ya no cabe en el encuadre y el endgame del
                # aproximador tiene criterios mas finos que los de esta
                # etapa. Insistir aqui solo quitaria margen.
                # -------------------------------------------------
                terminar = None

                if align_cmd.along < self.pf('align_min_distance'):
                    terminar = (
                        f'demasiado cerca ({align_cmd.along:.3f} m < '
                        f'{self.pf("align_min_distance"):.2f}); '
                        'lo termina APPROACH'
                    )

                elif align_elapsed > self.pf('align_timeout_sec'):
                    terminar = (
                        f'presupuesto agotado ({align_elapsed:.1f} s); '
                        'sigo con APPROACH, que sabe corregir'
                    )

                elif align_cmd.settled:

                    if align_settle_since_ns is None:
                        align_settle_since_ns = now_ns

                    elif (
                        (now_ns - align_settle_since_ns) / 1e9 >=
                        self.pf('align_settle_sec')
                    ):
                        terminar = (
                            'alineado y estable '
                            f'{self.pf("align_settle_sec"):.2f} s'
                        )

                else:
                    align_settle_since_ns = None

                # -------------------------------------------------
                # Seguridad: la etapa se ejecuta lejos del plano, pero
                # no se traslada a ciegas contra un obstaculo.
                # -------------------------------------------------
                align_observation = self.get_front_lidar_observation(
                    None, nearest=True
                )

                if (
                    align_observation is not None and
                    align_observation[1] is not None and
                    align_observation[1] < (
                        (
                            self.pf('min_chassis_clearance')
                            if self.pf('min_chassis_clearance') >= 0.0
                            else self.pf('min_front_clearance')
                        ) + self.pf('chassis_clearance_stop_margin')
                    ) and
                    align_cmd.phase == 'TRANSLATE'
                ):
                    # Solo se anula la TRASLACION. Girar sobre si mismo
                    # no reduce el despeje y es lo unico que puede sacar
                    # al robot de una pose mala.
                    self.get_logger().warn(
                        'Traslacion de alineacion inhibida: despeje '
                        f'{align_observation[1]:.3f} m',
                        throttle_duration_sec=1.0,
                    )
                    align_cmd.vx = 0.0
                    align_cmd.vy = 0.0

                if terminar is not None:

                    # -------------------------------------------------
                    # ENTREGA A APPROACH
                    #
                    # Se pasa el rumbo perpendicular como referencia ya
                    # asentada (yaw_settled=True) y se abre una ventana
                    # de gracia en la que el bloque de centrado por
                    # camara no puede reevaluarlo. Sin eso, APPROACH
                    # recalcula yaw_settled con center_x en el primer
                    # ciclo y deshace el encare con un giro en seco.
                    # -------------------------------------------------
                    align_stage = False
                    align_reference_yaw = math.atan2(-any_, -anx)
                    yaw_settled = True
                    align_handoff_until_ns = now_ns + int(
                        self.pf('align_handoff_grace_sec') * 1e9
                    )
                    align_start_ns = None
                    align_settle_since_ns = None
                    reacquire_start_ns = None
                    transition_reason = (
                        f'ALIGN_PERPENDICULAR -> APPROACH: {terminar}'
                    )

                    self.stop_robot()

                    self.get_logger().info(
                        f'ALIGN_PERPENDICULAR completada ({terminar}): '
                        f'err_angular='
                        f'{math.degrees(align_cmd.yaw_error):+.1f} deg, '
                        f'err_lateral={align_cmd.lateral:+.3f} m, '
                        f'perpendicular={align_cmd.along:.3f} m, '
                        f'yaw_ref={math.degrees(align_reference_yaw):+.1f} '
                        f'deg, {align_elapsed:.1f} s'
                    )

                    # Cae a PURSUING en este mismo ciclo: no hay motivo
                    # para gastar un periodo mas parado.

                else:

                    self.publish_cmd(
                        align_cmd.vx, align_cmd.vy, align_cmd.wz
                    )

                    self.log_stage_diagnostics(
                        state, align_cmd.phase, detection, reliable,
                        detection_age, blind_travel,
                        align_cmd.yaw_error, align_cmd.lateral,
                        align_cmd.along,
                        align_cmd.vx, align_cmd.vy, align_cmd.wz,
                        final_distance, loss_reason, transition_reason,
                    )

                    self.send_feedback(
                        goal_handle, state,
                        final_distance, center_error, elapsed,
                    )

                    time.sleep(period)
                    continue

            state = 'PURSUING'

            if (
                frozen_target is None and
                final_distance > 0.0 and
                final_distance <= self.pf('angular_freeze_distance') and
                estimate.samples >= int(self.pf('angular_freeze_min_samples'))
            ):
                candidate_yaw = math.atan2(-estimate_pose[3], -estimate_pose[2])
                if abs(normalize_angle(candidate_yaw - ryaw)) <= self.pf(
                    'angular_freeze_tolerance'
                ):
                    frozen_target = estimate_pose
                    frozen_yaw = candidate_yaw
                    self.get_logger().info(
                        'Referencia ArUco congelada para tramo final: '
                        f'distancia={final_distance:.3f} m, '
                        f'muestras={estimate.samples}, '
                        f'yaw={math.degrees(frozen_yaw):+.1f} deg'
                    )

            angular_frozen = frozen_target is not None
            mx, my, nx, ny = (
                frozen_target if angular_frozen else estimate_pose
            )

            # -------------------------------------------------
            # Estimacion vieja: avisar, y abortar si es mucho.
            #
            # Se comprueba ANTES de construir el camino o la logica de
            # llegada: contra una estimacion derivada, "he llegado"
            # tambien puede salir falso. Mejor abortar limpio.
            # -------------------------------------------------
            stale = (
                now_ns - (last_detection_ns or now_ns)
            ) / 1e9

            # -------------------------------------------------
            # TRAMO FINAL A CIEGAS
            #
            # Cerca del marcador la camara deja de verlo por geometria,
            # no por fallo: a 0.29 m un ArUco de 8 cm ya se sale del
            # encuadre. Exigir vision hasta el final hace inalcanzable
            # cualquier parada corta, por bien que vaya todo lo demas.
            #
            # No hace falta: el marcador esta fijado en `odom`, asi que
            # la odometria sabe donde esta aunque no se vea, y el LiDAR
            # sigue midiendo la distancia al plano. Lo que se pierde es
            # la CORRECCION, no la posicion.
            #
            # Por eso el limite del tramo a ciegas son METROS RECORRIDOS
            # y no segundos: la deriva de odometria crece con la
            # distancia, y un robot parado esperando no deriva nada. Con
            # el limite en tiempo, quedarse quieto un rato abortaba una
            # aproximacion perfectamente sana.
            # -------------------------------------------------

            # final_distance viene del ciclo ANTERIOR (se calcula mas
            # abajo). A 20 Hz eso es un ciclo de retraso, irrelevante.
            # El > 0.0 no es cosmetico: el valor inicial es -1.0 como
            # centinela de "aun no se sabe", y sin ese filtro el primer
            # ciclo entraria en modo ciego y desactivaria el abort por
            # marcador perdido durante toda la aproximacion.
            blind = (
                final_distance > 0.0 and
                final_distance <= self.pf('blind_endgame_distance')
            )

            # Ya calculado arriba (blind_travel), una sola definicion:
            # tenerlo por duplicado invitaba a que las dos se separaran.
            recorrido_ciego = blind_travel

            if blind and recorrido_ciego > self.pf('max_blind_travel'):

                self.stop_robot()
                goal_handle.abort()

                result = ArucoApproach.Result()

                result.success = False
                result.status = 'LOST'
                result.message = (
                    f'{recorrido_ciego:.3f} m recorridos sin ver el '
                    f'marcador (limite {self.pf("max_blind_travel"):.3f}). '
                    'La odometria ha derivado demasiado para fiarse.'
                )
                result.final_distance = final_distance

                self.get_logger().error(result.message)

                return result

            if not blind and stale > self.pf('estimate_abort_age'):

                self.stop_robot()
                goal_handle.abort()

                result = ArucoApproach.Result()

                result.success = False
                result.status = 'LOST'
                result.message = (
                    f'Marcador perdido {stale:.1f} s (limite '
                    f'{self.pf("estimate_abort_age"):.1f} s) y todavia a '
                    f'{final_distance:.3f} m, lejos del tramo final. '
                    'Abortando en vez de navegar a ciegas. '
                    f'Ultima pose fiable: {reliable!r}'
                )
                result.final_distance = final_distance

                self.get_logger().error(result.message)

                return result

            if (
                stale > self.pf('estimate_max_age') and
                not stale_warned
            ):

                stale_warned = True

                self.get_logger().warn(
                    f'Sin detecciones desde hace {stale:.1f} s; '
                    'navegando por odometria.'
                )

            standoff = self.pf('staging_standoff')
            path_stop_distance = self.planner_stop_distance(
                stop_distance, (nx, ny)
            )
            # El punto de encare debe quedar antes que el objetivo final.
            standoff = max(standoff, path_stop_distance + 0.10)

            path = planner.build_path(
                rx, ry, mx, my, nx, ny,
                standoff,
                path_stop_distance,
                corridor_radius=self.pf('corridor_radius'),
            )

            carrot_xy, remaining, off_path = planner.carrot(
                path, rx, ry,
                self.pf('lookahead_distance'),
            )

            along, lateral = planner.corridor_coords(
                rx, ry, mx, my, nx, ny
            )

            # -------------------------------------------------
            # VUELTA A ALIGN_PERPENDICULAR
            #
            # Si en mitad de la aproximacion aparece un error de
            # perpendicularidad grande, realinear con la etapa dedicada
            # sale mejor que dejar que el lazo lo arregle mezclando
            # giros bruscos con desplazamiento lateral.
            #
            # Dos guardas contra el pinponeo entre etapas:
            #   histeresis  realign_yaw_threshold es MUY superior a
            #               align_yaw_tolerance, asi que salir de la
            #               alineacion no puede disparar la vuelta.
            #   persistencia el error tiene que mantenerse
            #               realign_persist_sec; un pico de un ciclo es
            #               ruido del estimador, no un desvio real.
            # Y un tope duro de vueltas, porque un bucle estable de dos
            # etapas es peor que una aproximacion mediocre.
            # -------------------------------------------------
            if (
                align_enabled and
                not angular_frozen and
                realign_cycles < int(self.pf('max_realign_cycles')) and
                along > max(
                    self.pf('align_min_distance'),
                    self.pf('yaw_free_until'),
                )
            ):

                perpendicular_error = normalize_angle(
                    math.atan2(-ny, -nx) - ryaw
                )

                if (
                    abs(perpendicular_error) >
                    self.pf('realign_yaw_threshold')
                ):

                    if realign_since_ns is None:
                        realign_since_ns = now_ns

                    elif (
                        (now_ns - realign_since_ns) / 1e9 >=
                        self.pf('realign_persist_sec')
                    ):

                        realign_cycles += 1
                        align_stage = True
                        align_start_ns = None
                        align_yaw_settled = False
                        align_translation_settled = False
                        align_settle_since_ns = None
                        align_handoff_until_ns = None
                        realign_since_ns = None
                        transition_reason = (
                            'APPROACH -> ALIGN_PERPENDICULAR: '
                            f'perpendicularidad '
                            f'{math.degrees(perpendicular_error):+.1f} deg '
                            f'sostenida (umbral '
                            f'{math.degrees(self.pf("realign_yaw_threshold")):.0f}'
                            f' deg), vuelta {realign_cycles}/'
                            f'{int(self.pf("max_realign_cycles"))}'
                        )

                        self.stop_robot()
                        self.get_logger().warn(transition_reason)

                        self.send_feedback(
                            goal_handle, state,
                            final_distance, center_error, elapsed,
                        )

                        time.sleep(period)
                        continue

                else:
                    realign_since_ns = None

            target_yaw = planner.desired_heading(
                rx, ry, mx, my, nx, ny,
                remaining,
                standoff * 2.0,
            )
            if angular_frozen:
                # En el tramo final no se persigue la orientacion de los
                # frames de borde: la base mecanum conserva yaw y corrige
                # solo la traslacion contra la referencia fijada en odom.
                target_yaw = frozen_yaw
                yaw_settled = True

            # Medicion independiente de la geometria ArUco. La distancia
            # `along` puede estar sesgada si marker_length o TF de camara no
            # estan calibrados; el eco frontal mas cercano debe limitar el
            # mando del ciclo actual, no esperar al ciclo siguiente.
            safety_observation = self.get_front_lidar_observation(
                None,
                nearest=True,
            )
            safety_front = (
                safety_observation[0]
                if safety_observation is not None else None
            )
            safety_chassis_clearance = (
                safety_observation[1]
                if safety_observation is not None else None
            )

            if safety_front is not None and safety_chassis_clearance is None:
                self.stop_robot()
                goal_handle.abort()
                result = ArucoApproach.Result()
                result.success = False
                result.status = 'TF_ERROR'
                result.message = (
                    'No se puede calcular el despeje del chasis: falta TF '
                    f'{self.get_parameter("chassis_frame").value} <- laser_frame.'
                )
                result.final_distance = safety_front
                result.final_chassis_clearance = -1.0
                self.get_logger().error(result.message)
                return result

            final_lidar_heading = None
            if (
                last_lidar_outward_normal is not None and
                last_lidar_normal_ns is not None and
                (now_ns - last_lidar_normal_ns) / 1e9 <=
                self.pf('final_lidar_normal_max_age')
            ):
                lnx, lny = last_lidar_outward_normal
                final_lidar_heading = math.atan2(-lny, -lnx)

            final_alignment_active = (
                safety_front is not None and
                safety_front - stop_distance <=
                self.pf('final_alignment_distance')
            )

            if final_alignment_active and final_lidar_heading is not None:
                # Prueba opcional: la normal LiDAR puede sustituir el rumbo
                # fijado por la ultima normal ArUco. Por defecto se conserva
                # el ArUco y LiDAR solo mide distancia y seguridad.
                target_yaw = final_lidar_heading
                if abs(normalize_angle(target_yaw - ryaw)) > self.pf(
                    'final_heading_tolerance'
                ):
                    yaw_settled = False

            # La histeresis de marcha normal puede dejar yaw_settled activo
            # hasta un umbral mas ancho que el criterio de llegada. En la
            # zona final eso crea un bloqueo: yaw_error aun no es valido,
            # pero el controlador ya no gira. La llegada debe prevalecer.
            if (
                final_alignment_active and
                abs(normalize_angle(target_yaw - ryaw)) >
                self.pf('final_heading_tolerance')
            ):
                yaw_settled = False

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
                'yaw_hysteresis': self.pf('yaw_hysteresis'),
                'stop_margin': planner.stopping_distance(
                    self.pf('min_linear_speed'),
                    self.pf('command_latency'),
                    period,
                ),
            }

            slow_final = (
                safety_front is not None and
                safety_front - stop_distance <= self.pf('final_slow_distance')
            )
            if slow_final:
                limits['max_linear'] = min(
                    limits['max_linear'], self.pf('final_max_linear_speed')
                )
                limits['max_lateral'] = min(
                    limits['max_lateral'], self.pf('final_max_lateral_speed')
                )
                limits['max_angular'] = min(
                    limits['max_angular'], self.pf('final_max_angular_speed')
                )

            # -------------------------------------------------
            # GIRAR LO MENOS POSIBLE MIENTRAS SE APROXIMA
            #
            # Es una base mecanum: el desvio lateral se corrige
            # DESPLAZANDOSE, no rotando. Y rotar aqui sale caro por tres
            # motivos que se realimentan:
            #
            #   1. La base no sabe girar despacio. Su suelo son
            #      0.37 rad/s, asi que cualquier correccion de rumbo es
            #      un tiron (medido con curva_respuesta.py).
            #   2. Ese tiron desenfoca la imagen y el detector pierde el
            #      marcador. Medido durante una aproximacion: 93
            #      mensajes sin deteccion contra 78 con ella, un 54% de
            #      perdida.
            #   3. Sin detecciones el rumbo estimado se degrada, lo que
            #      pide otra correccion. Vuelta al punto 1.
            #
            # Ademas, como la placa no acepta los tres ejes, cada ciclo
            # de giro es un ciclo que NO avanza. Con el rodeo quitado la
            # aproximacion iba a 5.4 mm/s teniendo un suelo de avance de
            # 70: el cuello de botella era este, no el camino.
            #
            # La primera fase exige dos cosas de la camara: el ArUco
            # centrado y el robot mirando hacia el marcador. Asi la base
            # no empieza a avanzar oblicua y el LiDAR no tiene que reparar
            # una mala orientacion desde el ultimo tramo.
            # La ventana de gracia protege la entrega de
            # ALIGN_PERPENDICULAR: durante align_handoff_grace_sec el
            # centrado de camara NO puede reevaluar yaw_settled. Sin
            # ella, el encare recien conseguido se deshace en el primer
            # ciclo con un giro en seco, que es justo lo que la etapa
            # venia a quitar del tramo final.
            handoff_grace = (
                align_handoff_until_ns is not None and
                now_ns < align_handoff_until_ns
            )

            if (
                detection is not None and
                remaining > self.pf('yaw_free_until') and
                not handoff_grace
            ):

                camera_yaw_error = normalize_angle(target_yaw - ryaw)
                camera_aligned = (
                    abs(center_error) <= self.pf('center_keep_margin') and
                    abs(camera_yaw_error) <= yaw_tolerance
                )
                yaw_settled = camera_aligned

            # -------------------------------------------------
            # FRENAR CON LA REGLA MAS PESIMISTA
            #
            # `remaining` sale del camino, y el camino sale de la pose
            # del marcador por camara. Un sesgo de la camara HACIA
            # ARRIBA no frena a tiempo: medido en pista, el robot se
            # planto a 0.159 m con 0.200 pedidos, y luego no podia
            # corregir porque para la geometria ya habia llegado.
            #
            # Las tres corridas del dia dieron 0.226, 0.197 y 0.159
            # sobre una tolerancia de 0.03: las dos buenas cayeron
            # dentro por poco, no por precision.
            #
            # El LiDAR mide el plano de verdad y es quien juzga la
            # llegada, asi que aqui manda la regla que diga "estas mas
            # cerca". El minimo solo puede ADELANTAR la frenada, nunca
            # retrasarla, o sea que no añade riesgo de choque.
            #
            # final_distance es del ciclo anterior; a 20 Hz y 0.18 m/s
            # eso son 9 mm. El > 0.0 filtra el centinela inicial.
            # -------------------------------------------------

            remaining_ctrl = remaining
            brake_equivalent = None

            # ¿estamos en el endgame? -- por la medida de LiDAR del
            # ciclo anterior, que es lo unico disponible aqui.
            endgame_speed = (
                final_distance > 0.0 and
                final_distance < self.pf('lidar_nearest_below')
            )

            # En el endgame el planificador no debe cerrar ni cortar el
            # mando con la tolerancia general: la llegada la decide abajo
            # el LiDAR con final_distance_tolerance.
            if endgame_speed:
                limits['distance_tolerance'] = self.pf(
                    'final_distance_tolerance'
                )

            # Cuando la camara pierde el marcador, el carrot de la ruta
            # puede quedar exactamente en la pose actual aunque el LiDAR
            # siga midiendo distancia para avanzar. En ese caso
            # holonomic_command recibe una velocidad valida, pero una
            # direccion de longitud cero y publica ruedas a cero. Durante
            # el endgame la normal LiDAR ya es la referencia de avance:
            # proyectamos un carrot virtual delante del robot para que la
            # velocidad LiDAR siga teniendo una direccion util.
            if (
                endgame_speed and
                (
                    detection is None or
                    remaining <= self.pf('distance_tolerance')
                )
            ):
                carrot_distance = max(
                    self.pf('lookahead_distance'),
                    self.pf('distance_tolerance'),
                )
                carrot_xy = (
                    rx + carrot_distance * math.cos(target_yaw),
                    ry + carrot_distance * math.sin(target_yaw),
                )

            # final_distance viene del ciclo anterior y ya es la medida
            # unificada (eco cercano en endgame, mediana lejos). El eco
            # de seguridad de ESTE ciclo solo puede hacerla mas
            # restrictiva, nunca menos.
            control_distance = final_distance
            if (
                safety_front is not None and
                (
                    control_distance <= 0.0 or
                    safety_front < control_distance -
                    self.pf('lidar_safety_obstacle_margin')
                )
            ):
                control_distance = safety_front

            if control_distance > 0.0:
                # `lidar_dt`: los ecos frontales van a ~8 Hz y el lazo a
                # 20, asi que la distancia con la que se decide puede
                # llegar hasta un periodo de LiDAR rancia. Se suma a la
                # inercia real.
                lidar_dt = 1.0 / max(1.0, self.pf('lidar_rate_hint_hz'))
                bt = planner.brake_target(
                    control_distance,
                    max(
                        0.0,
                        stop_distance - self.pf('final_braking_bias'),
                    ),
                    last_forward_speed,
                    self.pf('command_latency'),
                    period,
                    sensor_period=lidar_dt,
                    a_max=limits['accel'],
                    v_max=limits['max_linear'],
                )
                brake_equivalent = bt

                # `bt` no es la distancia fisica al marcador: es la
                # distancia equivalente de la velocidad compensada. Si se
                # compara de nuevo con la tolerancia o stop_margin fisicos,
                # el valor saturado (v_max^2 / 2a) puede quedar por debajo
                # de ambos incluso a varios metros. Eso ordenaba vx=vy=0
                # con LiDAR=3.076 m y objetivo=0.090 m. La compensacion ya
                # contiene el margen de parada; para este perfil solo se
                # detiene cuando bt <= 0.
                limits['distance_tolerance'] = 0.0
                limits['stop_margin'] = 0.0

                # Cerca del objetivo el CAMINO manda -- su ultimo tramo
                # acaba en stop_distance del marcador POR GEOMETRIA de
                # camara, asi que `remaining` (del carrot) llega a cero
                # antes de que el LiDAR de por buena la llegada. Si se
                # toma el min, el robot se planta ahi: STALLED a 0.23
                # pidiendo 0.20, medido en pista.
                #
                # En el endgame la VELOCIDAD la fija solo el LiDAR
                # (brake_target); la DIRECCION la sigue marcando el
                # carrot, que va aparte en holonomic_command. Lejos se
                # respeta el camino, que ahi si es la referencia buena.
                if endgame_speed:
                    remaining_ctrl = bt
                else:
                    remaining_ctrl = min(remaining, bt)

            vx, vy, wz, yaw_error, _planner_reached, yaw_settled = (
                planner.holonomic_command(
                    rx, ry, ryaw,
                    carrot_xy,
                    target_yaw,
                    remaining_ctrl,
                    limits,
                    yaw_settled=yaw_settled,
                )
            )
            if angular_frozen:
                wz = 0.0
                yaw_settled = True

            # Correccion lateral de precision en la zona final. El perfil
            # de frenado puede dejar vx=vy=0 cuando el LiDAR ya esta en la
            # banda objetivo, aunque el ArUco siga fuera del centro por un
            # sesgo de TF o de la pose estimada. En ese punto solo hay que
            # desplazar la base: avanzar volveria a empeorar la distancia.
            if (
                endgame_speed and
                safety_chassis_clearance is not None and
                detection is not None and
                not angular_frozen and
                abs(center_error) >
                self.pf('final_camera_center_tolerance') and
                abs(wz) < 1e-9
            ):
                vy = clamp(
                    -self.pf('final_camera_lateral_kp') * center_error,
                    -limits['max_lateral'],
                    limits['max_lateral'],
                )

                vy = planner.apply_deadband(
                    vy,
                    min(self.pf('min_lateral_speed'), limits['max_lateral']),
                )

                vx = 0.0

            # -------------------------------------------------
            # El LiDAR manda en la distancia
            #
            # `along` sale de la posicion del marcador por TF, que
            # depende de que marker_length sea correcto. El LiDAR mide
            # el plano de verdad, asi que decide la llegada y es lo que
            # se reporta. Con el tamaño del marcador mal, la geometria
            # se equivoca y esto lo salva.
            # -------------------------------------------------

            # Que espera ver el LiDAR: la geometria dice que el plano
            # del marcador esta a `along` de base_link, y el sensor va
            # laser_x por delante. Sin esta expectativa el sector no
            # puede distinguir el marcador del fondo.
            laser_to_base = self.get_laser_to_base()
            laser_x = (
                laser_to_base[0] if laser_to_base is not None else 0.0
            )
            expected_plane = along + laser_x * nx + (
                laser_to_base[1] * ny if laser_to_base is not None else 0.0
            )

            # UNA SOLA MEDIDA para frenar Y para declarar llegada.
            #
            # Antes se frenaba con el eco mas cercano del sector
            # (safety_clearance) pero se declaraba llegada con la MEDIANA
            # (front). Si el sector coge algo de fondo -- marcador sobre
            # un poste fino, superficie oblicua -- la mediana lee mas
            # lejos que el minimo: el robot frena bien pero nunca cierra
            # y salta STALLED. O al reves, y llega antes de tiempo.
            #
            # Cerca del objetivo manda el eco MAS CERCANO del sector: si
            # nos acercamos de frente y el area esta despejada, lo mas
            # cercano ES el plano del marcador; nada puede tirar de esa
            # cifra hacia delante salvo un obstaculo real, y para uno de
            # esos ya queremos parar. Lejos se usa la mediana, mas
            # estable frente al ruido puntual.
            endgame = along < self.pf('lidar_nearest_below')

            front_median_observation = self.get_front_lidar_observation(
                expected_plane
            )
            front_robust_observation = (
                self.get_front_lidar_observation(
                    expected_plane,
                    robust_nearest_mode=True,
                )
            )
            front_observation = None
            if endgame and front_robust_observation is not None:
                front_observation = front_robust_observation
            elif front_median_observation is not None:
                front_observation = front_median_observation
            elif safety_observation is not None:
                front_observation = safety_observation

            front = None
            front_chassis_clearance = None
            if front_observation is not None:
                front, front_chassis_clearance = front_observation
                # final_distance is explicitly LiDAR -> wall. The chassis
                # clearance is kept separately for collision safety.
                final_distance = front
            else:
                final_distance = along
            self._active_goal_context.update(
                state=state,
                final_distance=final_distance,
                lidar_failure=self._lidar_fail,
            )

            # La odometria/camara guia el movimiento, pero no puede
            # declarar llegada: hace falta LiDAR fresco y ArUco centrado.
            reached = False
            camera_centered = (
                detection is not None and
                not angular_frozen and
                abs(center_error) <= self.pf('final_camera_center_tolerance')
            )

            final_heading_tolerance = self.pf(
                'final_heading_tolerance'
            )

            if abs(yaw_error) <= final_heading_tolerance:
                if final_heading_since_ns is None:
                    final_heading_since_ns = now_ns
            else:
                final_heading_since_ns = None

            aligned = angular_frozen or (
                final_heading_since_ns is not None and
                (now_ns - final_heading_since_ns) / 1e9 >=
                self.pf('final_heading_settle_sec')
            )

            # Mientras hay imagen, el centro del ArUco es la medida mas
            # directa del desfase lateral real. La pose filtrada puede
            # quedar desplazada por el error de TF o por la escala del
            # marcador. En tramo ciego no hay esa medida, asi que se
            # conserva el criterio odometrico.
            centred = (
                camera_centered
                if detection is not None and not angular_frozen
                else abs(lateral) <= self.pf('lateral_tolerance')
            )

            distance_in_band = (
                front is not None and
                abs(front - stop_distance) <=
                self.pf('final_distance_tolerance')
            )
            if distance_in_band:
                if final_distance_since_ns is None:
                    final_distance_since_ns = now_ns
            else:
                final_distance_since_ns = None
            distance_settled = (
                final_distance_since_ns is not None and
                (now_ns - final_distance_since_ns) / 1e9 >=
                self.pf('final_distance_settle_sec')
            )

            odom_velocity = self.get_robot_velocity()
            linear_velocity_tolerance = self.pf(
                'final_linear_velocity_tolerance'
            )
            angular_velocity_tolerance = self.pf(
                'final_angular_velocity_tolerance'
            )
            command_slow = (
                math.hypot(vx, vy) <= linear_velocity_tolerance and
                abs(wz) <= angular_velocity_tolerance
            )
            odom_slow = (
                odom_velocity is not None and
                odom_velocity[0] <= linear_velocity_tolerance and
                odom_velocity[1] <= angular_velocity_tolerance
            )
            if command_slow and odom_slow:
                if final_velocity_since_ns is None:
                    final_velocity_since_ns = now_ns
            else:
                final_velocity_since_ns = None
            velocity_settled = (
                final_velocity_since_ns is not None and
                (now_ns - final_velocity_since_ns) / 1e9 >=
                self.pf('final_velocity_settle_sec')
            )

            if (
                distance_settled and
                velocity_settled and
                aligned and
                centred and
                # La perpendicularidad la juzga la normal LiDAR. El centro
                # de imagen es diagnostico: la camara tiene un offset
                # angular propio y no debe invalidar una pose geometrica.
                # El tramo ciego permite seguir avanzando con odometria,
                # pero exige una referencia de orientacion fresca: LiDAR si
                # esta disponible o la normal de camara/estimador en respaldo.
                (
                    final_lidar_heading is not None or
                    estimate.ready
                )
            ):
                reached = True

            # -------------------------------------------------
            # Parada de seguridad
            # -------------------------------------------------

            configured_clearance = self.pf('min_chassis_clearance')
            clearance = (
                configured_clearance
                if configured_clearance >= 0.0
                else self.pf('min_front_clearance')
            )

            operational_clearance = (
                clearance +
                self.pf('chassis_clearance_stop_margin')
            )
            if (
                front_chassis_clearance is not None and
                front_chassis_clearance < operational_clearance
            ):

                self.stop_robot()

                result = ArucoApproach.Result()

                result.success = False
                emergency = self.pf('emergency_chassis_clearance')
                hard_block = front_chassis_clearance < emergency
                result.status = 'BLOCKED' if hard_block else 'SAFE_STOP'
                result.message = (
                    f'{"Obstaculo" if hard_block else "Parada segura"}: '
                    f'despeje chasis={front_chassis_clearance:.3f} m '
                    f'(operativo {operational_clearance:.3f} m, '
                    f'minimo fisico {clearance:.3f} m), '
                    f'LiDAR-pared={front:.3f} m, '
                    f'aligned={aligned}, centred={centred}'
                )
                result.final_distance = front if front is not None else -1.0
                result.final_chassis_clearance = front_chassis_clearance

                if hard_block:
                    goal_handle.abort()
                    self.get_logger().error(result.message)
                else:
                    # SAFE_STOP finaliza el goal sin declararlo abortado.
                    # El orquestador aun verifica la pose antes del brazo.
                    goal_handle.succeed()
                    self.get_logger().warn(result.message)

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
                result_range = (
                    front if front is not None else safety_front
                )
                chassis_text = (
                    f'despeje_chasis={front_chassis_clearance:.3f} m, '
                    if front_chassis_clearance is not None else
                    'despeje_chasis=desconocido, '
                )
                result.message = (
                    f'Llegada: lidar_pared={final_distance:.3f} m, '
                    f'{chassis_text}'
                    f'rango={result_range:.3f} m, '
                    f'geometria={along:.3f} m, '
                    f'lateral={lateral:+.3f} m, '
                    f'camara={center_error:+.2f}, '
                    f'yaw={math.degrees(yaw_error):+.1f} deg, '
                    f'velocidad_estable={velocity_settled}, {elapsed:.1f} s'
                )
                result.final_distance = final_distance
                result.final_chassis_clearance = front_chassis_clearance

                self.get_logger().info(result.message)

                return result

            # -------------------------------------------------
            # Ni avanza ni llega: no colgarse callado
            #
            # Si el mando cae a cero mientras `reached` sigue False, el
            # robot ya no se va a mover solo: el camino se agoto pero el
            # juez de la llegada no da su brazo a torcer. Eso paso en
            # pista y costo el goal entero -- 150 s de timeout, 140 de
            # ellos inmovil, sin una sola linea de log.
            #
            # Un bloqueo asi casi siempre significa que las dos medidas
            # de la distancia discrepan, asi que se abortan las dos a la
            # vista. Diagnosticarlo costaba una prueba de pista; ahora
            # cuesta dos segundos.
            # -------------------------------------------------

            parado = (
                abs(vx) < 1e-6 and
                abs(vy) < 1e-6 and
                abs(wz) < 1e-6
            )

            # ¿el problema es de DISTANCIA o solo de asentar?
            #
            # Si la distancia ya esta en banda y lo unico que falta es
            # alinear o centrar, NO es un atasco -- el robot esta donde
            # tiene que estar y solo pule. Contar eso como STALLED
            # abortaba a 1 mm de la tolerancia (medido: 0.231 pidiendo
            # 0.20, tolerancia 0.03). Se le da mas margen: un
            # stall_timeout largo para el pulido fino, el corto solo si
            # de verdad no llega en distancia.
            cerca_en_distancia = (
                front is not None and
                abs(front - stop_distance) <=
                self.pf('distance_tolerance') * 2.0
            )

            if parado and not reached:
                stalled += 1
            else:
                stalled = 0

            limite_stall = self.pf('stall_timeout')
            if cerca_en_distancia:
                limite_stall = self.pf('stall_timeout') * 4.0

            if stalled >= max(1, int(limite_stall / period)):

                self.stop_robot()
                goal_handle.abort()

                result = ArucoApproach.Result()

                result.success = False
                result.status = 'STALLED'
                result.message = (
                    'Sin mando y sin llegada: el control se agoto pero '
                    'la llegada no se acepta. '
                    f'LiDAR={final_distance:.3f} m, '
                    f'geometria={along:.3f} m, '
                    f'pedido={stop_distance:.3f} m '
                    f'(tolerancia {self.pf("distance_tolerance"):.3f}). '
                    f'aligned={aligned}, centred={centred}, '
                    f'normal_lidar={final_lidar_heading is not None}, '
                    f'yaw_error={math.degrees(yaw_error):+.1f} deg, '
                    f'camara={center_error:+.3f}; '
                    f'endgame={endgame_speed}, '
                    f'control_distance={control_distance:.3f} m, '
                    f'brake_equivalent='
                    f'{brake_equivalent if brake_equivalent is not None else float("nan"):.4f} m, '
                    f'remaining={remaining:.3f} m, '
                    f'profile_remaining={remaining_ctrl:.4f} m, '
                    f'carrot_error={math.hypot(carrot_xy[0] - rx, carrot_xy[1] - ry):.4f} m, '
                    f'cmd=({vx:.3f}, {vy:.3f}, {wz:.3f}). '
                    'Si las dos distancias discrepan, revisa '
                    'marker_length, lidar_to_front_bumper_m y que el '
                    'sector frontal no este midiendo el fondo.'
                )
                result.final_distance = float(final_distance)
                result.final_chassis_clearance = front_chassis_clearance

                self.get_logger().error(result.message)

                return result

            # Para la compensacion de inercia del proximo ciclo: la
            # componente de AVANCE del mando, no el modulo (vy no
            # empuja hacia el marcador).
            last_forward_speed = vx

            self.publish_cmd(vx, vy, wz)

            if detection is None and loss_reason is None:
                loss_reason = (
                    f'sin deteccion desde hace {detection_age:.2f} s; '
                    'navegando con la pose fijada en odom'
                    if detection_age != float('inf') else 'nunca detectado'
                )

            self.log_stage_diagnostics(
                'APPROACH',
                'ENDGAME' if endgame_speed else 'CRUCERO',
                detection, reliable, detection_age, blind_travel,
                yaw_error, lateral, along,
                vx, vy, wz,
                final_distance, loss_reason, transition_reason,
            )

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
