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

        # El LiDAR no esta en el borde delantero que mide el operador.
        # En la prueba de poste leia 0.206 m con el borde a ~0.01 m, por
        # lo que el borde queda unos 0.195 m por delante del sensor.
        self.declare_parameter(
            'lidar_to_front_bumper_m',
            # 0.09, medido por el usuario directamente sobre el robot:
            # del sensor al borde delantero, sin marcador de por medio.
            # El 0.195 salio de la prueba de poste y era de otra escena.
            #
            # Las dos reconciliaciones indirectas daban 0.081 y 0.122
            # segun de que corrida se partiera, o sea que ninguna era de
            # fiar. Una medida estatica del propio robot no depende de
            # donde este ni de que este mirando.
            0.09
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
        # marker_length y lidar_to_front_bumper_m (0.195 salio de la
        # prueba de poste, otra escena; la cinta apunta a ~0.081), y
        # SOLO entonces poner esto a ~0.08. Hasta ahi, si el sector
        # midiera el fondo, el aborto por STALLED lo dice en 2 s.
        self.declare_parameter(
            'lidar_front_depth_band',
            0.0
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

        # Histeresis del giro. La tolerancia fina hace de umbral de
        # ENTRADA en asentado, y esta por el de SALIDA. Sin los dos
        # umbrales el giro castañea: la zona muerta obliga a mandar el
        # minimo en cuanto se sale de tolerancia, ese minimo se pasa de
        # largo, y al ciclo siguiente hay que corregir al otro lado.
        # Cuanto se deja alejar el marcador del centro del encuadre
        # antes de gastar un ciclo en girar. En unidades de
        # center_x_normalized, que va de -1 a +1: 0.55 deja mas de la
        # mitad del semiancho de margen y aun asi avisa mucho antes de
        # que el marcador salga.
        #
        # Subirlo = menos giros y aproximacion mas rapida, pero mas
        # riesgo de perder el marcador. Bajarlo = lo contrario.
        self.declare_parameter(
            'center_keep_margin',
            0.55
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

        # Retardo del lazo: lo que tarda un mando en surtir efecto
        # (tuberia + red + driver). Con el servidor en el portatil
        # medimos ~200 ms; pegado a los drivers en la Jetson seria
        # bastante menos. Se usa para la distancia de parada.
        self.declare_parameter(
            'command_latency',
            0.20
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

    def planner_stop_distance(self, bumper_clearance):
        """Convierte el despeje del borde en distancia desde base_link."""
        laser_to_base = self.get_laser_to_base()
        laser_x = laser_to_base[0] if laser_to_base is not None else 0.0
        return (
            bumper_clearance +
            self.pf('lidar_to_front_bumper_m') +
            laser_x
        )

    def get_front_lidar_range(self, expected=None, nearest=False):

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

        for nombre, tol, suelo, unidad in (
            ('heading_tolerance',
             math.radians(0.0) + self.pf('heading_tolerance'),
             self.pf('min_heading_speed'), 'rad'),
            ('distance_tolerance',
             self.pf('distance_tolerance'),
             self.pf('min_linear_speed'), 'm'),
            ('lateral_tolerance',
             self.pf('lateral_tolerance'),
             self.pf('min_lateral_speed'), 'm'),
        ):

            parada = planner.stopping_distance(suelo, latency, period)

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
            max_normal_jump=math.radians(
                self.pf('max_normal_jump_deg')
            ),
        )

        use_lidar = bool(
            self.get_parameter(
                'use_lidar_normal'
            ).value
        )

        state = 'SEARCHING'

        # Ciclos seguidos sin mando y sin llegada declarada. Ver el
        # bloque "Ni avanza ni llega" mas abajo.
        stalled = 0

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

            state = 'PURSUING'

            rx, ry, ryaw = robot_pose
            mx, my, nx, ny = estimate.pose

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

            recorrido_ciego = (
                math.hypot(rx - last_detection_xy[0],
                           ry - last_detection_xy[1])
                if last_detection_xy is not None else 0.0
            )

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
                    'Abortando en vez de navegar a ciegas.'
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
            path_stop_distance = self.planner_stop_distance(stop_distance)
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

            target_yaw = planner.desired_heading(
                rx, ry, mx, my, nx, ny,
                remaining,
                standoff * 2.0,
            )

            # Medicion independiente de la geometria ArUco. La distancia
            # `along` puede estar sesgada si marker_length o TF de camara no
            # estan calibrados; el eco frontal mas cercano debe limitar el
            # mando del ciclo actual, no esperar al ciclo siguiente.
            safety_front = self.get_front_lidar_range(
                None,
                nearest=True,
            )
            safety_clearance = (
                safety_front - self.pf('lidar_to_front_bumper_m')
                if safety_front is not None else None
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
                'yaw_hysteresis': self.pf('yaw_hysteresis'),
                'stop_margin': planner.stopping_distance(
                    self.pf('min_linear_speed'),
                    self.pf('command_latency'),
                    period,
                ),
            }

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
            # Asi que el criterio para girar deja de ser el error de
            # rumbo ESTIMADO y pasa a ser lo unico que de verdad
            # obliga: que el marcador se salga del encuadre. Se mide
            # directamente en la imagen (center_x_normalized), que no
            # depende de la normal ni de la pose del ArUco, asi que es
            # inmune a los saltos de las dos.
            #
            # Cerca del objetivo se devuelve el mando al rumbo, para que
            # la llegada quede perpendicular: ahi ya casi no queda
            # traslacion que perder.
            if (
                detection is not None and
                remaining > self.pf('yaw_free_until')
            ):

                if abs(center_error) < self.pf('center_keep_margin'):
                    # el marcador esta comodo en el encuadre: no gires,
                    # corrige de lado
                    yaw_settled = True

                else:
                    # se va por el borde: este ciclo es de giro puro
                    yaw_settled = False

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

            control_distance = final_distance
            if safety_clearance is not None:
                control_distance = safety_clearance

            if control_distance > 0.0:
                remaining_ctrl = min(
                    remaining,
                    control_distance - stop_distance,
                )

            vx, vy, wz, yaw_error, reached, yaw_settled = (
                planner.holonomic_command(
                    rx, ry, ryaw,
                    carrot_xy,
                    target_yaw,
                    remaining_ctrl,
                    limits,
                    yaw_settled=yaw_settled,
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

            # Que espera ver el LiDAR: la geometria dice que el plano
            # del marcador esta a `along` de base_link, y el sensor va
            # laser_x por delante. Sin esta expectativa el sector no
            # puede distinguir el marcador del fondo.
            laser_to_base = self.get_laser_to_base()
            laser_x = (
                laser_to_base[0] if laser_to_base is not None else 0.0
            )
            expected_plane = along - laser_x

            front = self.get_front_lidar_range(expected_plane)
            front_clearance = None

            if front is not None:
                front_clearance = front - self.pf('lidar_to_front_bumper_m')
                final_distance = front_clearance
            elif safety_clearance is not None:
                front_clearance = safety_clearance
                final_distance = safety_clearance
            else:
                final_distance = along

            # La odometria/camara guia el movimiento, pero no puede
            # declarar llegada: hace falta LiDAR fresco y ArUco centrado.
            reached = False
            camera_centered = (
                detection is not None and
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

            aligned = (
                final_heading_since_ns is not None and
                (now_ns - final_heading_since_ns) / 1e9 >=
                self.pf('final_heading_settle_sec')
            )

            centred = abs(lateral) <= self.pf('lateral_tolerance')

            if (
                front_clearance is not None and
                abs(front_clearance - stop_distance) <=
                self.pf('distance_tolerance') and
                aligned and
                centred and
                # La perpendicularidad la juzga la normal LiDAR. El centro
                # de imagen es diagnostico: la camara tiene un offset
                # angular propio y no debe invalidar una pose geometrica.
                (camera_centered or blind)
            ):
                reached = True

            # -------------------------------------------------
            # Parada de seguridad
            # -------------------------------------------------

            clearance = self.pf('min_front_clearance')

            if (
                front_clearance is not None and
                front_clearance < clearance
            ):

                self.stop_robot()
                goal_handle.abort()

                result = ArucoApproach.Result()

                result.success = False
                result.status = 'BLOCKED'
                result.message = (
                    f'Obstaculo a {front_clearance:.3f} m '
                    f'(minimo {clearance:.3f} m)'
                )
                result.final_distance = front_clearance

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
                result_range = (
                    front if front is not None else safety_front
                )
                result.message = (
                    f'Llegada: despeje_lidar={final_distance:.3f} m '
                    f'(rango={result_range:.3f} m), '
                    f'geometria={along:.3f} m, '
                    f'lateral={lateral:+.3f} m, '
                    f'camara={center_error:+.2f}, '
                    f'yaw={math.degrees(yaw_error):+.1f} deg, '
                    f'{elapsed:.1f} s'
                )
                result.final_distance = final_distance

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

            if parado and not reached:
                stalled += 1
            else:
                stalled = 0

            if stalled >= max(1, int(self.pf('stall_timeout') / period)):

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
                    'Si las dos distancias discrepan, revisa '
                    'marker_length, lidar_to_front_bumper_m y que el '
                    'sector frontal no este midiendo el fondo.'
                )
                result.final_distance = float(final_distance)

                self.get_logger().error(result.message)

                return result

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
