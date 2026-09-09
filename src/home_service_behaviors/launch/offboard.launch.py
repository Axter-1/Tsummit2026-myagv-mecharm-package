#!/usr/bin/env python3
"""Pila de PROCESAMIENTO para ejecutar FUERA del robot (portatil/servidor).

Reparto (ver scripts/tsummit_offboard.sh):

    Jetson  -> drivers y seguridad:  camara CSI, LiDAR, odometria/motores,
               twist_mux, scan_sanitizer, driver del MechArm.
    AQUI    -> lo que come CPU:      detector ArUco, aproximacion,
               orquestador de agarre  (y Nav2/SLAM si se lanzan aparte).

Motivo: el FAQ oficial del T-SUMMIT lo recomienda explicitamente para
este sintoma ("image recognition... insufficient computing power ->
distributed computing"). En la Nano el detector ArUco llegaba a comerse
1.6 nucleos y la deteccion caia a 0.1 Hz.

La imagen viaja COMPRIMIDA (JPEG): medido en el robot, 246.8 Mbit/s en
crudo frente a 6.1 Mbit/s comprimida, con identica tasa de deteccion
(0/10 vs 0/10 en la misma escena; ninguna diferencia atribuible al JPEG).
Por eso 'use_compressed' va a true por defecto aqui.

Requisitos ANTES de lanzar esto (los comprueba tsummit_offboard.sh):
  * misma red y mismo ROS_DOMAIN_ID que el robot,
  * CYCLONEDDS_URI con peers unicast (multicast sobre WiFi no es fiable),
  * relojes sincronizados (chrony) o TF fallara con "message too old".
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_compressed',
            default_value='true',
            description='Consumir <image_topic>/compressed en vez de la '
                        'imagen cruda. true salvo que corras esto en el '
                        'propio robot.',
        ),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera_info'
        ),
        DeclareLaunchArgument('marker_length', default_value='0.08'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_filtered',
            description='Lo publica el scan_sanitizer, que corre en el robot.',
        ),
        # Fuera de la Nano sobra CPU: sin tope de proceso y sin reducir
        # la imagen para detectar. Son justo las dos concesiones que
        # habia que hacer en el robot.
        DeclareLaunchArgument('max_process_hz', default_value='0.0'),
        DeclareLaunchArgument('detect_scale', default_value='1.0'),
        # Ritmo fijo algo por encima de la camara a 15 fps: reduce cuanto
        # espera el detector para tomar el JPEG mas reciente sin procesar a
        # rafagas ni aumentar el trafico desde la Jetson.
        DeclareLaunchArgument('process_hz', default_value='18.0'),
        # 0 = OpenCV usa todos los nucleos para detectMarkers/imdecode.
        DeclareLaunchArgument('opencv_threads', default_value='0'),
        # La imagen anotada acompana mejor las correcciones sin competir
        # materialmente con la deteccion en el portatil.
        DeclareLaunchArgument('annotated_hz', default_value='8.0'),
        DeclareLaunchArgument('start_aruco_detector', default_value='true'),
        DeclareLaunchArgument('start_aruco_approach', default_value='true'),
        DeclareLaunchArgument('start_object_grasp', default_value='false'),
        DeclareLaunchArgument('grasp_enable_arm', default_value='true'),
        DeclareLaunchArgument('grasp_enable_approach', default_value='true'),
        DeclareLaunchArgument('table_height_mm', default_value='100'),
        DeclareLaunchArgument(
            'search_angular_speed', default_value='0.37',
            description='Suelo angular medido de la base; giro estable de busqueda.',
        ),
        # foxglove_bridge AQUI, no en la Jetson: en la Nano se comia CPU
        # serializando cada topic a CBOR, y mirar /aruco/image_annotated
        # (Image cruda que publica ESTA maquina) mandaba el frame de
        # vuelta a la Jetson solo para re-serializarlo. Local es casi
        # gratis. tsummit_offboard.sh lo activa en 'run' salvo FOXGLOVE=0.
        DeclareLaunchArgument('start_foxglove', default_value='false'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        # Zona muerta de los motores: velocidad minima que de verdad
        # mueve el robot. Calibrar (2 min, ver HANDOFF_OFFBOARD.md); por
        # debajo de esto el mando se publica y las ruedas no giran.
        DeclareLaunchArgument('min_lateral_speed', default_value='0.035'),
        # El lateral es el eje mas brusco del mecanum. El techo queda cerca
        # del avance para evitar que una correccion saque el ArUco del cuadro.
        DeclareLaunchArgument('max_lateral_speed', default_value='0.055'),
        DeclareLaunchArgument('min_linear_speed', default_value='0.07'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.08'),
        DeclareLaunchArgument('max_heading_speed', default_value='0.45'),
        DeclareLaunchArgument('use_lidar_normal', default_value='false'),
        # Margen minimo medido desde el eco hasta el borde del footprint.
        # El umbral anterior de 80 mm rechazaba lecturas de 76 mm por solo
        # 4 mm. Se amplia moderadamente a 70 mm; BLOCKED sigue siendo
        # terminal y no se permite continuar si el despeje es menor.
        DeclareLaunchArgument('min_chassis_clearance', default_value='0.07'),
        DeclareLaunchArgument(
            'chassis_clearance_stop_margin', default_value='0.015'
        ),
        DeclareLaunchArgument(
            'emergency_chassis_clearance', default_value='0.040'
        ),
        # Aproximacion con punto de encare y carrot. Ver
        # home_service_behaviors/approach_planner.py.
        DeclareLaunchArgument(
            'staging_standoff',
            default_value='0.45',
            description='Distancia del punto de encare al marcador, '
                        'sobre su normal. Desde ahi la aproximacion '
                        'final es una recta perpendicular.',
        ),
        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='0.25',
            description='Anticipacion del carrot. Mas alto = mas suave '
                        'y mas lento en reaccionar; mas bajo = mas '
                        'ceñido al camino y mas nervioso.',
        ),
        DeclareLaunchArgument(
            'corridor_radius',
            default_value='0.035',
            description='Semiancho del pasillo. Dentro de el se va '
                        'recto al objetivo sin rodear por el encare. '
                        'Tiene que caber en heading_tolerance o el '
                        'desvio se paga como un giro en seco al final.',
        ),
        DeclareLaunchArgument(
            'linear_accel',
            default_value='0.25',
            description='Frenada del perfil trapezoidal, v=sqrt(2*a*d).',
        ),
        DeclareLaunchArgument(
            'final_braking_bias',
            default_value='0.055',
            description='Compensa el avance que queda por latencia y suelo '
                        'de velocidad; no modifica la distancia reportada.',
        ),
        # El detector y el servidor de aproximacion corren en esta maquina.
        DeclareLaunchArgument(
            'lidar_to_front_bumper_m',
            default_value='0.080',
            description='Bumper -> centro de giro del LiDAR. Medido de '
                        'punta a punta con measure_front_offset.py '
                        '(rango 0.3803 - cinta 0.300), y corroborado '
                        'por otros dos caminos dentro de 1.5 mm. Alias '
                        'legado para la geometria del pasillo; la '
                        'seguridad usa TF laser->base_link y footprint.',
        ),
        DeclareLaunchArgument(
            'blind_endgame_distance',
            default_value='0.35',
            description='Por debajo de esta distancia se deja de exigir '
                        'ver el marcador: a 0.29 m un ArUco de 8 cm ya '
                        'no cabe en el encuadre. Se navega con el '
                        'marcador fijado en odom y el LiDAR midiendo.',
        ),
        DeclareLaunchArgument(
            'max_blind_travel',
            default_value='0.25',
            description='Metros que se admite recorrer sin ver el '
                        'marcador. En metros y no en segundos porque la '
                        'deriva crece con la distancia, no con la '
                        'espera.',
        ),
        DeclareLaunchArgument(
            'lidar_front_depth_band',
            default_value='0.0',
            description='Descarta ecos del sector frontal mas lejanos '
                        'que el plano esperado. 0.0 = apagada. NO '
                        'encender hasta que marker_length sea correcto: '
                        'la expectativa sale de la camara, y con la '
                        'camara mal escalada rechaza el eco bueno.',
        ),
        DeclareLaunchArgument('final_slow_distance', default_value='0.30'),
        DeclareLaunchArgument('final_max_linear_speed', default_value='0.07'),
        DeclareLaunchArgument('final_max_lateral_speed', default_value='0.035'),
        DeclareLaunchArgument('final_max_angular_speed', default_value='0.37'),
        DeclareLaunchArgument('final_distance_tolerance', default_value='0.020'),
        DeclareLaunchArgument(
            'final_linear_velocity_tolerance', default_value='0.015'
        ),
        DeclareLaunchArgument(
            'final_angular_velocity_tolerance', default_value='0.03'
        ),
        DeclareLaunchArgument('command_latency', default_value='0.27'),
        DeclareLaunchArgument('lidar_rate_hint_hz', default_value='8.0'),

        # ---- ALIGN_PERPENDICULAR ----
        # Etapa previa a la aproximacion: encararse al PLANO del
        # marcador y ponerse sobre su eje normal antes de avanzar.
        # align_enabled:=false devuelve el comportamiento anterior
        # (SEARCHING -> APPROACH directo), que es la comparacion a
        # hacer en pista.
        DeclareLaunchArgument(
            'align_enabled', default_value='true',
            description='Activa la etapa ALIGN_PERPENDICULAR previa a '
                        'APPROACH. false = comportamiento anterior.',
        ),
        DeclareLaunchArgument(
            'align_yaw_tolerance', default_value='0.13',
            description='Tolerancia de PERPENDICULARIDAD en rad. No es '
                        'el centrado en imagen. Suelo alcanzable: '
                        'min_heading_speed * (command_latency + '
                        'periodo) = 0.37 * 0.32 = 0.118 rad.',
        ),
        DeclareLaunchArgument('align_yaw_hysteresis', default_value='1.6'),
        DeclareLaunchArgument(
            'align_lateral_tolerance', default_value='0.035',
            description='Tolerancia de centrado sobre el eje normal, m. '
                        'DEBE ser <= lateral_tolerance (0.04), que es lo '
                        'que exige la llegada: si es mas ancha, la '
                        'alineacion entrega poses que la llegada no '
                        'acepta y se acaba en SAFE_STOP.',
        ),
        DeclareLaunchArgument(
            'align_lateral_hysteresis', default_value='1.6'
        ),
        DeclareLaunchArgument(
            'align_regulate_distance', default_value='true',
            description='La etapa coloca tambien en el punto de encare. '
                        'false = solo perpendicularidad y centrado; la '
                        'separacion la deja entera a APPROACH.',
        ),
        DeclareLaunchArgument(
            'align_standoff_tolerance', default_value='0.10'
        ),
        DeclareLaunchArgument('align_kp_angular', default_value='1.2'),
        DeclareLaunchArgument('align_kp_linear', default_value='0.6'),
        DeclareLaunchArgument('align_kp_lateral', default_value='0.9'),
        DeclareLaunchArgument(
            'align_max_angular_speed', default_value='0.45'
        ),
        DeclareLaunchArgument(
            'align_max_linear_speed', default_value='0.09'
        ),
        DeclareLaunchArgument(
            'align_max_lateral_speed', default_value='0.10'
        ),
        DeclareLaunchArgument('align_settle_sec', default_value='0.35'),
        DeclareLaunchArgument('align_timeout_sec', default_value='25.0'),
        DeclareLaunchArgument(
            'align_handoff_grace_sec', default_value='1.5',
            description='Ventana tras la alineacion en la que APPROACH '
                        'no puede rehacer el rumbo por centrado de '
                        'camara.',
        ),
        DeclareLaunchArgument(
            'align_min_distance', default_value='0.30',
            description='Por debajo de esta separacion no se alinea: el '
                        'marcador ya no cabe en el encuadre y el '
                        'endgame de APPROACH tiene criterios mas finos.',
        ),
        DeclareLaunchArgument(
            'align_max_pose_age', default_value='2.0',
            description='Edad maxima de la ultima pose fiable para '
                        'seguir corrigiendo sin ver el marcador.',
        ),
        DeclareLaunchArgument(
            'align_max_blind_travel', default_value='0.15',
            description='Metros que se admite recorrer sin vision '
                        'durante la alineacion. En metros y no en '
                        'segundos: la deriva crece con la distancia.',
        ),
        DeclareLaunchArgument(
            'align_min_quality', default_value='0.5',
            description='Calidad minima (0-1) de la estimacion para '
                        'fiarse de ella a ciegas. Reciente no es lo '
                        'mismo que fiable.',
        ),
        DeclareLaunchArgument(
            'align_recovery_angular_speed', default_value='0.40'
        ),
        DeclareLaunchArgument(
            'align_recovery_timeout_sec', default_value='6.0'
        ),
        DeclareLaunchArgument('align_max_attempts', default_value='3'),
        DeclareLaunchArgument(
            'realign_yaw_threshold', default_value='0.35',
            description='Error de perpendicularidad que devuelve a '
                        'ALIGN_PERPENDICULAR desde APPROACH. Debe '
                        'quedar por encima de align_yaw_tolerance * '
                        'align_yaw_hysteresis o las etapas hacen '
                        'pinpon.',
        ),
        DeclareLaunchArgument(
            'realign_persist_sec', default_value='0.6'
        ),
        DeclareLaunchArgument('max_realign_cycles', default_value='2'),
        DeclareLaunchArgument('align_log_period', default_value='0.5'),
        DeclareLaunchArgument(
            'kp_lateral_odom', default_value='0.9',
            description='Ganancia del recentrado lateral en el tramo '
                        'ciego, contra el desvio odometrico respecto al '
                        'eje del pasillo (metros). Sin esto nada corrige '
                        'el lateral cuando el ArUco ya no cabe en el '
                        'encuadre.',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    aruco_detector = Node(
        package='home_service_perception',
        executable='aruco_detector_node',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_compressed': LaunchConfiguration('use_compressed'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'marker_length': LaunchConfiguration('marker_length'),
            'max_process_hz': LaunchConfiguration('max_process_hz'),
            'detect_scale': LaunchConfiguration('detect_scale'),
            'process_hz': ParameterValue(
                LaunchConfiguration('process_hz'), value_type=float),
            'opencv_threads': ParameterValue(
                LaunchConfiguration('opencv_threads'), value_type=int),
            'annotated_hz': ParameterValue(
                LaunchConfiguration('annotated_hz'), value_type=float),
            'equalize_hist': True,
            'publish_tf': True,
        }],
        condition=IfCondition(LaunchConfiguration('start_aruco_detector')),
    )

    aruco_approach = Node(
        package='home_service_behaviors',
        executable='aruco_lidar_approach_server',
        name='aruco_lidar_approach_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'detections_topic': '/aruco/detections',
            'scan_topic': LaunchConfiguration('scan_topic'),
            'odom_topic': '/odom',
            'cmd_vel_topic': '/cmd_vel_aruco',
            'action_name': '/aruco_lidar_approach',
            'odom_frame': 'odom',
            # Con la red de por medio hay que ser un poco mas tolerante
            # que en local, pero NO tanto como para no notar una caida:
            # el watchdog de myagv_odometry (300 ms) es la red de
            # seguridad real si el enlace se cae.
            'detection_timeout': 0.8,
            'scan_timeout': 0.8,
            'control_rate': 20.0,
            'search_angular_speed': ParameterValue(
                LaunchConfiguration('search_angular_speed'), value_type=float
            ),
            'min_lateral_speed': ParameterValue(
                LaunchConfiguration('min_lateral_speed'),
                value_type=float,
            ),
            'max_lateral_speed': ParameterValue(
                LaunchConfiguration('max_lateral_speed'),
                value_type=float,
            ),
            'min_linear_speed': ParameterValue(
                LaunchConfiguration('min_linear_speed'),
                value_type=float,
            ),
            'max_linear_speed': ParameterValue(
                LaunchConfiguration('max_linear_speed'),
                value_type=float,
            ),
            'max_heading_speed': ParameterValue(
                LaunchConfiguration('max_heading_speed'),
                value_type=float,
            ),
            'use_lidar_normal': ParameterValue(
                LaunchConfiguration('use_lidar_normal'),
                value_type=bool,
            ),
            'staging_standoff': ParameterValue(
                LaunchConfiguration('staging_standoff'),
                value_type=float,
            ),
            'lookahead_distance': ParameterValue(
                LaunchConfiguration('lookahead_distance'),
                value_type=float,
            ),
            'corridor_radius': ParameterValue(
                LaunchConfiguration('corridor_radius'),
                value_type=float,
            ),
            'linear_accel': ParameterValue(
                LaunchConfiguration('linear_accel'),
                value_type=float,
            ),
            'final_braking_bias': ParameterValue(
                LaunchConfiguration('final_braking_bias'),
                value_type=float,
            ),
            'lidar_to_front_bumper_m': ParameterValue(
                LaunchConfiguration('lidar_to_front_bumper_m'),
                value_type=float,
            ),
            'lidar_front_depth_band': ParameterValue(
                LaunchConfiguration('lidar_front_depth_band'),
                value_type=float,
            ),
            'min_chassis_clearance': ParameterValue(
                LaunchConfiguration('min_chassis_clearance'),
                value_type=float,
            ),
            'chassis_clearance_stop_margin': ParameterValue(
                LaunchConfiguration('chassis_clearance_stop_margin'),
                value_type=float,
            ),
            'emergency_chassis_clearance': ParameterValue(
                LaunchConfiguration('emergency_chassis_clearance'),
                value_type=float,
            ),
            'final_slow_distance': ParameterValue(
                LaunchConfiguration('final_slow_distance'), value_type=float
            ),
            'final_max_linear_speed': ParameterValue(
                LaunchConfiguration('final_max_linear_speed'), value_type=float
            ),
            'final_max_lateral_speed': ParameterValue(
                LaunchConfiguration('final_max_lateral_speed'), value_type=float
            ),
            'final_max_angular_speed': ParameterValue(
                LaunchConfiguration('final_max_angular_speed'), value_type=float
            ),
            'final_distance_tolerance': ParameterValue(
                LaunchConfiguration('final_distance_tolerance'), value_type=float
            ),
            'final_linear_velocity_tolerance': ParameterValue(
                LaunchConfiguration('final_linear_velocity_tolerance'),
                value_type=float,
            ),
            'final_angular_velocity_tolerance': ParameterValue(
                LaunchConfiguration('final_angular_velocity_tolerance'),
                value_type=float,
            ),
            'command_latency': ParameterValue(
                LaunchConfiguration('command_latency'), value_type=float
            ),
            'lidar_rate_hint_hz': ParameterValue(
                LaunchConfiguration('lidar_rate_hint_hz'), value_type=float
            ),
            'blind_endgame_distance': ParameterValue(
                LaunchConfiguration('blind_endgame_distance'),
                value_type=float,
            ),
            'max_blind_travel': ParameterValue(
                LaunchConfiguration('max_blind_travel'),
                value_type=float,
            ),
            # ---- ALIGN_PERPENDICULAR ----
            'align_enabled': ParameterValue(
                LaunchConfiguration('align_enabled'), value_type=bool
            ),
            'align_regulate_distance': ParameterValue(
                LaunchConfiguration('align_regulate_distance'),
                value_type=bool,
            ),
            'align_max_attempts': ParameterValue(
                LaunchConfiguration('align_max_attempts'), value_type=int
            ),
            'max_realign_cycles': ParameterValue(
                LaunchConfiguration('max_realign_cycles'), value_type=int
            ),
            'align_yaw_tolerance': ParameterValue(
                LaunchConfiguration('align_yaw_tolerance'), value_type=float
            ),
            'align_yaw_hysteresis': ParameterValue(
                LaunchConfiguration('align_yaw_hysteresis'), value_type=float
            ),
            'align_lateral_tolerance': ParameterValue(
                LaunchConfiguration('align_lateral_tolerance'), value_type=float
            ),
            'align_lateral_hysteresis': ParameterValue(
                LaunchConfiguration('align_lateral_hysteresis'), value_type=float
            ),
            'align_standoff_tolerance': ParameterValue(
                LaunchConfiguration('align_standoff_tolerance'), value_type=float
            ),
            'align_kp_angular': ParameterValue(
                LaunchConfiguration('align_kp_angular'), value_type=float
            ),
            'align_kp_linear': ParameterValue(
                LaunchConfiguration('align_kp_linear'), value_type=float
            ),
            'align_kp_lateral': ParameterValue(
                LaunchConfiguration('align_kp_lateral'), value_type=float
            ),
            'align_max_angular_speed': ParameterValue(
                LaunchConfiguration('align_max_angular_speed'), value_type=float
            ),
            'align_max_linear_speed': ParameterValue(
                LaunchConfiguration('align_max_linear_speed'), value_type=float
            ),
            'align_max_lateral_speed': ParameterValue(
                LaunchConfiguration('align_max_lateral_speed'), value_type=float
            ),
            'align_settle_sec': ParameterValue(
                LaunchConfiguration('align_settle_sec'), value_type=float
            ),
            'align_timeout_sec': ParameterValue(
                LaunchConfiguration('align_timeout_sec'), value_type=float
            ),
            'align_handoff_grace_sec': ParameterValue(
                LaunchConfiguration('align_handoff_grace_sec'), value_type=float
            ),
            'align_min_distance': ParameterValue(
                LaunchConfiguration('align_min_distance'), value_type=float
            ),
            'align_max_pose_age': ParameterValue(
                LaunchConfiguration('align_max_pose_age'), value_type=float
            ),
            'align_max_blind_travel': ParameterValue(
                LaunchConfiguration('align_max_blind_travel'), value_type=float
            ),
            'align_min_quality': ParameterValue(
                LaunchConfiguration('align_min_quality'), value_type=float
            ),
            'align_recovery_angular_speed': ParameterValue(
                LaunchConfiguration('align_recovery_angular_speed'), value_type=float
            ),
            'align_recovery_timeout_sec': ParameterValue(
                LaunchConfiguration('align_recovery_timeout_sec'), value_type=float
            ),
            'realign_yaw_threshold': ParameterValue(
                LaunchConfiguration('realign_yaw_threshold'), value_type=float
            ),
            'realign_persist_sec': ParameterValue(
                LaunchConfiguration('realign_persist_sec'), value_type=float
            ),
            'align_log_period': ParameterValue(
                LaunchConfiguration('align_log_period'), value_type=float
            ),
            'kp_lateral_odom': ParameterValue(
                LaunchConfiguration('kp_lateral_odom'), value_type=float
            ),
        }],
        condition=IfCondition(LaunchConfiguration('start_aruco_approach')),
    )

    object_grasp = Node(
        package='home_service_behaviors',
        executable='object_grasp_server',
        name='object_grasp_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'detections_topic': '/aruco/detections',
            'scan_topic': LaunchConfiguration('scan_topic'),
            'enable_arm': LaunchConfiguration('grasp_enable_arm'),
            'enable_approach': LaunchConfiguration('grasp_enable_approach'),
            'table_height_mm': ParameterValue(
                LaunchConfiguration('table_height_mm'), value_type=int
            ),
        }],
        condition=IfCondition(LaunchConfiguration('start_object_grasp')),
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': ParameterValue(
                LaunchConfiguration('foxglove_port'),
                value_type=int,
            ),
            'address': '0.0.0.0',
            'use_compression': False,
        }],
        condition=IfCondition(LaunchConfiguration('start_foxglove')),
    )

    return LaunchDescription(
        args + [aruco_detector, aruco_approach, object_grasp, foxglove]
    )
