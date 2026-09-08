from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='true en simulacion, false en el robot real.'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='En el robot real conviene /scan_filtered '
                    '(salida del scan_sanitizer).'
    )

    lidar_approach = Node(
        package='home_service_behaviors',
        executable='aruco_lidar_approach_server',
        name='aruco_lidar_approach_server',
        output='screen',
        parameters=[{
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'),
            value_type=bool,
        ),

        'detections_topic': '/aruco/detections',
        'scan_topic': LaunchConfiguration('scan_topic'),
        'odom_topic': '/odom',
        'cmd_vel_topic': '/cmd_vel_aruco',
        'action_name': '/aruco_lidar_approach',
        'odom_frame': 'odom',

        # Incremento pequeno respecto de 0.22 rad/s: con el dwell de 0.70 s
        # aun quedan varios frames estables de la camara entre pasos.
        'search_angular_speed': 0.27,

        'lock_duration': 0.50,
        'lock_min_samples': 5,

        'kp_heading': 1.5,
        # La base gira como minimo a ~0.37 rad/s. 0.02 rad era
        # inalcanzable con la latencia de la red y provocaba sobrepaso.
        'max_heading_speed': 0.60,
        'heading_tolerance': 0.12,
        'heading_realign_threshold': 0.25,

        'kp_lateral': 0.08,
        # El minimo real de avance es ~0.07 m/s. Un techo lineal de 0.06
        # dejaba todo el rango util dentro de la zona muerta.
        'max_lateral_speed': 0.08,
        'lateral_tolerance': 0.04,
        'lateral_realign_threshold': 0.15,

        'kp_linear': 0.5,
        'max_linear_speed': 0.12,
        'distance_tolerance': 0.045,

        # Separacion medida entre camera_link y laser_frame.
        'camera_x_minus_lidar_x': 0.095,
        'lidar_sector_half_angle_deg': 6.0,

        # 0.6: la deteccion en la Nano ronda 4-9 Hz y con picos de carga
        # se salta algun frame; 0.35 abortaba con TARGET_LOST en falso.
        'detection_timeout': 0.6,
        'scan_timeout': 0.35,
        'control_rate': 20.0,
}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        scan_topic_arg,
        lidar_approach
    ])
