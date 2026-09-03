"""Retos 1 (Clasificacion) y 2 (Kitting) — pila completa.

    robot.launch.py    camara CSI + scan_sanitizer + detector ArUco
                       + aproximacion ArUco/LiDAR + brazo + twist_mux
            |
    slam_toolbox       mapa en vivo + map->odom
            |
    Nav2 (nav2_core)   planifica y controla (holonomico, sin girar)
            |
    mission_manager    ejecuta el YAML de la mision

Secuencia obligatoria de ambos retos:
    START -> Verde ArUco 0 -> ArUco 2 -> Azul ArUco 1 -> ArUco 3 -> FINISH

Ejemplos
--------
    # Reto 1 completo
    ros2 launch home_service_bringup classification.launch.py

    # Reto 2 (Kitting): misma secuencia con la logica "Pieza Omitida"
    ros2 launch home_service_bringup classification.launch.py \\
        mission:=reto2_kitting.yaml

    # Solo la pila, sin ejecutar la mision (para probar a mano)
    ros2 launch home_service_bringup classification.launch.py \\
        run_mission:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_share = get_package_share_directory('home_service_bringup')

    default_nav_params = os.path.join(
        bringup_share, 'config', 'nav2_maze.yaml'
    )
    default_slam_params = os.path.join(
        bringup_share, 'config', 'slam_toolbox_maze.yaml'
    )
    default_bt = os.path.join(
        bringup_share, 'config', 'bt_maze_no_spin.xml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Mapa .yaml (solo con slam:=false).',
        ),
        # nav2_maze.yaml esta afinado para pasillos estrechos y para la
        # geometria real del myAGV: sirve igual para los pasillos de
        # 0.50 / 1.00 m de los retos 1 y 2.
        DeclareLaunchArgument('params_file', default_value=default_nav_params),
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_slam_params
        ),
        DeclareLaunchArgument('bt_xml', default_value=default_bt),
        DeclareLaunchArgument('autostart', default_value='true'),

        # --- Mision ---
        DeclareLaunchArgument('run_mission', default_value='true'),
        DeclareLaunchArgument(
            'mission',
            default_value='reto1_clasificacion.yaml',
            description='Nombre del YAML dentro de '
                        'home_service_mission/config/.',
        ),
        DeclareLaunchArgument(
            'mission_delay_sec',
            default_value='15.0',
            description='Espera antes de arrancar la mision, para que '
                        'Nav2 y el SLAM se estabilicen.',
        ),

        # --- Hardware ---
        DeclareLaunchArgument('marker_length', default_value='0.08'),
        DeclareLaunchArgument('camera_source', default_value='nvargus'),
        DeclareLaunchArgument('arm_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument(
            'blind_sectors_deg', default_value='[-50.0, 50.0]'
        ),
    ]

    # -----------------------------------------------------------------
    # 1. Percepcion + brazo + twist_mux
    # -----------------------------------------------------------------
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'camera_source': LaunchConfiguration('camera_source'),
            'marker_length': LaunchConfiguration('marker_length'),
            'arm_port': LaunchConfiguration('arm_port'),
            'blind_sectors_deg': LaunchConfiguration('blind_sectors_deg'),
            'start_scan_sanitizer': 'true',
            'start_twist_mux': 'true',
        }.items(),
    )

    # -----------------------------------------------------------------
    # 2a. SLAM en vivo
    # -----------------------------------------------------------------
    slam_params = RewrittenYaml(
        source_file=LaunchConfiguration('slam_params_file'),
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'scan_topic': '/scan_filtered',
        },
        convert_types=True,
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
        condition=IfCondition(slam),
    )

    # -----------------------------------------------------------------
    # 2b. AMCL + mapa guardado
    # -----------------------------------------------------------------
    localization_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': LaunchConfiguration('map'),
        },
        convert_types=True,
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[localization_params],
        condition=UnlessCondition(slam),
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[localization_params],
        condition=UnlessCondition(slam),
    )

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['map_server', 'amcl'],
        }],
        condition=UnlessCondition(slam),
    )

    # -----------------------------------------------------------------
    # 3. Nav2
    # -----------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'nav2_core.launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart'),
            'bt_xml': LaunchConfiguration('bt_xml'),
        }.items(),
    )

    # -----------------------------------------------------------------
    # 4. Mision (retrasada para que la pila se estabilice)
    # -----------------------------------------------------------------
    mission_file = PathJoinSubstitution([
        FindPackageShare('home_service_mission'),
        'config',
        LaunchConfiguration('mission'),
    ])

    mission = TimerAction(
        period=LaunchConfiguration('mission_delay_sec'),
        actions=[
            Node(
                package='home_service_mission',
                executable='mission_manager',
                name='home_service_mission_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'mission_file': mission_file,
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('run_mission')),
    )

    return LaunchDescription(args + [
        robot,
        slam_toolbox,
        map_server,
        amcl,
        lifecycle_localization,
        nav2,
        mission,
    ])
