"""Reto 4 — LABERINTO. Pila completa de navegacion autonoma.

    scan_sanitizer   /scan -> /scan_filtered   (mata las paredes fantasma)
            |
    slam_toolbox     map <- /scan_filtered     (mapa en vivo + map->odom)
            |
    Nav2 (nav2_core) planifica y controla      (holonomico, sin girar)
            |
    maze_runner      START -> FINISH + anti-bloqueo

Modo por defecto: SLAM en vivo. Con ``slam:=false map:=<ruta.yaml>`` se
usa AMCL contra un mapa guardado (parametros ya corregidos en
nav2_maze.yaml).

Ejemplos
--------
    # Todo automatico, SLAM en vivo
    ros2 launch home_service_bringup maze.launch.py

    # Solo la pila, el objetivo se lanza a mano desde RViz o servicio
    ros2 launch home_service_bringup maze.launch.py auto_start:=false

    # Diagnostico de los sectores ciegos del laser
    ros2 launch home_service_bringup maze.launch.py \\
        report_blind_sectors:=true run_maze_runner:=false
"""

import os

from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from nav2_common.launch import RewrittenYaml


def typed(name, value_type):
    """LaunchConfiguration convertida al tipo que declara el nodo.

    Sin esto, un argumento de launch llega como cadena y el nodo lo
    rechaza por incompatibilidad de tipos.
    """
    return ParameterValue(
        LaunchConfiguration(name), value_type=value_type
    )


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
    twist_mux_params = os.path.join(
        bringup_share, 'config', 'twist_mux_real.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    scan_in = LaunchConfiguration('scan_topic')
    scan_out = LaunchConfiguration('scan_filtered_topic')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='true = slam_toolbox en vivo; '
                        'false = AMCL sobre "map".',
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Mapa .yaml (solo con slam:=false).',
        ),
        DeclareLaunchArgument('params_file', default_value=default_nav_params),
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_slam_params
        ),
        DeclareLaunchArgument('bt_xml', default_value=default_bt),
        DeclareLaunchArgument('autostart', default_value='true'),

        # --- Saneador del laser ---
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'scan_filtered_topic', default_value='/scan_filtered'
        ),
        DeclareLaunchArgument(
            'blind_sectors_deg',
            default_value='[-50.0, 50.0]',
            description='Sectores ocluidos por el propio robot, en '
                        'grados y por pares. Coincide con el '
                        'ignore_array del driver del X2L.',
        ),
        DeclareLaunchArgument(
            'report_blind_sectors',
            default_value='false',
            description='Publica en el log que angulos no devuelven '
                        'nunca eco (para calibrar blind_sectors_deg).',
        ),

        # --- Ejecutor del reto ---
        DeclareLaunchArgument('run_maze_runner', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('start_delay_sec', default_value='8.0'),
        DeclareLaunchArgument('tuck_arm', default_value='true'),
        # FINISH relativo a START. Con SLAM, el origen de "map" es la
        # pose inicial del robot: START = (0, 0).
        # Del plano del laberinto (4.5 x 3.0 m): START arriba-izquierda,
        # META abajo-derecha  ->  dx = +3.7 m, dy = -2.2 m.
        # VERIFICA estos valores sobre la pista real antes de competir.
        DeclareLaunchArgument('goal_x', default_value='3.7'),
        DeclareLaunchArgument('goal_y', default_value='-2.2'),
        DeclareLaunchArgument('goal_yaw_deg', default_value='0.0'),

        DeclareLaunchArgument(
            'start_twist_mux',
            default_value='false',
            description='true solo si NO esta corriendo ya '
                        'home_service_bringup robot.launch.py.',
        ),
    ]

    # -----------------------------------------------------------------
    # 1. Saneador del LaserScan
    # -----------------------------------------------------------------
    scan_sanitizer = Node(
        package='home_service_navigation',
        executable='scan_sanitizer_node',
        name='scan_sanitizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': scan_in,
            'output_topic': scan_out,
            'range_min': 0.16,
            'range_max': 5.0,
            'blind_sectors_deg': typed(
                'blind_sectors_deg', List[float]
            ),
            'zeros_to_inf': True,
            'speckle_filter': True,
            'speckle_window': 2,
            'speckle_threshold': 0.12,
            'report_blind_sectors': typed(
                'report_blind_sectors', bool
            ),
        }],
    )

    # -----------------------------------------------------------------
    # 2a. SLAM en vivo (modo por defecto)
    # -----------------------------------------------------------------
    slam_params = RewrittenYaml(
        source_file=LaunchConfiguration('slam_params_file'),
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'scan_topic': scan_out,
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
    # 2b. AMCL + mapa guardado (alternativa)
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
    # 4. twist_mux (solo si no lo lanzo ya robot.launch.py)
    # -----------------------------------------------------------------
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel')],
        condition=IfCondition(LaunchConfiguration('start_twist_mux')),
    )

    # -----------------------------------------------------------------
    # 5. Ejecutor del reto
    # -----------------------------------------------------------------
    maze_runner = Node(
        package='home_service_navigation',
        executable='maze_runner_node',
        name='maze_runner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'goal_x': typed('goal_x', float),
            'goal_y': typed('goal_y', float),
            'goal_yaw_deg': typed('goal_yaw_deg', float),
            'goal_frame': 'map',
            'auto_start': typed('auto_start', bool),
            'start_delay_sec': typed('start_delay_sec', float),
            'tuck_arm': typed('tuck_arm', bool),
            'tuck_pose': 'home',
            'stuck_window_sec': 6.0,
            'stuck_min_progress_m': 0.05,
            'max_recoveries': 8,
            'periodic_clear_sec': 0.0,
        }],
        condition=IfCondition(LaunchConfiguration('run_maze_runner')),
    )

    return LaunchDescription(args + [
        scan_sanitizer,
        slam_toolbox,
        map_server,
        amcl,
        lifecycle_localization,
        nav2,
        twist_mux,
        maze_runner,
    ])
