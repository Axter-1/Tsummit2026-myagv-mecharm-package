"""Nav2 con mapa guardado + AMCL, para el ROBOT REAL.

Usa el mismo cableado de velocidad que el resto del proyecto:

    Nav2 -> /cmd_vel_nav -> twist_mux -> /cmd_vel -> myagv_odometry

Para el Reto 4 (Laberinto) usa mejor ``maze.launch.py``, que ademas
levanta SLAM en vivo y el saneador del laser.

Uso:
    ros2 launch home_service_bringup nav2.launch.py \\
        map:=/workspace/maps/home_service_challenge_myagv.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_share = get_package_share_directory('home_service_bringup')

    default_params = os.path.join(
        bringup_share, 'config', 'nav2_real.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    args = [
        DeclareLaunchArgument(
            'map', description='Ruta al .yaml del mapa.'
        ),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
    ]

    localization_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': LaunchConfiguration('map'),
        },
        convert_types=True,
    )

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[localization_params],
        remappings=tf_remaps,
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[localization_params],
        remappings=tf_remaps,
    )

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'nav2_core.launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items(),
    )

    return LaunchDescription(args + [
        map_server,
        amcl,
        lifecycle_localization,
        nav2,
    ])
