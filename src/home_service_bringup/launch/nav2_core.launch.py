"""Servidores de Nav2 con el cableado de velocidad de ESTE robot.

Por que no se usa nav2_bringup/navigation_launch.py
===================================================
En Humble, ese launch fija estos remapeos:

    controller_server   : cmd_vel          -> cmd_vel_nav
    velocity_smoother   : cmd_vel          -> cmd_vel_nav   (entrada)
                          cmd_vel_smoothed -> cmd_vel       (salida)

Es decir, el suavizador publica DIRECTAMENTE en /cmd_vel. En este robot
/cmd_vel lo publica twist_mux (y lo consume myagv_odometry), asi que
habria dos publicadores peleandose por el mismo topic y el arbitraje
ArUco / Nav2 / teleop dejaria de funcionar.

Cableado correcto aqui:

    controller_server  --cmd_vel_raw-->  velocity_smoother
    velocity_smoother  --cmd_vel_nav-->  twist_mux  --cmd_vel-->  myAGV
    behavior_server    --cmd_vel_nav-->  twist_mux
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]

TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():

    bringup_share = get_package_share_directory('home_service_bringup')

    default_params = os.path.join(bringup_share, 'config', 'nav2_maze.yaml')
    default_bt = os.path.join(
        bringup_share, 'config', 'bt_maze_no_spin.xml'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    bt_xml = LaunchConfiguration('bt_xml')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'bt_xml',
            default_value=default_bt,
            description='Arbol de comportamiento de NavigateToPose.',
        ),
    ]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        # La salida cruda va al suavizador, no a twist_mux.
        remappings=TF_REMAPS + [('cmd_vel', 'cmd_vel_raw')],
    )

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=TF_REMAPS,
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=TF_REMAPS,
    )

    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        # Las recuperaciones (BackUp) no pasan por el suavizador.
        remappings=TF_REMAPS + [('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {'default_nav_to_pose_bt_xml': bt_xml},
        ],
        remappings=TF_REMAPS,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=TF_REMAPS + [
            ('cmd_vel', 'cmd_vel_raw'),
            ('cmd_vel_smoothed', 'cmd_vel_nav'),
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': LIFECYCLE_NODES,
        }],
    )

    return LaunchDescription(args + [
        controller,
        smoother,
        planner,
        behaviors,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
    ])
