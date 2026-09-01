from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    default_params = PathJoinSubstitution([
        FindPackageShare('myagv_teleop_joy'),
        'config',
        'xbox_series.yaml',
    ])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML con los parametros del teleop de mando',
    )

    cmd_vel_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic Twist de salida hacia el myAGV',
    )

    teleop_node = Node(
        package='myagv_teleop_joy',
        executable='bluetooth_gamepad_teleop',
        name='bluetooth_gamepad_teleop',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')},
        ],
    )

    return LaunchDescription([
        params_arg,
        cmd_vel_arg,
        teleop_node,
    ])
