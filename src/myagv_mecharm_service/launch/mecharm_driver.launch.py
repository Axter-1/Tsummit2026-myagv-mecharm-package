import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory("myagv_mecharm_service"),
        "config",
        "mecharm.yaml",
    )

    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyACM0"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false"
    )

    node = Node(
        package="myagv_mecharm_service",
        executable="mecharm_driver_node",
        name="mecharm_driver_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "port": LaunchConfiguration("port"),
            },
        ],
    )

    return LaunchDescription([
        port_arg,
        use_sim_time_arg,
        node,
    ])
