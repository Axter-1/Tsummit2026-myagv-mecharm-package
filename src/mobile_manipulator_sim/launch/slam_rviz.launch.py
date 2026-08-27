import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    package_share = get_package_share_directory(
        "mobile_manipulator_sim"
    )

    rviz_config = os.path.join(
        package_share,
        "rviz",
        "slam.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config
        ],
        parameters=[
            {
                "use_sim_time": True
            }
        ]
    )

    return LaunchDescription([
        rviz_node
    ])