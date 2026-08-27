from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node

from launch_ros.substitutions import (
    FindPackageShare,
)


def generate_launch_description():

    default_mission_file = (
        PathJoinSubstitution([
            FindPackageShare(
                'home_service_mission'
            ),
            'config',
            'test_mission.yaml',
        ])
    )

    mission_file_arg = (
        DeclareLaunchArgument(
            'mission_file',
            default_value=(
                default_mission_file
            ),
            description=(
                'YAML mission definition'
            )
        )
    )

    use_sim_time_arg = (
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        )
    )

    mission_manager = Node(
        package=(
            'home_service_mission'
        ),
        executable=(
            'mission_manager'
        ),
        name=(
            'home_service_mission_manager'
        ),
        output='screen',
        parameters=[
            {
                'mission_file':
                    LaunchConfiguration(
                        'mission_file'
                    ),

                'use_sim_time':
                    LaunchConfiguration(
                        'use_sim_time'
                    ),
            }
        ]
    )

    return LaunchDescription([
        mission_file_arg,
        use_sim_time_arg,
        mission_manager,
    ])
