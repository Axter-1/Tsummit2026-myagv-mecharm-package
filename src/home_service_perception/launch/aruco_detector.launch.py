from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )
    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed', default_value='true'
    )
    marker_length_arg = DeclareLaunchArgument(
        'marker_length', default_value='0.08'
    )

    aruco_detector = Node(
        package='home_service_perception',
        executable='aruco_detector_node',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool
            ),
            'use_compressed': ParameterValue(
                LaunchConfiguration('use_compressed'), value_type=bool
            ),
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'marker_length': ParameterValue(
                LaunchConfiguration('marker_length'), value_type=float
            ),
            'equalize_hist': True,
            'publish_tf': True,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_compressed_arg,
        marker_length_arg,
        aruco_detector,
    ])
