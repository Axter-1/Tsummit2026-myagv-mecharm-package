import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    default_calib = os.path.join(
        get_package_share_directory("myagv_camera"),
        "config",
        "csi_camera_960x540.yaml",
    )

    source_arg = DeclareLaunchArgument(
        "source",
        default_value="nvargus",
        description="nvargus | v4l2 | custom",
    )

    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="camera",
        description="Prefijo de los topics (<camera_name>/image_raw).",
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="camera_link",
    )

    camera_info_url_arg = DeclareLaunchArgument(
        "camera_info_url",
        default_value="file://" + default_calib,
    )

    flip_method_arg = DeclareLaunchArgument(
        "flip_method",
        default_value="0",
        description="Rotacion/espejo por hardware (0-7).",
    )

    output_width_arg = DeclareLaunchArgument(
        "output_width", default_value="960"
    )
    output_height_arg = DeclareLaunchArgument(
        "output_height", default_value="540"
    )
    framerate_arg = DeclareLaunchArgument(
        "framerate", default_value="21"
    )
    device_index_arg = DeclareLaunchArgument(
        "device_index", default_value="0"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false"
    )

    node = Node(
        package="myagv_camera",
        executable="csi_camera_node",
        name="csi_camera_node",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "source": LaunchConfiguration("source"),
            "camera_name": LaunchConfiguration("camera_name"),
            "frame_id": LaunchConfiguration("frame_id"),
            "camera_info_url": LaunchConfiguration("camera_info_url"),
            "flip_method": LaunchConfiguration("flip_method"),
            "output_width": LaunchConfiguration("output_width"),
            "output_height": LaunchConfiguration("output_height"),
            "framerate": LaunchConfiguration("framerate"),
            "device_index": LaunchConfiguration("device_index"),
        }],
    )

    return LaunchDescription([
        source_arg,
        camera_name_arg,
        frame_id_arg,
        camera_info_url_arg,
        flip_method_arg,
        output_width_arg,
        output_height_arg,
        framerate_arg,
        device_index_arg,
        use_sim_time_arg,
        node,
    ])
