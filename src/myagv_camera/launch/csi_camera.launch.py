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
        # El modulo CSI del myAGV va montado boca abajo -> 2 (rot 180).
        # No es solo estetico: sin girar, 'center_x_normalized' de las
        # detecciones sale con el signo cambiado y la aproximacion
        # lateral steerea hacia el lado contrario.
        default_value="2",
        description="Rotacion/espejo por hardware nvvidconv "
                    "(0=nada, 2=rot180, 4=espejo-h, 6=espejo-v).",
    )

    # 960x540: a 640x360 un ArUco de 8 cm a ~1 m solo mide ~48 px de
    # lado (6 px/celda para un 6x6) y NO se detecta. La carga de CPU del
    # detector se controla por otra via: aruco_detector tiene un tope de
    # proceso (max_process_hz, 6 Hz) y run_robot_routine.sh lo fija con
    # taskset fuera del nucleo del laser.
    output_width_arg = DeclareLaunchArgument(
        "output_width", default_value="960"
    )
    output_height_arg = DeclareLaunchArgument(
        "output_height", default_value="540"
    )
    # Modo de sensor ligero (1280x720) en vez de 3264x2464: menos ISP.
    capture_width_arg = DeclareLaunchArgument(
        "capture_width", default_value="1280"
    )
    capture_height_arg = DeclareLaunchArgument(
        "capture_height", default_value="720"
    )
    framerate_arg = DeclareLaunchArgument(
        "framerate", default_value="21"
    )
    publish_raw_arg = DeclareLaunchArgument(
        "publish_raw", default_value="true"
    )
    publish_compressed_arg = DeclareLaunchArgument(
        "publish_compressed", default_value="true"
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
            "capture_width": LaunchConfiguration("capture_width"),
            "capture_height": LaunchConfiguration("capture_height"),
            "framerate": LaunchConfiguration("framerate"),
            "publish_raw": LaunchConfiguration("publish_raw"),
            "publish_compressed": LaunchConfiguration("publish_compressed"),
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
        capture_width_arg,
        capture_height_arg,
        framerate_arg,
        publish_raw_arg,
        publish_compressed_arg,
        device_index_arg,
        use_sim_time_arg,
        node,
    ])
