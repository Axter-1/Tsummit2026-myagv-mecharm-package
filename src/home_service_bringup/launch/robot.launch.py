"""Bringup del robot REAL: capa de percepcion + comportamiento + brazo.

Lanza:
  * camara CSI            (myagv_camera)
  * detector de ArUco     (home_service_perception)
  * aproximacion ArUco+LiDAR (home_service_behaviors)
  * driver del MechArm    (myagv_mecharm_service)
  * twist_mux             (arbitraje de /cmd_vel)

NO lanza la odometria ni el LiDAR de la base: eso lo hace
docker/run_all_robot_nodes.sh (myagv_odometry + ydlidar_ros2_driver).
Nav2 y la mision se lanzan por separado.

Enrutado de velocidad en el robot real:
  Nav2        -> /cmd_vel_nav    (prioridad 50)
  ArUco       -> /cmd_vel_aruco  (prioridad 100)
  teleop joy  -> /cmd_vel_joy    (prioridad 200)
  twist_mux   -> /cmd_vel        (lo consume myagv_odometry)
"""

import os

from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    camera_share = get_package_share_directory("myagv_camera")
    mecharm_share = get_package_share_directory("myagv_mecharm_service")
    behaviors_share = get_package_share_directory("home_service_behaviors")
    bringup_share = get_package_share_directory("home_service_bringup")

    # -----------------------------------------------------------------
    # Argumentos
    # -----------------------------------------------------------------
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("camera_source", default_value="nvargus"),
        DeclareLaunchArgument("camera_flip_method", default_value="2"),
        # En modo distribuido el robot NO publica la imagen cruda: por
        # WiFi solo viaja el JPEG (246.8 -> 6.1 Mbit/s medidos) y
        # publicar ambas seria gastar CPU de la Nano para nada.
        DeclareLaunchArgument("camera_publish_raw", default_value="true"),
        DeclareLaunchArgument("camera_publish_compressed", default_value="true"),
        DeclareLaunchArgument("marker_length", default_value="0.08"),
        DeclareLaunchArgument("arm_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_aruco_detector", default_value="true"),
        DeclareLaunchArgument("start_aruco_approach", default_value="true"),
        DeclareLaunchArgument("start_arm", default_value="true"),
        DeclareLaunchArgument("start_twist_mux", default_value="true"),
        DeclareLaunchArgument("start_scan_sanitizer", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "scan_filtered_topic", default_value="/scan_filtered"
        ),
        DeclareLaunchArgument(
            "blind_sectors_deg", default_value="[-50.0, 50.0]"
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # -----------------------------------------------------------------
    # Camara CSI
    # -----------------------------------------------------------------
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_share, "launch", "csi_camera.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "source": LaunchConfiguration("camera_source"),
            "flip_method": LaunchConfiguration("camera_flip_method"),
            "publish_raw": LaunchConfiguration("camera_publish_raw"),
            "publish_compressed": LaunchConfiguration(
                "camera_publish_compressed"
            ),
            "camera_name": "camera",
            "frame_id": "camera_link",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    # -----------------------------------------------------------------
    # Detector de ArUco
    # -----------------------------------------------------------------
    aruco_detector = Node(
        package="home_service_perception",
        executable="aruco_detector_node",
        name="aruco_detector",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "image_topic": "/camera/image_raw",
            "camera_info_topic": "/camera/camera_info",
            "marker_length": LaunchConfiguration("marker_length"),
            "equalize_hist": True,
            "publish_tf": True,
        }],
        condition=IfCondition(LaunchConfiguration("start_aruco_detector")),
    )

    # -----------------------------------------------------------------
    # Saneador del LaserScan
    #
    # Convierte los haces sin eco (0.0) en +inf para que Nav2 pueda
    # limpiar el costmap, y descarta auto-impactos y motas. Tambien
    # mejora la medida de distancia de la aproximacion ArUco.
    # -----------------------------------------------------------------
    scan_sanitizer = Node(
        package="home_service_navigation",
        executable="scan_sanitizer_node",
        name="scan_sanitizer",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": LaunchConfiguration("scan_topic"),
            "output_topic": LaunchConfiguration("scan_filtered_topic"),
            "range_min": 0.16,
            "range_max": 5.0,
            "blind_sectors_deg": ParameterValue(
                LaunchConfiguration("blind_sectors_deg"),
                value_type=List[float],
            ),
            "zeros_to_inf": True,
            "speckle_filter": True,
        }],
        condition=IfCondition(
            LaunchConfiguration("start_scan_sanitizer")
        ),
    )

    # -----------------------------------------------------------------
    # Aproximacion ArUco + LiDAR
    # -----------------------------------------------------------------
    aruco_approach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                behaviors_share,
                "launch",
                "aruco_lidar_approach.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "scan_topic": LaunchConfiguration("scan_filtered_topic"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_aruco_approach")),
    )

    # -----------------------------------------------------------------
    # Driver del MechArm 270
    # -----------------------------------------------------------------
    mecharm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mecharm_share, "launch", "mecharm_driver.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "port": LaunchConfiguration("arm_port"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_arm")),
    )

    # -----------------------------------------------------------------
    # twist_mux
    # -----------------------------------------------------------------
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            os.path.join(
                bringup_share, "config", "twist_mux_real.yaml"
            ),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("cmd_vel_out", "/cmd_vel")],
        condition=IfCondition(LaunchConfiguration("start_twist_mux")),
    )

    return LaunchDescription(
        args
        + [camera, scan_sanitizer, aruco_detector, aruco_approach,
           mecharm, twist_mux]
    )
