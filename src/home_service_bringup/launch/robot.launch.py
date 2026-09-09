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
        DeclareLaunchArgument("camera_framerate", default_value="15"),
        DeclareLaunchArgument("camera_exposure_time_us", default_value="0"),
        DeclareLaunchArgument("camera_gain", default_value="0.0"),
        # En modo distribuido el robot NO publica la imagen cruda: por
        # WiFi solo viaja el JPEG (246.8 -> 6.1 Mbit/s medidos) y
        # publicar ambas seria gastar CPU de la Nano para nada.
        DeclareLaunchArgument("camera_publish_raw", default_value="true"),
        # En local el detector consume la imagen raw; comprimir ademas cada
        # frame con cv2.imencode solo añade carga CPU. El modo distribuido
        # lo activa explicitamente desde run_robot_routine.sh.
        DeclareLaunchArgument("camera_publish_compressed", default_value="false"),
        DeclareLaunchArgument("marker_length", default_value="0.075"),
        # Montaje de la camara respecto a base_link, en metros y radianes.
        # SIN MEDIR: son estimaciones. Un error aqui desplaza el marcador
        # en odom y la aproximacion se para donde no es.
        DeclareLaunchArgument("camera_x", default_value="0.16"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.07"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
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
            "framerate": LaunchConfiguration("camera_framerate"),
            "exposure_time_us": LaunchConfiguration("camera_exposure_time_us"),
            "gain": LaunchConfiguration("camera_gain"),
            "publish_raw": LaunchConfiguration("camera_publish_raw"),
            "publish_compressed": LaunchConfiguration(
                "camera_publish_compressed"
            ),
            "camera_name": "camera",
            # Frame OPTICO, no camera_link: el tvec de OpenCV viene en
            # convencion optica (x derecha, y abajo, z hacia delante),
            # mientras que camera_link sigue REP-103 (x delante, y
            # izquierda, z arriba). Etiquetarlo como camera_link
            # rotaba la pose 90 grados en dos ejes.
            "frame_id": "camera_optical_frame",
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
            "use_compressed": False,
            "equalize_hist": True,
            "publish_tf": True,
            "process_hz": 8.0,
            "opencv_threads": 1,
            "annotated_hz": 3.0,
            "detect_scale": 0.6,
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

    # -----------------------------------------------------------------
    # TF de la camara
    #
    # Sin esto 'aruco_<id>' cuelga de un frame que no existe en el
    # arbol, lookup_transform(odom, aruco_N) falla y la aproximacion
    # pierde la normal del marcador (falla en silencio: el except
    # devuelve None).
    #
    # Dos eslabones, como manda REP-103:
    #   base_link -> camera_link           montaje fisico
    #   camera_link -> camera_optical_frame  convencion optica (fija)
    # -----------------------------------------------------------------
    cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_camera",
        arguments=[
            LaunchConfiguration("camera_x"),
            LaunchConfiguration("camera_y"),
            LaunchConfiguration("camera_z"),
            LaunchConfiguration("camera_yaw"),
            LaunchConfiguration("camera_pitch"),
            LaunchConfiguration("camera_roll"),
            "base_link", "camera_link",
        ],
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    # Rotacion fija optica: -90 en Z y -90 en X. No se toca.
    cam_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera_optical",
        arguments=[
            "0", "0", "0",
            "-1.5707963267948966", "0", "-1.5707963267948966",
            "camera_link", "camera_optical_frame",
        ],
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    return LaunchDescription(
        args
        + [camera, cam_tf, cam_optical_tf, scan_sanitizer, aruco_detector,
           aruco_approach, mecharm, twist_mux]
    )
