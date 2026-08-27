from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ========================================================
    # Velocidades configurables
    # ========================================================

    search_speed_arg = DeclareLaunchArgument(
        "search_angular_speed",
        default_value="0.15"
    )

    heading_speed_arg = DeclareLaunchArgument(
        "max_heading_speed",
        default_value="0.30"
    )

    lateral_speed_arg = DeclareLaunchArgument(
        "max_lateral_speed",
        default_value="0.04"
    )

    linear_speed_arg = DeclareLaunchArgument(
        "max_linear_speed",
        default_value="0.05"
    )

    # ========================================================
    # Nodo
    # ========================================================

    server = Node(

        package="home_service_behaviors",

        executable="aruco_approach_server",

        name="aruco_approach_server",

        output="screen",

        parameters=[{

            # Simulación
            "use_sim_time":
                True,

            # Topics
            "detections_topic":
                "/aruco/detections",

            "cmd_vel_topic":
                "/cmd_vel_aruco",

            "odom_topic":
                "/odom",

            # Cámara respecto a base_link
            "camera_x_offset":
                0.160,

            "camera_y_offset":
                -0.004,

            # Control
            "control_rate":
                20.0,

            "detection_timeout":
                0.35,

            # =================================================
            # SEARCHING
            # =================================================

            "search_angular_speed":
                ParameterValue(
                    LaunchConfiguration(
                        "search_angular_speed"
                    ),
                    value_type=float
                ),

            # =================================================
            # LOCK_TARGET
            # =================================================

            "target_lock_sec":
                0.50,

            "target_lock_min_samples":
                5,

            # =================================================
            # ALIGN HEADING
            # =================================================

            "kp_heading":
                1.5,

            "max_heading_speed":
                ParameterValue(
                    LaunchConfiguration(
                        "max_heading_speed"
                    ),
                    value_type=float
                ),

            "heading_tolerance":
                0.02,

            "heading_realign_threshold":
                0.04,

            # =================================================
            # ALIGNING_LATERAL
            # =================================================

            "kp_lateral":
                0.8,

            "max_lateral_speed":
                ParameterValue(
                    LaunchConfiguration(
                        "max_lateral_speed"
                    ),
                    value_type=float
                ),

            "lateral_tolerance":
                0.01,

            "lateral_realign_threshold":
                0.02,

            # =================================================
            # APPROACHING
            # =================================================

            "kp_linear":
                0.5,

            "max_linear_speed":
                ParameterValue(
                    LaunchConfiguration(
                        "max_linear_speed"
                    ),
                    value_type=float
                ),

            "max_reverse_speed":
                0.02,

            "distance_tolerance":
                0.01,
        }]
    )

    return LaunchDescription([

        search_speed_arg,

        heading_speed_arg,

        lateral_speed_arg,

        linear_speed_arg,

        server,
    ])
