from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    aruco_detector = Node(

        package="home_service_perception",

        executable="aruco_detector_node",

        name="aruco_detector",

        output="screen",

        parameters=[{

            "use_sim_time": True,

            "image_topic":
                "/camera/image_raw",

            "camera_info_topic":
                "/camera/camera_info",

            "marker_length":
                0.05,

            "equalize_hist":
                True,

            "publish_tf":
                True,
        }]
    )

    return LaunchDescription([
        aruco_detector
    ])
