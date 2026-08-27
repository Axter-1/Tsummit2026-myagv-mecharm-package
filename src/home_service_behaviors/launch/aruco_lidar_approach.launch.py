from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    lidar_approach = Node(
        package='home_service_behaviors',
        executable='aruco_lidar_approach_server',
        name='aruco_lidar_approach_server',
        output='screen',
        parameters=[{
        'use_sim_time': True,

        'detections_topic': '/aruco/detections',
        'scan_topic': '/scan',
        'odom_topic': '/odom',
        'cmd_vel_topic': '/cmd_vel_aruco',
        'action_name': '/aruco_lidar_approach',
        'odom_frame': 'odom',

        'search_angular_speed': 0.22,

        'lock_duration': 0.50,
        'lock_min_samples': 5,

        'kp_heading': 1.5,
        'max_heading_speed': 0.30,
        'heading_tolerance': 0.02,
        'heading_realign_threshold': 0.05,

        'kp_lateral': 0.08,
        'max_lateral_speed': 0.04,
        'lateral_tolerance': 0.04,
        'lateral_realign_threshold': 0.15,

        'kp_linear': 0.5,
        'max_linear_speed': 0.06,
        'distance_tolerance': 0.01,

        'camera_x_minus_lidar_x': 0.16,
        'lidar_sector_half_angle_deg': 4.0,

        'detection_timeout': 0.35,
        'scan_timeout': 0.35,
        'control_rate': 20.0,
}]
    )

    return LaunchDescription([
        lidar_approach
    ])
