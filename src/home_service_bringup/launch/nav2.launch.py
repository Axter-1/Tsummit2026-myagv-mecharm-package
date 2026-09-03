"""Nav2 para el ROBOT REAL, con la salida remapeada a /cmd_vel_nav.

En el robot real, twist_mux es quien publica /cmd_vel (lo consume
myagv_odometry). Nav2 debe publicar en /cmd_vel_nav para que twist_mux
lo arbitre frente a la aproximacion ArUco (/cmd_vel_aruco).

Uso:
    ros2 launch home_service_bringup nav2.launch.py \\
        map:=/workspace/maps/home_service_challenge_myagv.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():

    bringup_share = get_package_share_directory("home_service_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(
        bringup_share, "config", "nav2_real.yaml"
    )

    map_arg = DeclareLaunchArgument(
        "map",
        description="Ruta al .yaml del mapa.",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart", default_value="true"
    )

    nav2 = GroupAction([
        # Nav2 publica en /cmd_vel por defecto -> lo mandamos a twist_mux.
        SetRemap("/cmd_vel", "/cmd_vel_nav"),
        SetRemap("cmd_vel", "/cmd_vel_nav"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_bringup_share,
                    "launch",
                    "bringup_launch.py",
                )
            ),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": "false",
                "autostart": LaunchConfiguration("autostart"),
                "use_composition": "False",
            }.items(),
        ),
    ])

    return LaunchDescription([
        map_arg,
        params_arg,
        autostart_arg,
        nav2,
    ])
