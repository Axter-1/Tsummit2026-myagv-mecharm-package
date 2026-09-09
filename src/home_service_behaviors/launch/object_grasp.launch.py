#!/usr/bin/env python3
"""Lanza el modulo de toma de pieza (object_grasp_server).

Solo arranca el orquestador. Los tres nodos de los que depende los
levanta scripts/tsummit.sh por separado, porque cada uno tiene su propio
ciclo de vida y su propio hardware:

    camara + detector ArUco -> home_service_bringup/robot.launch.py
    servidor de aproximacion -> home_service_bringup/robot.launch.py
    driver del MechArm       -> myagv_mecharm_service/mecharm_driver.launch.py

Argumentos utiles para ensayar sin romper nada:
    enable_arm:=false       identifica y aproxima, pero NO mueve el brazo
    enable_approach:=false  la pieza ya esta delante; solo agarra
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("home_service_behaviors")
    default_catalog = os.path.join(share, "config", "grasp_catalog.yaml")

    args = [
        DeclareLaunchArgument(
            "catalog_file",
            default_value=default_catalog,
            description="Ruta a grasp_catalog.yaml",
        ),
        DeclareLaunchArgument(
            "enable_arm",
            default_value="true",
            description="false = ensayo sin mover el brazo",
        ),
        DeclareLaunchArgument(
            "enable_approach",
            default_value="true",
            description="false = no mover la base (pieza ya delante)",
        ),
        DeclareLaunchArgument(
            "approach_stop_distance",
            default_value="0.20",
            description="Distancia (m) a la que se para la base del ArUco",
        ),
        DeclareLaunchArgument(
            "approach_timeout_sec",
            default_value="120.0",
            description="Tiempo maximo (s) para encontrar y aproximar el ArUco",
        ),
        DeclareLaunchArgument(
            "detections_topic",
            default_value="/aruco/detections",
        ),
        DeclareLaunchArgument("scan_topic", default_value="/scan_filtered"),
        DeclareLaunchArgument(
            "table_height_mm",
            default_value="100",
            description="Altura (mm) de la plataforma: elige la calibracion "
            "de agarre (grasp_calibrations.yaml). 100 o 200.",
        ),
    ]

    node = Node(
        package="home_service_behaviors",
        executable="object_grasp_server",
        name="object_grasp_server",
        output="screen",
        parameters=[{
            "catalog_file": LaunchConfiguration("catalog_file"),
            "enable_arm": LaunchConfiguration("enable_arm"),
            "enable_approach": LaunchConfiguration("enable_approach"),
            "approach_stop_distance": LaunchConfiguration(
                "approach_stop_distance"
            ),
            "approach_timeout_sec": ParameterValue(
                LaunchConfiguration("approach_timeout_sec"), value_type=float
            ),
            "detections_topic": LaunchConfiguration("detections_topic"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "table_height_mm": ParameterValue(
                LaunchConfiguration("table_height_mm"), value_type=int
            ),
        }],
    )

    return LaunchDescription(args + [node])
