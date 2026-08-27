#!/usr/bin/env python3

import copy
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from launch_ros.actions import Node


XACRO_NAMESPACE = "{http://www.ros.org/wiki/xacro}"


# ============================================================
# MONTAJE FIJO myAGV -> mechArm 270 M5
# ============================================================
#
# Estas coordenadas forman parte de nuestra descripción del
# robot compuesto. Ya no se pasan por línea de comandos.
#
# Z se elevó con respecto a la primera versión porque el
# modelo del brazo quedaba ligeramente hundido en la tapa.
# ============================================================

ARM_MOUNT_X = 0.0
ARM_MOUNT_Y = 0.0
ARM_MOUNT_Z = 0.13

ARM_MOUNT_ROLL = 0.0
ARM_MOUNT_PITCH = 0.0
ARM_MOUNT_YAW = 0.0


def merge_robot_descriptions(context):

    # ========================================================
    # 1. Obtener los paquetes originales de Elephant Robotics
    # ========================================================

    myagv_share = get_package_share_directory(
        "myagv_description"
    )

    mycobot_share = get_package_share_directory(
        "mycobot_description"
    )

    # ========================================================
    # 2. Modelos originales
    # ========================================================

    myagv_urdf = os.path.join(
        myagv_share,
        "urdf",
        "myAGV.urdf"
    )

    # IMPORTANTE:
    # Ahora usamos el modelo oficial con Adaptive Gripper.
    mecharm_urdf = os.path.join(
        mycobot_share,
        "urdf",
        "mecharm_270_m5",
        "mecharm_270_m5_adaptive_gripper.urdf"
    )

    # ========================================================
    # 3. Comprobar archivos
    # ========================================================

    if not os.path.exists(myagv_urdf):
        raise FileNotFoundError(
            f"No se encontró el modelo del myAGV: {myagv_urdf}"
        )

    if not os.path.exists(mecharm_urdf):
        raise FileNotFoundError(
            f"No se encontró el modelo del MechArm: {mecharm_urdf}"
        )

    print("")
    print("==============================================")
    print("  MOBILE MANIPULATOR - ELEPHANT ROBOTICS")
    print("==============================================")
    print("")
    print("Modelo myAGV:")
    print(myagv_urdf)
    print("")
    print("Modelo MechArm + Adaptive Gripper:")
    print(mecharm_urdf)
    print("")
    print(
        "Montaje XYZ:",
        ARM_MOUNT_X,
        ARM_MOUNT_Y,
        ARM_MOUNT_Z
    )
    print(
        "Montaje RPY:",
        ARM_MOUNT_ROLL,
        ARM_MOUNT_PITCH,
        ARM_MOUNT_YAW
    )
    print("")
    print("==============================================")
    print("")

    # ========================================================
    # 4. Leer URDF
    # ========================================================

    myagv_tree = ET.parse(myagv_urdf)
    mecharm_tree = ET.parse(mecharm_urdf)

    myagv_root = myagv_tree.getroot()
    mecharm_root = mecharm_tree.getroot()

    # ========================================================
    # 5. Crear robot compuesto
    # ========================================================

    robot = ET.Element(
        "robot",
        {
            "name": "myagv_mecharm270_adaptive_gripper"
        }
    )

    # ========================================================
    # 6. Copiar myAGV original
    # ========================================================

    for element in list(myagv_root):

        if element.tag.startswith(XACRO_NAMESPACE):
            continue

        robot.append(
            copy.deepcopy(element)
        )

    # ========================================================
    # 7. Copiar MechArm original + Adaptive Gripper
    # ========================================================

    for element in list(mecharm_root):

        if element.tag.startswith(XACRO_NAMESPACE):
            continue

        robot.append(
            copy.deepcopy(element)
        )

    # ========================================================
    # 8. Joint fijo myAGV -> MechArm
    # ========================================================

    mount_joint = ET.SubElement(
        robot,
        "joint",
        {
            "name": "mecharm_mount_joint",
            "type": "fixed"
        }
    )

    ET.SubElement(
        mount_joint,
        "parent",
        {
            "link": "base_link"
        }
    )

    ET.SubElement(
        mount_joint,
        "child",
        {
            "link": "base"
        }
    )

    ET.SubElement(
        mount_joint,
        "origin",
        {
            "xyz": (
                f"{ARM_MOUNT_X} "
                f"{ARM_MOUNT_Y} "
                f"{ARM_MOUNT_Z}"
            ),
            "rpy": (
                f"{ARM_MOUNT_ROLL} "
                f"{ARM_MOUNT_PITCH} "
                f"{ARM_MOUNT_YAW}"
            )
        }
    )

    # ========================================================
    # 9. robot_description
    # ========================================================

    robot_description = ET.tostring(
        robot,
        encoding="unicode"
    )

    # ========================================================
    # 10. Robot State Publisher
    # ========================================================
    #
    # IMPORTANTE:
    # Usamos un joint_states propio para evitar interferencias
    # de otros nodos que publiquen en /joint_states.
    # ========================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="mobile_manipulator_state_publisher",
        output="screen",

        parameters=[
            {
                "robot_description": robot_description
            }
        ],

        remappings=[
            (
                "/joint_states",
                "/mobile_manipulator/joint_states"
            )
        ]
    )

    # ========================================================
    # 11. Joint State Publisher GUI
    # ========================================================

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="mobile_manipulator_joint_gui",
        output="screen",

        parameters=[
            {
                "robot_description": robot_description
            }
        ],

        remappings=[
            (
                "/joint_states",
                "/mobile_manipulator/joint_states"
            )
        ]
    )

    # ========================================================
    # 12. RViz
    # ========================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return [
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ]


def generate_launch_description():

    return LaunchDescription([

        OpaqueFunction(
            function=merge_robot_descriptions
        )

    ])
