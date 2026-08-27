import copy
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


XACRO_NAMESPACE = "{http://www.ros.org/wiki/xacro}"

# ============================================================
# Cámara frontal myAGV - simulación
# ============================================================

CAMERA_WIDTH = 960
CAMERA_HEIGHT = 540
CAMERA_UPDATE_RATE = 21.0

# Aproximación del FOV de la cámara física usando fx ~= 785.855 px
# a resolución 960x540.
CAMERA_HFOV = 1.096645

# Pose inicial respecto a base_link.
#
# IMPORTANTE:
# estos valores son de calibración geométrica de la simulación,
# no coordenadas oficiales del fabricante.
CAMERA_X = 0.160
CAMERA_Y = - 0.004
CAMERA_Z = 0.070

CAMERA_ROLL = 0.0
CAMERA_PITCH = 0.0
CAMERA_YAW = 0.0

ARM_JOINTS = [
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
]

# ============================================================
# MONTAJE myAGV -> MechArm
# ============================================================
#
# IMPORTANTE:
# coloca aquí EXACTAMENTE los valores con los que ya quedó
# correctamente colocado el brazo en RViz.
#
# Si 0.135 fue el valor final, déjalo así.
# ============================================================

ARM_MOUNT_X = 0.0
ARM_MOUNT_Y = 0.0
ARM_MOUNT_Z = 0.135

ARM_MOUNT_ROLL = 0.0
ARM_MOUNT_PITCH = 0.0
ARM_MOUNT_YAW = 0.0


# ============================================================
# myAGV - aproximación física
# ============================================================

# Dimensiones aproximadas derivadas del plano físico.
CHASSIS_MASS = 3.76

CHASSIS_LENGTH = 0.250
CHASSIS_WIDTH = 0.180
CHASSIS_HEIGHT = 0.055

CHASSIS_Z = 0.0675

# ============================================================
# myAGV Jetson Nano 2023
# ============================================================
#
# Dimensiones exteriores publicadas:
#
# 331.15 mm x 230 mm x 110 mm
#
# Convención ROS:
#
# X -> frente
# Y -> izquierda
# Z -> arriba
#
# ============================================================

MYAGV_LENGTH = 0.33115
MYAGV_WIDTH = 0.230
MYAGV_HEIGHT = 0.110


# ============================================================
# Ruedas Mecanum
# ============================================================
#
# El URDF oficial no proporciona links individuales de ruedas.
# Estas dimensiones son aproximaciones físicas para Gazebo.
#
# Visualmente las ruedas continúan viniendo del DAE original.
# ============================================================

WHEEL_RADIUS = 0.040
WHEEL_WIDTH = 0.032
WHEEL_MASS = 0.10


# El centro longitudinal se calcula quitando un radio
# a cada extremo del largo total.
# El centro lateral se calcula quitando media anchura de
# rueda al ancho exterior.
# El eje de la rueda queda a un radio del suelo.

# WHEEL_X = (
#   MYAGV_LENGTH / 2.0
#   - WHEEL_RADIUS
#)

# WHEEL_Y = (
#   MYAGV_WIDTH / 2.0
#   - WHEEL_WIDTH / 2.0
#)

# WHEEL_Z = WHEEL_RADIUS

# ============================================================
# Ruedas físicas calibradas contra el mesh original del myAGV
# ============================================================

WHEEL_RADIUS = 0.040
WHEEL_WIDTH  = 0.032

# Estos valores ya NO se derivan del tamaño exterior.
# Son valores de calibración visual contra el agujero/rueda del mesh.

FRONT_WHEEL_X = 0.110
REAR_WHEEL_X = -0.096
WHEEL_Y = 0.095
WHEEL_Z = 0.040

# FL = front left
# FR = front right
# RL = rear left
# RR = rear right

#WHEEL_POSITIONS = {
#
#    "wheel_fl": (
#        +WHEEL_X,
#        +WHEEL_Y,
#        WHEEL_Z
#    ),
#
#    "wheel_fr": (
#        +WHEEL_X,
#        -WHEEL_Y,
#        WHEEL_Z
#    ),
#
#    "wheel_rl": (
#        -WHEEL_X,
#        +WHEEL_Y,
#        WHEEL_Z
#    ),
#
#    "wheel_rr": (
#        -WHEEL_X,
#        -WHEEL_Y,
#        WHEEL_Z
#    ),
#}

WHEEL_POSITIONS = {
    "wheel_fl": (FRONT_WHEEL_X, +WHEEL_Y, WHEEL_Z),
    "wheel_fr": (FRONT_WHEEL_X, -WHEEL_Y, WHEEL_Z),
    "wheel_rl": (REAR_WHEEL_X, +WHEEL_Y, WHEEL_Z),
    "wheel_rr": (REAR_WHEEL_X, -WHEEL_Y, WHEEL_Z),
}

# ============================================================
# Funciones de utilidad
# ============================================================

def remove_element(parent, tag):
    for element in list(parent.findall(tag)):
        parent.remove(element)


def add_box_inertial(link, mass, x, y, z, origin=(0.0, 0.0, 0.0)):

    remove_element(link, "inertial")

    ixx = mass * (y * y + z * z) / 12.0
    iyy = mass * (x * x + z * z) / 12.0
    izz = mass * (x * x + y * y) / 12.0

    inertial = ET.SubElement(link, "inertial")

    ET.SubElement(
        inertial,
        "origin",
        {
            "xyz": f"{origin[0]} {origin[1]} {origin[2]}",
            "rpy": "0 0 0",
        },
    )

    ET.SubElement(
        inertial,
        "mass",
        {"value": str(mass)},
    )

    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": str(ixx),
            "ixy": "0.0",
            "ixz": "0.0",
            "iyy": str(iyy),
            "iyz": "0.0",
            "izz": str(izz),
        },
    )


def add_cylinder_inertial_y(
    link,
    mass,
    radius,
    length,
    origin=(0.0, 0.0, 0.0),
):

    remove_element(link, "inertial")

    # Cilindro cuyo eje físico es Y
    i_axis = 0.5 * mass * radius * radius

    i_side = (
        mass
        * (3.0 * radius * radius + length * length)
        / 12.0
    )

    inertial = ET.SubElement(link, "inertial")

    ET.SubElement(
        inertial,
        "origin",
        {
            "xyz": f"{origin[0]} {origin[1]} {origin[2]}",
            "rpy": "0 0 0",
        },
    )

    ET.SubElement(
        inertial,
        "mass",
        {"value": str(mass)},
    )

    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": str(i_side),
            "ixy": "0.0",
            "ixz": "0.0",
            "iyy": str(i_axis),
            "iyz": "0.0",
            "izz": str(i_side),
        },
    )


def add_box_collision(
    link,
    x,
    y,
    z,
    origin=(0.0, 0.0, 0.0),
):

    collision = ET.SubElement(link, "collision")

    ET.SubElement(
        collision,
        "origin",
        {
            "xyz": f"{origin[0]} {origin[1]} {origin[2]}",
            "rpy": "0 0 0",
        },
    )

    geometry = ET.SubElement(collision, "geometry")

    ET.SubElement(
        geometry,
        "box",
        {
            "size": f"{x} {y} {z}"
        },
    )


def add_cylinder_collision_y(
    link,
    radius,
    length,
    origin=(0.0, 0.0, 0.0),
):

    collision = ET.SubElement(link, "collision")

    ET.SubElement(
        collision,
        "origin",
        {
            "xyz": f"{origin[0]} {origin[1]} {origin[2]}",
            "rpy": f"{math.pi / 2.0} 0 0",
        },
    )

    geometry = ET.SubElement(collision, "geometry")

    ET.SubElement(
        geometry,
        "cylinder",
        {
            "radius": str(radius),
            "length": str(length),
        },
    )

# ============================================================
# Contacto de ruedas en simulación Mecanum ideal
# ============================================================

def add_sim_wheel_contact(
    robot,
    link_name
):

    # Las ruedas siguen teniendo colisión vertical para
    # soportar físicamente el robot sobre el suelo.
    #
    # Pero eliminamos prácticamente toda la fricción
    # tangencial porque el movimiento holonómico de la base
    # será aplicado por gazebo_ros_planar_move.
    #
    # Esto evita que los cilindros convencionales bloqueen
    # linear.y y angular.z.

    gazebo = ET.SubElement(
        robot,
        "gazebo",
        {
            "reference": link_name
        }
    )

    mu1 = ET.SubElement(
        gazebo,
        "mu1"
    )
    mu1.text = "0.001"

    mu2 = ET.SubElement(
        gazebo,
        "mu2"
    )
    mu2.text = "0.001"

    kp = ET.SubElement(
        gazebo,
        "kp"
    )
    kp.text = "1000000.0"

    kd = ET.SubElement(
        gazebo,
        "kd"
    )
    kd.text = "1.0"

    min_depth = ET.SubElement(
        gazebo,
        "minDepth"
    )
    min_depth.text = "0.001"

    max_vel = ET.SubElement(
        gazebo,
        "maxVel"
    )
    max_vel.text = "0.1"

# ============================================================
# Ruedas físicas
# ============================================================

def add_wheel(
    robot,
    name,
    x,
    y,
    z
):

    link_name = f"{name}_link"
    joint_name = f"{name}_joint"

    # ========================================================
    # LINK
    # ========================================================

    link = ET.SubElement(
        robot,
        "link",
        {
            "name": link_name
        }
    )

    # ========================================================
    # DEBUG VISUAL
    # ========================================================

    visual = ET.SubElement(
        link,
        "visual",
        {
            "name": f"{name}_debug_visual"
        }
    )

    ET.SubElement(
        visual,
        "origin",
        {
            "xyz": "0 0 0",
            "rpy": f"{math.pi / 2.0} 0 0",
        }
    )

    geometry = ET.SubElement(
        visual,
        "geometry"
    )

    ET.SubElement(
        geometry,
        "cylinder",
        {
            "radius": str(WHEEL_RADIUS),
            "length": str(WHEEL_WIDTH),
        }
    )

    # --------------------------------------------------------
    # Inercia
    # --------------------------------------------------------

    add_cylinder_inertial_y(
        link,
        WHEEL_MASS,
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        origin=(0.0, 0.0, 0.0),
    )

    # --------------------------------------------------------
    # Collision
    #
    # IMPORTANTE:
    # origin = 0 dentro del wheel_link.
    #
    # La posición física de la rueda pertenece al JOINT.
    # --------------------------------------------------------

    add_cylinder_collision_y(
        link,
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        origin=(0.0, 0.0, 0.0),
    )

    # --------------------------------------------------------
    # Física de contacto para simulación Mecanum ideal
    # --------------------------------------------------------

    add_sim_wheel_contact(
        robot,
        link_name
    )

    # ========================================================
    # JOINT
    # ========================================================

    joint = ET.SubElement(
        robot,
        "joint",
        {
            "name": joint_name,
            "type": "continuous",
        }
    )

    ET.SubElement(
        joint,
        "parent",
        {
            "link": "base_link"
        }
    )

    ET.SubElement(
        joint,
        "child",
        {
            "link": link_name
        }
    )

    # Aquí es donde realmente se posiciona cada rueda.

    ET.SubElement(
        joint,
        "origin",
        {
            "xyz": f"{x} {y} {z}",
            "rpy": "0 0 0",
        }
    )

    # Eje de rotación.
    #
    # ROS:
    # X = adelante
    # Y = izquierda
    # Z = arriba

    ET.SubElement(
        joint,
        "axis",
        {
            "xyz": "0 1 0"
        }
    )

    ET.SubElement(
    joint,
    "limit",
    {
        "effort": "5.0",
        "velocity": "25.0",
    }
    )

    ET.SubElement(
        joint,
        "dynamics",
        {
            "damping": "0.05",
            "friction": "0.02",
        }
    )

# ============================================================
# Física del MechArm
# ============================================================

def configure_arm_physics(robot):

    # --------------------------------------------------------
    # Distribución aproximada.
    #
    # La suma de base + link1 ... link6 = 1.0 kg,
    # coincidiendo con la masa total indicada por Elephant.
    # --------------------------------------------------------

    arm_data = {

        "base": {
            "mass": 0.25,
            "size": (0.10, 0.10, 0.10),
            "origin": (0.0, 0.0, 0.05),
        },

        "link1": {
            "mass": 0.15,
            "size": (0.065, 0.065, 0.10),
            "origin": (0.0, 0.0, 0.04),
        },

        "link2": {
            "mass": 0.16,
            "size": (0.055, 0.10, 0.055),
            "origin": (0.0, -0.05, 0.0),
        },

        "link3": {
            "mass": 0.16,
            "size": (0.108, 0.055, 0.055),
            "origin": (0.054, 0.0, 0.0),
        },

        "link4": {
            "mass": 0.10,
            "size": (0.065, 0.055, 0.055),
            "origin": (0.03, 0.0, 0.0),
        },

        "link5": {
            "mass": 0.09,
            "size": (0.060, 0.050, 0.050),
            "origin": (0.03, 0.0, 0.0),
        },

        "link6": {
            "mass": 0.09,
            "size": (0.050, 0.050, 0.050),
            "origin": (0.02, 0.0, 0.0),
        },
    }

    for link_name, data in arm_data.items():

        link = robot.find(
            f"./link[@name='{link_name}']"
        )

        if link is None:
            raise RuntimeError(
                f"No se encontró {link_name}"
            )

        # No utilizamos los meshes DAE originales como collision.
        # El visual original NO se toca.

        remove_element(
            link,
            "collision",
        )

        sx, sy, sz = data["size"]

        add_box_inertial(
            link,
            data["mass"],
            sx,
            sy,
            sz,
            data["origin"],
        )

        add_box_collision(
            link,
            sx,
            sy,
            sz,
            data["origin"],
        )

    # --------------------------------------------------------
    # Gripper
    # --------------------------------------------------------

    gripper_data = {

        "gripper_base":
            (0.08, 0.055, 0.040, 0.040),

        "gripper_left1":
            (0.01, 0.040, 0.012, 0.012),

        "gripper_left2":
            (0.01, 0.040, 0.012, 0.012),

        "gripper_left3":
            (0.01, 0.040, 0.012, 0.012),

        "gripper_right1":
            (0.01, 0.040, 0.012, 0.012),

        "gripper_right2":
            (0.01, 0.040, 0.012, 0.012),

        "gripper_right3":
            (0.01, 0.040, 0.012, 0.012),
    }

    for link_name, values in gripper_data.items():

        mass, sx, sy, sz = values

        link = robot.find(
            f"./link[@name='{link_name}']"
        )

        if link is None:
            continue

        remove_element(
            link,
            "collision",
        )

        add_box_inertial(
            link,
            mass,
            sx,
            sy,
            sz,
        )

        add_box_collision(
            link,
            sx,
            sy,
            sz,
        )


# ============================================================
# Dinámica de joints
# ============================================================

def configure_joint_dynamics(robot):

    arm_joints = [

        "joint1_to_base",
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
    ]

    for joint_name in arm_joints:

        joint = robot.find(
            f"./joint[@name='{joint_name}']"
        )

        if joint is None:
            continue

        dynamics = joint.find("dynamics")

        if dynamics is None:
            dynamics = ET.SubElement(
                joint,
                "dynamics"
            )

        # Aproximación temporal del holding torque de los servos.
        # En ETAPA 4 ros2_control será quien mantenga las posiciones.

        dynamics.set(
            "damping",
            "2.0"
        )

        dynamics.set(
            "friction",
            "2.0"
        )

        limit = joint.find("limit")

        if limit is not None:

            # 120 grados/s
            limit.set(
                "velocity",
                str(math.radians(120.0))
            )

    # Gripper
    gripper_joints = [

        "gripper_controller",
        "gripper_base_to_gripper_left2",
        "gripper_left3_to_gripper_left1",
        "gripper_base_to_gripper_right3",
        "gripper_base_to_gripper_right2",
        "gripper_right3_to_gripper_right1",
    ]

    for joint_name in gripper_joints:

        joint = robot.find(
            f"./joint[@name='{joint_name}']"
        )

        if joint is None:
            continue

        dynamics = joint.find("dynamics")

        if dynamics is None:
            dynamics = ET.SubElement(
                joint,
                "dynamics"
            )

        dynamics.set(
            "damping",
            "0.5"
        )

        dynamics.set(
            "friction",
            "0.4"
        )

        limit = joint.find("limit")

        if limit is not None:
            limit.set(
                "velocity",
                "1.0"
            )


# ============================================================
# Construcción final
# ============================================================

def build_robot_description():

    myagv_share = get_package_share_directory(
        "myagv_description"
    )

    mycobot_share = get_package_share_directory(
        "mycobot_description"
    )

    myagv_file = os.path.join(
        myagv_share,
        "urdf",
        "myAGV.urdf"
    )

    mecharm_file = os.path.join(
        mycobot_share,
        "urdf",
        "mecharm_270_m5",
        "mecharm_270_m5_adaptive_gripper.urdf"
    )

    myagv_root = ET.parse(
        myagv_file
    ).getroot()

    mecharm_root = ET.parse(
        mecharm_file
    ).getroot()

    robot = ET.Element(
        "robot",
        {
            "name":
                "myagv_mecharm270_gazebo"
        }
    )

    # --------------------------------------------------------
    # Copiar modelos originales
    # --------------------------------------------------------

    for element in list(myagv_root):

        if element.tag.startswith(
            XACRO_NAMESPACE
        ):
            continue

        robot.append(
            copy.deepcopy(element)
        )

    for element in list(mecharm_root):

        if element.tag.startswith(
            XACRO_NAMESPACE
        ):
            continue

        robot.append(
            copy.deepcopy(element)
        )

    # ========================================================
    # Física myAGV
    # ========================================================

    base = robot.find(
        "./link[@name='base_link']"
    )

    add_box_inertial(
        base,
    	CHASSIS_MASS,
    	CHASSIS_LENGTH,
    	CHASSIS_WIDTH,
    	CHASSIS_HEIGHT,
    	(0.0, 0.0, CHASSIS_Z),
    )

    add_box_collision(
    	base,
    	CHASSIS_LENGTH,
    	CHASSIS_WIDTH,
    	CHASSIS_HEIGHT,
    	(0.0, 0.0, CHASSIS_Z),
    )

    # ========================================================
    # Cuatro ruedas físicas
    # ========================================================

    for wheel_name, position in WHEEL_POSITIONS.items():
        x, y, z = position

        print(
        	f"[WHEEL] {wheel_name}: "
        	f"x={x:.6f}, "
        	f"y={y:.6f}, "
        	f"z={z:.6f}"
    	)

        add_wheel(
        	robot,
        	wheel_name,
        	x,
        	y,
        	z
    	)

    # ========================================================
    # Unión myAGV -> MechArm
    # ========================================================

    mount = ET.SubElement(
        robot,
        "joint",
        {
            "name": "mecharm_mount_joint",
            "type": "fixed",
        },
    )

    ET.SubElement(
        mount,
        "parent",
        {"link": "base_link"},
    )

    ET.SubElement(
        mount,
        "child",
        {"link": "base"},
    )

    ET.SubElement(
        mount,
        "origin",
        {
            "xyz":
                f"{ARM_MOUNT_X} "
                f"{ARM_MOUNT_Y} "
                f"{ARM_MOUNT_Z}",

            "rpy":
                f"{ARM_MOUNT_ROLL} "
                f"{ARM_MOUNT_PITCH} "
                f"{ARM_MOUNT_YAW}",
        },
    )

    # ========================================================
    # Propiedades físicas del brazo
    # ========================================================

    configure_arm_physics(
        robot
    )

    # ========================================================
    # Corregir defectos geométricos del URDF oficial
    # ========================================================

    patch_elephant_urdf_for_gazebo(
        robot
    )

    # ========================================================
    # ETAPA 4:
    # brazo libre, gripper temporalmente bloqueado
    # ========================================================

    lock_gripper_for_stage4(
        robot
    )

    # ========================================================
    # ETAPA 2:
    # brazo rígido temporalmente
    # ========================================================
    # lock_arm_for_stage2(
    #     robot
    # )

    # ========================================================
    # ros2_control:
    #
    # - 4 ruedas
    # - 6 joints MechArm
    # ========================================================
    add_ros2_control(
        robot
    )

    # ========================================================
    # Base Mecanum ideal para Gazebo Classic
    # ========================================================

    add_planar_mecanum_sim(
        robot
    )

    # ========================================================
    # FASE 5
    # Cámara frontal simulada
    # ========================================================

    add_sim_camera(
        robot
    )
    # ========================================================
    # LiDAR 2D simulado
    # ========================================================

    add_sim_lidar(
        robot
    )

    # ========================================================
    # Resolver meshes para Gazebo Classic
    # ========================================================

    resolve_package_mesh_uris(
        robot
    )

    return ET.tostring(
        robot,
        encoding="unicode"
    )

def configure_arm_physics(robot):

    # ========================================================
    # Masas aproximadas.
    #
    # Total:
    # 0.25 + 0.15 + 0.16 + 0.16 + 0.10 + 0.09 + 0.09
    # = 1.00 kg
    #
    # Coincide con la masa total publicada del MechArm.
    # ========================================================

    arm_data = {

        "base": {
            "mass": 0.25,
            "size": (0.10, 0.10, 0.10),
            "com": (0.0, 0.0, 0.04),
        },

        "link1": {
            "mass": 0.15,
            "size": (0.065, 0.065, 0.060),
            "com": (0.0, 0.0, 0.019),
        },

        "link2": {
            "mass": 0.16,
            "size": (0.060, 0.110, 0.060),
            "com": (0.0, -0.05, 0.0),
        },

        "link3": {
            "mass": 0.16,
            "size": (0.115, 0.060, 0.060),
            "com": (0.054, -0.0025, 0.0),
        },

        "link4": {
            "mass": 0.10,
            "size": (0.060, 0.060, 0.060),
            "com": (0.0, 0.0, 0.0),
        },

        "link5": {
            "mass": 0.09,
            "size": (0.065, 0.055, 0.055),
            "com": (0.030, 0.0, 0.0),
        },

        "link6": {
            "mass": 0.09,
            "size": (0.050, 0.050, 0.050),
            "com": (0.0, 0.0, 0.019),
        },
    }

    for link_name, data in arm_data.items():

        link = robot.find(
            f"./link[@name='{link_name}']"
        )

        if link is None:
            raise RuntimeError(
                f"No se encontró el link {link_name}"
            )

        sx, sy, sz = data["size"]

        add_box_inertial(
            link,
            data["mass"],
            sx,
            sy,
            sz,
            data["com"],
        )

    # ========================================================
    # Gripper
    # ========================================================

    gripper_data = {

        "gripper_base":
            (0.08, 0.055, 0.055, 0.040),

        "gripper_left1":
            (0.01, 0.040, 0.012, 0.012),

        "gripper_left2":
            (0.01, 0.040, 0.012, 0.012),

        "gripper_left3":
            (0.01, 0.040, 0.012, 0.012),

        "gripper_right1":
            (0.01, 0.040, 0.012, 0.012),

        "gripper_right2":
            (0.01, 0.040, 0.012, 0.012),

        "gripper_right3":
            (0.01, 0.040, 0.012, 0.012),
    }

    for link_name, values in gripper_data.items():

        mass, sx, sy, sz = values

        link = robot.find(
            f"./link[@name='{link_name}']"
        )

        if link is None:
            continue

        add_box_inertial(
            link,
            mass,
            sx,
            sy,
            sz,
            (0.0, 0.0, 0.0),
        )

        # También conservamos la collision
        # original del gripper.    

    configure_joint_dynamics(
        robot
    )

    return ET.tostring(
        robot,
        encoding="unicode"
    )

    # ========================================================
    # Corregir las rutas de los archivos
    # Así el URDF enviado a Gazebo ya no tendrá:
    #
    # package://mycobot_description/...
    #
    # sino:
    #
    # file://<workspace>/install/...
    # ========================================================

def resolve_package_mesh_uris(robot):

    package_paths = {
        "myagv_description": Path(
            get_package_share_directory("myagv_description")
        ),

        "mycobot_description": Path(
            get_package_share_directory("mycobot_description")
        ),
    }

    for mesh in robot.findall(".//mesh"):

        filename = mesh.get("filename")

        if not filename:
            continue

        if not filename.startswith("package://"):
            continue

        relative_uri = filename[len("package://"):]

        parts = relative_uri.split("/", 1)

        if len(parts) != 2:
            raise RuntimeError(
                f"URI package inválida: {filename}"
            )

        package_name = parts[0]
        relative_path = parts[1]

        if package_name not in package_paths:
            raise RuntimeError(
                f"Paquete desconocido en mesh: {package_name}"
            )

        absolute_path = (
            package_paths[package_name]
            / relative_path
        ).resolve()

        if not absolute_path.exists():
            raise FileNotFoundError(
                f"No existe el mesh:\n{absolute_path}"
            )

        # package://... -> file:///home/...
        mesh.set(
            "filename",
            absolute_path.as_uri()
        )
def make_joint_fixed(robot, joint_name):

    joint = robot.find(
        f"./joint[@name='{joint_name}']"
    )

    if joint is None:
        raise RuntimeError(
            f"No se encontró el joint {joint_name}"
        )

    joint.set("type", "fixed")

    # Los joints fixed no necesitan estos elementos.

    for tag in [
        "axis",
        "limit",
        "dynamics",
        "mimic",
        "safety_controller",
        "calibration",
    ]:

        for element in list(
            joint.findall(tag)
        ):
            joint.remove(element)


def lock_arm_for_stage2(robot):

    joints = [

        "joint1_to_base",
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",

        "gripper_controller",

        "gripper_base_to_gripper_left2",
        "gripper_left3_to_gripper_left1",

        "gripper_base_to_gripper_right3",
        "gripper_base_to_gripper_right2",
        "gripper_right3_to_gripper_right1",
    ]

    for joint_name in joints:

        make_joint_fixed(
            robot,
            joint_name
        )

def patch_elephant_urdf_for_gazebo(robot):

    # ========================================================
    # Fix oficial MechArm 270 M5:
    #
    # link2:
    #
    # visual    y = +0.21
    # collision y = -0.21
    #
    # Para Gazebo hacemos que la collision coincida exactamente
    # con el visual.
    # ========================================================

    link2 = robot.find(
        "./link[@name='link2']"
    )

    if link2 is None:
        raise RuntimeError(
            "No se encontró link2 del MechArm"
        )

    visual = link2.find("visual")
    collision = link2.find("collision")

    if visual is None or collision is None:
        raise RuntimeError(
            "link2 no contiene visual/collision"
        )

    visual_origin = visual.find("origin")
    collision_origin = collision.find("origin")

    if visual_origin is None or collision_origin is None:
        raise RuntimeError(
            "link2 no contiene los origins esperados"
        )

    # Copiamos literalmente la transformación del visual.
    collision_origin.set(
        "xyz",
        visual_origin.get("xyz")
    )

    collision_origin.set(
        "rpy",
        visual_origin.get("rpy")
    )

    print(
        "[PATCH] link2 collision alineada con visual:"
    )

    print(
        "        xyz =",
        collision_origin.get("xyz")
    )

    print(
        "        rpy =",
        collision_origin.get("rpy")
    )

# ============================================================
# Movimiento planar holonómico para Gazebo Classic
# ============================================================

def add_planar_mecanum_sim(
    robot
):

    gazebo = ET.SubElement(
        robot,
        "gazebo"
    )

    plugin = ET.SubElement(
        gazebo,
        "plugin",
        {
            "name":
                "myagv_mecanum_planar_move",

            "filename":
                "libgazebo_ros_planar_move.so",
        }
    )

    # ========================================================
    # ROS 2
    # ========================================================

    ros = ET.SubElement(
        plugin,
        "ros"
    )

    namespace = ET.SubElement(
        ros,
        "namespace"
    )

    namespace.text = "/"

    # --------------------------------------------------------
    # Entrada Twist
    # --------------------------------------------------------

    remap_cmd = ET.SubElement(
        ros,
        "remapping"
    )

    remap_cmd.text = (
        "cmd_vel:="
        "/mecanum_drive_controller/"
        "reference_unstamped"
    )

    # --------------------------------------------------------
    # Odometría de ground truth
    # --------------------------------------------------------

    remap_odom = ET.SubElement(
        ros,
        "remapping"
    )

    remap_odom.text = (
        "odom:=/odom"
    )

    # ========================================================
    # Frecuencia
    # ========================================================

    update_rate = ET.SubElement(
        plugin,
        "update_rate"
    )

    update_rate.text = "100.0"

    publish_rate = ET.SubElement(
        plugin,
        "publish_rate"
    )

    publish_rate.text = "30.0"

    # ========================================================
    # Publicar odometría REAL de Gazebo
    # ========================================================

    publish_odom = ET.SubElement(
        plugin,
        "publish_odom"
    )

    publish_odom.text = "true"

    publish_odom_tf = ET.SubElement(
        plugin,
        "publish_odom_tf"
    )

    publish_odom_tf.text = "true"

    # ========================================================
    # Frames
    # ========================================================

    odometry_frame = ET.SubElement(
        plugin,
        "odometry_frame"
    )

    odometry_frame.text = "odom"

    robot_base_frame = ET.SubElement(
        plugin,
        "robot_base_frame"
    )

    robot_base_frame.text = "base_footprint"

def add_ros2_control(robot):

    # ========================================================
    # ros2_control
    # ========================================================

    ros2_control = ET.SubElement(
        robot,
        "ros2_control",
        {
            "name": "GazeboSystem",
            "type": "system",
        }
    )

    hardware = ET.SubElement(
        ros2_control,
        "hardware"
    )

    plugin = ET.SubElement(
        hardware,
        "plugin"
    )

    plugin.text = (
        "gazebo_ros2_control/GazeboSystem"
    )

    # ========================================================
    # Cuatro ruedas
    # ========================================================

    wheel_joints = [
        "wheel_fl_joint",
        "wheel_fr_joint",
        "wheel_rl_joint",
        "wheel_rr_joint",
    ]

    # ========================================================
    # MechArm 270 M5
    # ========================================================

    for joint_name in ARM_JOINTS:

        joint = ET.SubElement(
            ros2_control,
            "joint",
            {
                "name": joint_name
            }
        )

        # JointTrajectoryController enviará posiciones.
        ET.SubElement(
            joint,
            "command_interface",
            {
                "name": "position"
            }
        )

        # Feedback de posición.
        ET.SubElement(
            joint,
            "state_interface",
            {
                "name": "position"
            }
        )

        # Feedback de velocidad.
        ET.SubElement(
            joint,
            "state_interface",
            {
                "name": "velocity"
            }
        )

    for joint_name in wheel_joints:

        joint = ET.SubElement(
            ros2_control,
            "joint",
            {
                "name": joint_name
            }
        )

        command = ET.SubElement(
            joint,
            "command_interface",
            {
                "name": "velocity"
            }
        )

        state_position = ET.SubElement(
            joint,
            "state_interface",
            {
                "name": "position"
            }
        )

        state_velocity = ET.SubElement(
            joint,
            "state_interface",
            {
                "name": "velocity"
            }
        )

    # ========================================================
    # Plugin Gazebo Classic
    # ========================================================

    sim_share = get_package_share_directory(
        "mobile_manipulator_sim"
    )

    controllers_file = os.path.join(
        sim_share,
        "config",
        "controllers.yaml"
    )

    gazebo = ET.SubElement(
        robot,
        "gazebo"
    )

    gazebo_plugin = ET.SubElement(
        gazebo,
        "plugin",
        {
            "name": "gazebo_ros2_control",
            "filename":
                "libgazebo_ros2_control.so",
        }
    )

    hold_joints = ET.SubElement(
        gazebo_plugin,
        "hold_joints"
    )

    hold_joints.text = "true"

    robot_param = ET.SubElement(
        gazebo_plugin,
        "robot_param"
    )

    robot_param.text = (
        "robot_description"
    )

    robot_param_node = ET.SubElement(
        gazebo_plugin,
        "robot_param_node"
    )

    robot_param_node.text = (
        "robot_state_publisher"
    )

    hold_joints = ET.SubElement(
        gazebo_plugin,
        "hold_joints"
    )

    hold_joints.text = "true"

    parameters = ET.SubElement(
        gazebo_plugin,
        "parameters"
    )

    parameters.text = controllers_file

def lock_gripper_for_stage4(robot):
    for joint in robot.findall("./joint"):
        joint_name = joint.get("name", "")

        if "gripper" not in joint_name.lower():
            continue

        joint.set(
            "type",
            "fixed"
        )

        # Elementos incompatibles / innecesarios en un fixed joint.
        for tag in [
            "axis",
            "limit",
            "dynamics",
            "mimic",
            "safety_controller",
            "calibration",
        ]:

            for element in list(
                joint.findall(tag)
            ):
                joint.remove(element)

        print(
            f"[STAGE4] Gripper bloqueado: "
            f"{joint_name}"
        )

# ============================================================
# LiDAR 2D simulado
# ============================================================

def add_sim_lidar(robot):

    # --------------------------------------------------------
    # Link del LiDAR
    # --------------------------------------------------------

    lidar_link = ET.SubElement(
        robot,
        "link",
        {
            "name": "lidar_link"
        }
    )

    # --------------------------------------------------------
    # Joint fijo:
    # base_link -> lidar_link
    #
    # Posición inicial de simulación.
    # La podremos ajustar visualmente después.
    # --------------------------------------------------------

    lidar_joint = ET.SubElement(
        robot,
        "joint",
        {
            "name": "lidar_joint",
            "type": "fixed"
        }
    )

    ET.SubElement(
        lidar_joint,
        "parent",
        {
            "link": "base_link"
        }
    )

    ET.SubElement(
        lidar_joint,
        "child",
        {
            "link": "lidar_link"
        }
    )

    ET.SubElement(
        lidar_joint,
        "origin",
        {
            "xyz": "0.0 0.0 0.11",
            "rpy": "0 0 0"
        }
    )

    # --------------------------------------------------------
    # Visual simple
    # --------------------------------------------------------

    visual = ET.SubElement(
        lidar_link,
        "visual"
    )

    ET.SubElement(
        visual,
        "origin",
        {
            "xyz": "0 0 0",
            "rpy": "0 0 0"
        }
    )

    geometry = ET.SubElement(
        visual,
        "geometry"
    )

    ET.SubElement(
        geometry,
        "cylinder",
        {
            "radius": "0.025",
            "length": "0.020"
        }
    )

    # --------------------------------------------------------
    # Gazebo sensor
    # --------------------------------------------------------

    gazebo = ET.SubElement(
        robot,
        "gazebo",
        {
            "reference": "lidar_link"
        }
    )

    sensor = ET.SubElement(
        gazebo,
        "sensor",
        {
            "name": "lidar_sensor",
            "type": "ray"
        }
    )

    always_on = ET.SubElement(
        sensor,
        "always_on"
    )
    always_on.text = "true"

    visualize = ET.SubElement(
        sensor,
        "visualize"
    )
    visualize.text = "true"

    update_rate = ET.SubElement(
        sensor,
        "update_rate"
    )
    update_rate.text = "10.0"

    # --------------------------------------------------------
    # Ray
    # --------------------------------------------------------

    ray = ET.SubElement(
        sensor,
        "ray"
    )

    scan = ET.SubElement(
        ray,
        "scan"
    )

    horizontal = ET.SubElement(
        scan,
        "horizontal"
    )

    samples = ET.SubElement(
        horizontal,
        "samples"
    )
    samples.text = "720"

    resolution = ET.SubElement(
        horizontal,
        "resolution"
    )
    resolution.text = "1"

    min_angle = ET.SubElement(
        horizontal,
        "min_angle"
    )
    min_angle.text = str(
        -math.pi
    )

    max_angle = ET.SubElement(
        horizontal,
        "max_angle"
    )
    max_angle.text = str(
        math.pi
    )

    # --------------------------------------------------------
    # Rango
    # --------------------------------------------------------

    range_element = ET.SubElement(
        ray,
        "range"
    )

    minimum = ET.SubElement(
        range_element,
        "min"
    )
    minimum.text = "0.08"

    maximum = ET.SubElement(
        range_element,
        "max"
    )
    maximum.text = "8.0"

    range_resolution = ET.SubElement(
        range_element,
        "resolution"
    )
    range_resolution.text = "0.01"

    # --------------------------------------------------------
    # Ruido
    # --------------------------------------------------------

    noise = ET.SubElement(
        ray,
        "noise"
    )

    noise_type = ET.SubElement(
        noise,
        "type"
    )
    noise_type.text = "gaussian"

    noise_mean = ET.SubElement(
        noise,
        "mean"
    )
    noise_mean.text = "0.0"

    noise_stddev = ET.SubElement(
        noise,
        "stddev"
    )
    noise_stddev.text = "0.002"

    # --------------------------------------------------------
    # Plugin ROS
    # --------------------------------------------------------

    plugin = ET.SubElement(
        sensor,
        "plugin",
        {
            "name": "lidar_ros_plugin",
            "filename": "libgazebo_ros_ray_sensor.so"
        }
    )

    ros = ET.SubElement(
        plugin,
        "ros"
    )

    namespace = ET.SubElement(
        ros,
        "namespace"
    )
    namespace.text = "/"

    remapping = ET.SubElement(
        ros,
        "remapping"
    )

    remapping.text = "~/out:=/scan"

    output_type = ET.SubElement(
        plugin,
        "output_type"
    )
    output_type.text = "sensor_msgs/LaserScan"

    frame_name = ET.SubElement(
        plugin,
        "frame_name"
    )
    frame_name.text = "lidar_link"

def add_sim_camera(robot):

    print(
        "[CAMERA] Añadiendo cámara frontal: "
        f"x={CAMERA_X:.3f}, "
        f"y={CAMERA_Y:.3f}, "
        f"z={CAMERA_Z:.3f}"
    )

    # ========================================================
    # camera_link
    # ========================================================

    camera_link = ET.SubElement(
        robot,
        "link",
        {
            "name": "camera_link"
        }
    )

    # No añadimos visual ni collision:
    # la cámara física ya forma parte del mesh visual del myAGV.

    # ========================================================
    # base_link -> camera_link
    # ========================================================

    camera_joint = ET.SubElement(
        robot,
        "joint",
        {
            "name": "camera_joint",
            "type": "fixed",
        }
    )

    ET.SubElement(
        camera_joint,
        "parent",
        {
            "link": "base_link"
        }
    )

    ET.SubElement(
        camera_joint,
        "child",
        {
            "link": "camera_link"
        }
    )

    ET.SubElement(
        camera_joint,
        "origin",
        {
            "xyz":
                f"{CAMERA_X} "
                f"{CAMERA_Y} "
                f"{CAMERA_Z}",

            "rpy":
                f"{CAMERA_ROLL} "
                f"{CAMERA_PITCH} "
                f"{CAMERA_YAW}",
        }
    )

    # ========================================================
    # camera_optical_frame
    # ========================================================

    ET.SubElement(
        robot,
        "link",
        {
            "name": "camera_optical_frame"
        }
    )

    optical_joint = ET.SubElement(
        robot,
        "joint",
        {
            "name": "camera_optical_joint",
            "type": "fixed",
        }
    )

    ET.SubElement(
        optical_joint,
        "parent",
        {
            "link": "camera_link"
        }
    )

    ET.SubElement(
        optical_joint,
        "child",
        {
            "link": "camera_optical_frame"
        }
    )

    ET.SubElement(
        optical_joint,
        "origin",
        {
            "xyz": "0 0 0",
            "rpy": "-1.57079632679 0 -1.57079632679",
        }
    )

    # ========================================================
    # Gazebo camera sensor
    # ========================================================

    gazebo = ET.SubElement(
        robot,
        "gazebo",
        {
            "reference": "camera_link"
        }
    )

    sensor = ET.SubElement(
        gazebo,
        "sensor",
        {
            "name": "front_camera_sensor",
            "type": "camera",
        }
    )

    always_on = ET.SubElement(
        sensor,
        "always_on"
    )
    always_on.text = "true"

    update_rate = ET.SubElement(
        sensor,
        "update_rate"
    )
    update_rate.text = str(
        CAMERA_UPDATE_RATE
    )

    visualize = ET.SubElement(
        sensor,
        "visualize"
    )
    visualize.text = "true"

    # El sensor está exactamente en camera_link.
    pose = ET.SubElement(
        sensor,
        "pose"
    )
    pose.text = "0 0 0 0 0 0"

    # ========================================================
    # Parámetros ópticos
    # ========================================================

    camera = ET.SubElement(
        sensor,
        "camera",
        {
            "name": "front_camera"
        }
    )

    horizontal_fov = ET.SubElement(
        camera,
        "horizontal_fov"
    )
    horizontal_fov.text = str(
        CAMERA_HFOV
    )

    image = ET.SubElement(
        camera,
        "image"
    )

    width = ET.SubElement(
        image,
        "width"
    )
    width.text = str(
        CAMERA_WIDTH
    )

    height = ET.SubElement(
        image,
        "height"
    )
    height.text = str(
        CAMERA_HEIGHT
    )

    image_format = ET.SubElement(
        image,
        "format"
    )
    image_format.text = "R8G8B8"

    # ========================================================
    # Distancias visibles
    # ========================================================

    clip = ET.SubElement(
        camera,
        "clip"
    )

    near = ET.SubElement(
        clip,
        "near"
    )
    near.text = "0.02"

    far = ET.SubElement(
        clip,
        "far"
    )
    far.text = "20.0"

    # ========================================================
    # Ruido
    # ========================================================

    noise = ET.SubElement(
        camera,
        "noise"
    )

    noise_type = ET.SubElement(
        noise,
        "type"
    )
    noise_type.text = "gaussian"

    noise_mean = ET.SubElement(
        noise,
        "mean"
    )
    noise_mean.text = "0.0"

    noise_stddev = ET.SubElement(
        noise,
        "stddev"
    )

    # Primero sin ruido apreciable.
    # Después podremos aumentarlo para probar robustez ArUco.
    noise_stddev.text = "0.001"

    # ========================================================
    # gazebo_ros_camera
    # ========================================================

    plugin = ET.SubElement(
        sensor,
        "plugin",
        {
            "name": "front_camera_controller",
            "filename": "libgazebo_ros_camera.so",
        }
    )

    ros = ET.SubElement(
        plugin,
        "ros"
    )

    namespace = ET.SubElement(
        ros,
        "namespace"
    )
    namespace.text = "/"

    camera_name = ET.SubElement(
        plugin,
        "camera_name"
    )

    # El plugin ROS2 construye:
    #
    # camera_name/image_raw
    # camera_name/camera_info
    #
    camera_name.text = "camera"

    frame_name = ET.SubElement(
        plugin,
        "frame_name"
    )
    frame_name.text = "camera_optical_frame"

    print(
        "[CAMERA] Topics esperados: "
        "/camera/image_raw, "
        "/camera/camera_info"
    )

if __name__ == "__main__":

    print(
        build_robot_description()
    )
