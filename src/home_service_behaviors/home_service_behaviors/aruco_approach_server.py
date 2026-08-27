import math
import threading
import time

import rclpy

from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf2_ros import Buffer
from tf2_ros import TransformListener

from home_service_interfaces.action import ArucoApproach
from home_service_interfaces.msg import ArucoDetectionArray


# ============================================================
# Utilidades
# ============================================================

def clamp(value, minimum, maximum):
    return max(
        minimum,
        min(maximum, value)
    )


def normalize_angle(angle):

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def quaternion_to_yaw(x, y, z, w):

    siny_cosp = 2.0 * (
        w * z
        +
        x * y
    )

    cosy_cosp = 1.0 - 2.0 * (
        y * y
        +
        z * z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


def marker_normal_from_quaternion(
    x,
    y,
    z,
    w
):
    """
    Devuelve el eje +Z local del ArUco expresado
    en el frame padre.

    En OpenCV/ArUco, Z es normal al plano del marcador.
    """

    nx = 2.0 * (
        x * z
        +
        w * y
    )

    ny = 2.0 * (
        y * z
        -
        w * x
    )

    nz = 1.0 - 2.0 * (
        x * x
        +
        y * y
    )

    return (
        nx,
        ny,
        nz
    )


# ============================================================
# Action Server
# ============================================================

class ArucoApproachServer(Node):

    def __init__(self):

        super().__init__(
            "aruco_approach_server"
        )

        self.callback_group = ReentrantCallbackGroup()

        # ====================================================
        # Topics
        # ====================================================

        self.declare_parameter(
            "detections_topic",
            "/aruco/detections"
        )

        self.declare_parameter(
            "cmd_vel_topic",
            "/mecanum_drive_controller/reference_unstamped"
        )

        self.declare_parameter(
            "odom_topic",
            "/odom"
        )

        # ====================================================
        # Cámara respecto a base_link
        # ====================================================

        self.declare_parameter(
            "camera_x_offset",
            0.160
        )

        self.declare_parameter(
            "camera_y_offset",
            -0.004
        )

        # ====================================================
        # Control general
        # ====================================================

        self.declare_parameter(
            "control_rate",
            20.0
        )

        # Edad máxima permitida de una detección.
        # NO limita el tiempo de búsqueda.
        self.declare_parameter(
            "detection_timeout",
            0.35
        )

        # ====================================================
        # SEARCHING
        # ====================================================

        self.declare_parameter(
            "search_angular_speed",
            0.15
        )

        # ====================================================
        # LOCK_TARGET
        # ====================================================

        self.declare_parameter(
            "target_lock_sec",
            0.50
        )

        self.declare_parameter(
            "target_lock_min_samples",
            5
        )

        # ====================================================
        # ALIGN_HEADING_TO_ARUCO
        # ====================================================

        self.declare_parameter(
            "kp_heading",
            1.5
        )

        self.declare_parameter(
            "max_heading_speed",
            0.30
        )

        # ~1.1 grados
        self.declare_parameter(
            "heading_tolerance",
            0.02
        )

        # Si durante la traslación se desvía más que esto,
        # volvemos a corregir heading.
        self.declare_parameter(
            "heading_realign_threshold",
            0.04
        )

        # ====================================================
        # ALIGNING_LATERAL
        # ====================================================

        self.declare_parameter(
            "kp_lateral",
            0.8
        )

        self.declare_parameter(
            "max_lateral_speed",
            0.04
        )

        # 1 cm
        self.declare_parameter(
            "lateral_tolerance",
            0.01
        )

        # Durante APPROACHING, si vuelve a desviarse
        # lateralmente más de 2 cm, hacemos otra corrección Y.
        self.declare_parameter(
            "lateral_realign_threshold",
            0.02
        )

        # ====================================================
        # APPROACHING
        # ====================================================

        self.declare_parameter(
            "kp_linear",
            0.5
        )

        self.declare_parameter(
            "max_linear_speed",
            0.05
        )

        self.declare_parameter(
            "max_reverse_speed",
            0.02
        )

        # 1 cm
        self.declare_parameter(
            "distance_tolerance",
            0.01
        )

        # ====================================================
        # Leer parámetros
        # ====================================================

        self.detections_topic = (
            self.get_parameter(
                "detections_topic"
            ).value
        )

        self.cmd_vel_topic = (
            self.get_parameter(
                "cmd_vel_topic"
            ).value
        )

        self.odom_topic = (
            self.get_parameter(
                "odom_topic"
            ).value
        )

        self.camera_x_offset = float(
            self.get_parameter(
                "camera_x_offset"
            ).value
        )

        self.camera_y_offset = float(
            self.get_parameter(
                "camera_y_offset"
            ).value
        )

        self.control_rate = float(
            self.get_parameter(
                "control_rate"
            ).value
        )

        self.detection_timeout = float(
            self.get_parameter(
                "detection_timeout"
            ).value
        )

        self.search_angular_speed = float(
            self.get_parameter(
                "search_angular_speed"
            ).value
        )

        self.target_lock_sec = float(
            self.get_parameter(
                "target_lock_sec"
            ).value
        )

        self.target_lock_min_samples = int(
            self.get_parameter(
                "target_lock_min_samples"
            ).value
        )

        self.kp_heading = float(
            self.get_parameter(
                "kp_heading"
            ).value
        )

        self.max_heading_speed = float(
            self.get_parameter(
                "max_heading_speed"
            ).value
        )

        self.heading_tolerance = float(
            self.get_parameter(
                "heading_tolerance"
            ).value
        )

        self.heading_realign_threshold = float(
            self.get_parameter(
                "heading_realign_threshold"
            ).value
        )

        self.kp_lateral = float(
            self.get_parameter(
                "kp_lateral"
            ).value
        )

        self.max_lateral_speed = float(
            self.get_parameter(
                "max_lateral_speed"
            ).value
        )

        self.lateral_tolerance = float(
            self.get_parameter(
                "lateral_tolerance"
            ).value
        )

        self.lateral_realign_threshold = float(
            self.get_parameter(
                "lateral_realign_threshold"
            ).value
        )

        self.kp_linear = float(
            self.get_parameter(
                "kp_linear"
            ).value
        )

        self.max_linear_speed = float(
            self.get_parameter(
                "max_linear_speed"
            ).value
        )

        self.max_reverse_speed = float(
            self.get_parameter(
                "max_reverse_speed"
            ).value
        )

        self.distance_tolerance = float(
            self.get_parameter(
                "distance_tolerance"
            ).value
        )

        # ====================================================
        # Detecciones
        # ====================================================

        self.latest_detections = {}
        self.detection_lock = threading.Lock()

        # ====================================================
        # Odometría
        # ====================================================

        # (x, y, yaw)
        self.latest_odom = None
        self.odom_lock = threading.Lock()

        # ====================================================
        # TF
        # ====================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False
        )

        # ====================================================
        # Action
        # ====================================================

        self.goal_lock = threading.Lock()
        self.goal_active = False

        # ====================================================
        # ROS
        # ====================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.detection_sub = self.create_subscription(
            ArucoDetectionArray,
            self.detections_topic,
            self.detection_callback,
            10,
            callback_group=self.callback_group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        self.action_server = ActionServer(
            self,
            ArucoApproach,
            "aruco_approach",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            "ArUco Approach Server iniciado"
        )

        self.get_logger().info(
            "Automata: SEARCHING -> LOCK_TARGET -> "
            "ALIGN_HEADING_TO_ARUCO -> "
            "ALIGNING_LATERAL -> APPROACHING -> REACHED"
        )

    # ========================================================
    # Tiempo
    # ========================================================

    def now_seconds(self):

        return (
            self.get_clock()
            .now()
            .nanoseconds
            *
            1e-9
        )

    # ========================================================
    # Detecciones
    # ========================================================

    def detection_callback(
        self,
        msg
    ):

        now = self.now_seconds()

        with self.detection_lock:

            for detection in msg.detections:

                self.latest_detections[
                    int(detection.id)
                ] = {

                    "distance":
                        float(
                            detection.distance_z
                        ),

                    "center":
                        float(
                            detection.center_x_normalized
                        ),

                    "time":
                        now,
                }

    def get_detection(
        self,
        marker_id
    ):

        with self.detection_lock:

            result = (
                self.latest_detections.get(
                    marker_id
                )
            )

            if result is None:
                return None

            return result.copy()

    # ========================================================
    # Odom
    # ========================================================

    def odom_callback(
        self,
        msg
    ):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        with self.odom_lock:

            self.latest_odom = (
                float(p.x),
                float(p.y),
                float(yaw)
            )

    def get_odom(self):

        with self.odom_lock:

            if self.latest_odom is None:
                return None

            return tuple(
                self.latest_odom
            )

    # ========================================================
    # TF del ArUco
    # ========================================================

    def get_marker_tf(
        self,
        marker_id
    ):

        frame = (
            f"aruco_{marker_id}"
        )

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    "odom",
                    frame,
                    rclpy.time.Time()
                )
            )

            return transform

        except Exception:
            return None

    # ========================================================
    # Convertir TF del ArUco a:
    #
    # posición XY
    # +
    # vector perpendicular F
    # ========================================================

    def marker_sample_from_tf(
        self,
        transform,
        robot_pose
    ):

        t = transform.transform.translation
        q = transform.transform.rotation

        marker_x = float(
            t.x
        )

        marker_y = float(
            t.y
        )

        # Eje Z del marcador = normal al plano.
        nx, ny, _ = (
            marker_normal_from_quaternion(
                q.x,
                q.y,
                q.z,
                q.w
            )
        )

        norm = math.sqrt(
            nx * nx
            +
            ny * ny
        )

        # El marcador debe ser aproximadamente vertical,
        # por lo tanto su normal debe tener componente XY.
        if norm < 1e-6:
            return None

        nx /= norm
        ny /= norm

        # ----------------------------------------------------
        # La normal puede apuntar hacia cualquiera de los dos
        # lados del plano.
        #
        # Queremos F apuntando:
        #
        # robot -> ArUco
        # ----------------------------------------------------

        robot_to_marker_x = (
            marker_x
            -
            robot_pose[0]
        )

        robot_to_marker_y = (
            marker_y
            -
            robot_pose[1]
        )

        dot = (
            nx * robot_to_marker_x
            +
            ny * robot_to_marker_y
        )

        if dot < 0.0:
            nx = -nx
            ny = -ny

        return (
            marker_x,
            marker_y,
            nx,
            ny
        )

    # ========================================================
    # Calcular punto final del base_link
    # ========================================================

    def calculate_desired_base_position(
        self,
        marker_x,
        marker_y,
        forward_x,
        forward_y,
        stop_distance
    ):

        # Eje lateral del robot.
        #
        # Si F es +X del robot,
        # L es +Y del robot.
        lateral_x = (
            -forward_y
        )

        lateral_y = (
            forward_x
        )

        # Queremos la cámara a stop_distance delante
        # del ArUco.
        #
        # desired_camera =
        # marker - F * stop_distance
        #
        # desired_base =
        # desired_camera
        # - F * camera_x_offset
        # - L * camera_y_offset

        desired_x = (
            marker_x
            -
            forward_x
            *
            (
                stop_distance
                +
                self.camera_x_offset
            )
            -
            lateral_x
            *
            self.camera_y_offset
        )

        desired_y = (
            marker_y
            -
            forward_y
            *
            (
                stop_distance
                +
                self.camera_x_offset
            )
            -
            lateral_y
            *
            self.camera_y_offset
        )

        return (
            desired_x,
            desired_y
        )

    # ========================================================
    # Goal
    # ========================================================

    def goal_callback(
        self,
        goal_request
    ):

        if (
            goal_request.target_id < 0
            or
            goal_request.target_id > 249
        ):

            return GoalResponse.REJECT

        if goal_request.stop_distance <= 0.0:
            return GoalResponse.REJECT

        with self.goal_lock:

            if self.goal_active:
                return GoalResponse.REJECT

            self.goal_active = True

        return GoalResponse.ACCEPT

    # ========================================================
    # Cancel
    # ========================================================

    def cancel_callback(
        self,
        goal_handle
    ):

        return CancelResponse.ACCEPT

    # ========================================================
    # Movimiento
    # ========================================================

    def publish_command(
        self,
        linear_x=0.0,
        linear_y=0.0,
        angular_z=0.0
    ):

        msg = Twist()

        msg.linear.x = float(
            linear_x
        )

        msg.linear.y = float(
            linear_y
        )

        msg.angular.z = float(
            angular_z
        )

        self.cmd_pub.publish(
            msg
        )

    def stop_robot(self):

        self.publish_command(
            0.0,
            0.0,
            0.0
        )

    # ========================================================
    # Feedback
    # ========================================================

    def publish_feedback(
        self,
        goal_handle,
        state,
        distance,
        error,
        elapsed
    ):

        feedback = (
            ArucoApproach.Feedback()
        )

        feedback.state = str(
            state
        )

        feedback.distance = float(
            distance
        )

        feedback.center_error = float(
            error
        )

        feedback.elapsed_sec = float(
            elapsed
        )

        goal_handle.publish_feedback(
            feedback
        )

    # ========================================================
    # Result
    # ========================================================

    def make_result(
        self,
        success,
        status,
        message,
        final_distance
    ):

        result = (
            ArucoApproach.Result()
        )

        result.success = bool(
            success
        )

        result.status = str(
            status
        )

        result.message = str(
            message
        )

        result.final_distance = float(
            final_distance
        )

        return result

    # ========================================================
    # Execute
    # ========================================================

    def execute_callback(
        self,
        goal_handle
    ):

        target_id = int(
            goal_handle.request.target_id
        )

        stop_distance = float(
            goal_handle.request.stop_distance
        )

        if (
            goal_handle.request.timeout_sec
            >
            0.0
        ):

            goal_timeout = float(
                goal_handle.request.timeout_sec
            )

        else:

            goal_timeout = None

        period = (
            1.0
            /
            self.control_rate
        )

        start_time = (
            self.now_seconds()
        )

        state = "WAITING_ODOM"

        # ====================================================
        # Información fijada del ArUco
        # ====================================================

        marker_x = None
        marker_y = None

        forward_x = None
        forward_y = None

        lateral_x = None
        lateral_y = None

        target_yaw = None

        desired_x = None
        desired_y = None

        # ====================================================
        # LOCK samples
        # ====================================================

        lock_samples = []
        lock_start = None

        last_camera_distance = -1.0

        try:

            while rclpy.ok():

                now = (
                    self.now_seconds()
                )

                elapsed = (
                    now
                    -
                    start_time
                )

                # ============================================
                # CANCEL
                # ============================================

                if (
                    goal_handle
                    .is_cancel_requested
                ):

                    self.stop_robot()

                    goal_handle.canceled()

                    return self.make_result(
                        False,
                        "CANCELED",
                        "Goal cancelado",
                        last_camera_distance
                    )

                # ============================================
                # TIMEOUT OPCIONAL
                # ============================================

                if (
                    goal_timeout is not None
                    and
                    elapsed >= goal_timeout
                ):

                    self.stop_robot()

                    goal_handle.abort()

                    return self.make_result(
                        False,
                        "TIMEOUT",
                        "Tiempo limite alcanzado",
                        last_camera_distance
                    )

                # ============================================
                # ODOM
                # ============================================

                robot_pose = (
                    self.get_odom()
                )

                if robot_pose is None:

                    self.stop_robot()

                    self.publish_feedback(
                        goal_handle,
                        "WAITING_ODOM",
                        -1.0,
                        0.0,
                        elapsed
                    )

                    time.sleep(
                        period
                    )

                    continue

                if state == "WAITING_ODOM":

                    state = "SEARCHING"

                    self.get_logger().info(
                        f"Buscando ArUco {target_id}"
                    )

                # ============================================
                # SEARCHING
                # ============================================

                if state == "SEARCHING":

                    detection = (
                        self.get_detection(
                            target_id
                        )
                    )

                    fresh = False

                    if detection is not None:

                        age = (
                            now
                            -
                            detection["time"]
                        )

                        fresh = (
                            age
                            <=
                            self.detection_timeout
                        )

                    if not fresh:

                        self.publish_command(
                            linear_x=0.0,
                            linear_y=0.0,
                            angular_z=(
                                self.search_angular_speed
                            )
                        )

                        self.publish_feedback(
                            goal_handle,
                            "SEARCHING",
                            last_camera_distance,
                            0.0,
                            elapsed
                        )

                        time.sleep(
                            period
                        )

                        continue

                    # Marker encontrado.
                    self.stop_robot()

                    last_camera_distance = (
                        detection["distance"]
                    )

                    state = "LOCK_TARGET"

                    lock_samples = []

                    lock_start = now

                    self.get_logger().info(
                        f"ArUco {target_id} encontrado"
                    )

                    self.get_logger().info(
                        "Estado -> LOCK_TARGET"
                    )

                    time.sleep(
                        period
                    )

                    continue

                # ============================================
                # LOCK_TARGET
                # ============================================

                if state == "LOCK_TARGET":

                    self.stop_robot()

                    detection = (
                        self.get_detection(
                            target_id
                        )
                    )

                    fresh = False

                    if detection is not None:

                        age = (
                            now
                            -
                            detection["time"]
                        )

                        fresh = (
                            age
                            <=
                            self.detection_timeout
                        )

                    if not fresh:

                        self.get_logger().info(
                            "Target perdido durante LOCK. "
                            "Volviendo a SEARCHING"
                        )

                        state = "SEARCHING"

                        lock_samples = []

                        time.sleep(
                            period
                        )

                        continue

                    last_camera_distance = (
                        detection["distance"]
                    )

                    marker_tf = (
                        self.get_marker_tf(
                            target_id
                        )
                    )

                    if marker_tf is not None:

                        sample = (
                            self.marker_sample_from_tf(
                                marker_tf,
                                robot_pose
                            )
                        )

                        if sample is not None:

                            lock_samples.append(
                                sample
                            )

                    self.publish_feedback(
                        goal_handle,
                        "LOCK_TARGET",
                        last_camera_distance,
                        (
                            detection["center"]
                            -
                            0.5
                        ),
                        elapsed
                    )

                    lock_elapsed = (
                        now
                        -
                        lock_start
                    )

                    if (
                        lock_elapsed
                        <
                        self.target_lock_sec
                        or
                        len(lock_samples)
                        <
                        self.target_lock_min_samples
                    ):

                        time.sleep(
                            period
                        )

                        continue

                    # ========================================
                    # Promediar posición
                    # ========================================

                    marker_x = (
                        sum(
                            s[0]
                            for s in lock_samples
                        )
                        /
                        len(lock_samples)
                    )

                    marker_y = (
                        sum(
                            s[1]
                            for s in lock_samples
                        )
                        /
                        len(lock_samples)
                    )

                    # ========================================
                    # Promediar dirección perpendicular
                    # ========================================

                    avg_fx = (
                        sum(
                            s[2]
                            for s in lock_samples
                        )
                        /
                        len(lock_samples)
                    )

                    avg_fy = (
                        sum(
                            s[3]
                            for s in lock_samples
                        )
                        /
                        len(lock_samples)
                    )

                    norm = math.sqrt(
                        avg_fx * avg_fx
                        +
                        avg_fy * avg_fy
                    )

                    if norm < 1e-6:

                        self.get_logger().warn(
                            "No se pudo calcular normal "
                            "del ArUco. Reintentando busqueda."
                        )

                        state = "SEARCHING"

                        time.sleep(
                            period
                        )

                        continue

                    forward_x = (
                        avg_fx
                        /
                        norm
                    )

                    forward_y = (
                        avg_fy
                        /
                        norm
                    )

                    # +Y local cuando +X local es F.
                    lateral_x = (
                        -forward_y
                    )

                    lateral_y = (
                        forward_x
                    )

                    target_yaw = (
                        math.atan2(
                            forward_y,
                            forward_x
                        )
                    )

                    desired_x, desired_y = (
                        self.calculate_desired_base_position(
                            marker_x,
                            marker_y,
                            forward_x,
                            forward_y,
                            stop_distance
                        )
                    )

                    self.get_logger().info(
                        "Target bloqueado:"
                    )

                    self.get_logger().info(
                        f"marker = "
                        f"({marker_x:.3f}, "
                        f"{marker_y:.3f})"
                    )

                    self.get_logger().info(
                        f"normal F = "
                        f"({forward_x:.3f}, "
                        f"{forward_y:.3f})"
                    )

                    self.get_logger().info(
                        f"target yaw = "
                        f"{math.degrees(target_yaw):.2f} deg"
                    )

                    self.get_logger().info(
                        f"desired base = "
                        f"({desired_x:.3f}, "
                        f"{desired_y:.3f})"
                    )

                    state = (
                        "ALIGN_HEADING_TO_ARUCO"
                    )

                    self.get_logger().info(
                        "Estado -> "
                        "ALIGN_HEADING_TO_ARUCO"
                    )

                    time.sleep(
                        period
                    )

                    continue

                # ============================================
                # Errores geométricos en frame del ArUco
                # ============================================

                robot_pose = (
                    self.get_odom()
                )

                dx = (
                    desired_x
                    -
                    robot_pose[0]
                )

                dy = (
                    desired_y
                    -
                    robot_pose[1]
                )

                forward_error = (
                    forward_x * dx
                    +
                    forward_y * dy
                )

                lateral_error = (
                    lateral_x * dx
                    +
                    lateral_y * dy
                )

                heading_error = (
                    normalize_angle(
                        target_yaw
                        -
                        robot_pose[2]
                    )
                )

                # ============================================
                # ALIGN_HEADING_TO_ARUCO
                # ============================================

                if (
                    state
                    ==
                    "ALIGN_HEADING_TO_ARUCO"
                ):

                    if (
                        abs(heading_error)
                        <=
                        self.heading_tolerance
                    ):

                        self.stop_robot()

                        state = (
                            "ALIGNING_LATERAL"
                        )

                        self.get_logger().info(
                            "Heading perpendicular "
                            "al ArUco alcanzado"
                        )

                        self.get_logger().info(
                            "Estado -> ALIGNING_LATERAL"
                        )

                        time.sleep(
                            period
                        )

                        continue

                    angular_speed = (
                        self.kp_heading
                        *
                        heading_error
                    )

                    angular_speed = clamp(
                        angular_speed,
                        -self.max_heading_speed,
                        self.max_heading_speed
                    )

                    self.publish_command(
                        linear_x=0.0,
                        linear_y=0.0,
                        angular_z=angular_speed
                    )

                    self.publish_feedback(
                        goal_handle,
                        "ALIGN_HEADING_TO_ARUCO",
                        last_camera_distance,
                        heading_error,
                        elapsed
                    )

                    time.sleep(
                        period
                    )

                    continue

                # ============================================
                # ALIGNING_LATERAL
                # ============================================

                if (
                    state
                    ==
                    "ALIGNING_LATERAL"
                ):

                    # Heading se desvió.
                    if (
                        abs(heading_error)
                        >
                        self.heading_realign_threshold
                    ):

                        self.stop_robot()

                        state = (
                            "ALIGN_HEADING_TO_ARUCO"
                        )

                        time.sleep(
                            period
                        )

                        continue

                    # Ya alineado lateralmente.
                    if (
                        abs(lateral_error)
                        <=
                        self.lateral_tolerance
                    ):

                        self.stop_robot()

                        state = (
                            "APPROACHING"
                        )

                        self.get_logger().info(
                            "Alineacion lateral completada"
                        )

                        self.get_logger().info(
                            "Estado -> APPROACHING"
                        )

                        time.sleep(
                            period
                        )

                        continue

                    # SOLO Y.
                    lateral_speed = (
                        self.kp_lateral
                        *
                        lateral_error
                    )

                    lateral_speed = clamp(
                        lateral_speed,
                        -self.max_lateral_speed,
                        self.max_lateral_speed
                    )

                    self.publish_command(
                        linear_x=0.0,
                        linear_y=lateral_speed,
                        angular_z=0.0
                    )

                    self.publish_feedback(
                        goal_handle,
                        "ALIGNING_LATERAL",
                        forward_error,
                        lateral_error,
                        elapsed
                    )

                    time.sleep(
                        period
                    )

                    continue

                # ============================================
                # APPROACHING
                # ============================================

                if state == "APPROACHING":

                    # Heading se desvió.
                    if (
                        abs(heading_error)
                        >
                        self.heading_realign_threshold
                    ):

                        self.stop_robot()

                        state = (
                            "ALIGN_HEADING_TO_ARUCO"
                        )

                        time.sleep(
                            period
                        )

                        continue

                    # Desalineación lateral.
                    if (
                        abs(lateral_error)
                        >
                        self.lateral_realign_threshold
                    ):

                        self.stop_robot()

                        state = (
                            "ALIGNING_LATERAL"
                        )

                        self.get_logger().info(
                            "Correccion lateral necesaria"
                        )

                        time.sleep(
                            period
                        )

                        continue

                    # ========================================
                    # REACHED
                    # ========================================

                    if (
                        abs(forward_error)
                        <=
                        self.distance_tolerance
                        and
                        abs(lateral_error)
                        <=
                        self.lateral_tolerance
                    ):

                        self.stop_robot()

                        goal_handle.succeed()

                        message = (
                            f"ArUco {target_id} alcanzado "
                            "en pose objetivo"
                        )

                        self.get_logger().info(
                            message
                        )

                        return self.make_result(
                            True,
                            "SUCCEEDED",
                            message,
                            stop_distance
                        )

                    # SOLO X.
                    linear_speed = (
                        self.kp_linear
                        *
                        forward_error
                    )

                    linear_speed = clamp(
                        linear_speed,
                        -self.max_reverse_speed,
                        self.max_linear_speed
                    )

                    self.publish_command(
                        linear_x=linear_speed,
                        linear_y=0.0,
                        angular_z=0.0
                    )

                    self.publish_feedback(
                        goal_handle,
                        "APPROACHING",
                        forward_error,
                        lateral_error,
                        elapsed
                    )

                    time.sleep(
                        period
                    )

                    continue

                time.sleep(
                    period
                )

        finally:

            self.stop_robot()

            with self.goal_lock:
                self.goal_active = False


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        ArucoApproachServer()
    )

    executor = (
        MultiThreadedExecutor(
            num_threads=4
        )
    )

    executor.add_node(
        node
    )

    try:

        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_robot()

        executor.shutdown()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()