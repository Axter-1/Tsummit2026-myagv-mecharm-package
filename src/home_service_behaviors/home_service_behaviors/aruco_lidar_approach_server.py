#!/usr/bin/env python3

import math
import time
import threading

import numpy as np

import rclpy

from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener

from home_service_interfaces.msg import ArucoDetectionArray
from home_service_interfaces.action import ArucoApproach


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (
        q.w * q.z +
        q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y +
        q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


class ArucoLidarApproachServer(Node):

    def __init__(self):

        super().__init__(
            'aruco_lidar_approach_server'
        )

        self.callback_group = (
            ReentrantCallbackGroup()
        )

        # =========================================================
        # Topics
        # =========================================================

        self.declare_parameter(
            'detections_topic',
            '/aruco/detections'
        )

        self.declare_parameter(
            'scan_topic',
            '/scan'
        )

        self.declare_parameter(
            'odom_topic',
            '/odom'
        )

        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel_aruco'
        )

        self.declare_parameter(
            'action_name',
            '/aruco_lidar_approach'
        )

        self.declare_parameter(
            'odom_frame',
            'odom'
        )

        # =========================================================
        # Search
        # =========================================================

        self.declare_parameter(
            'search_angular_speed',
            0.22
        )

        # =========================================================
        # Lock target normal
        # =========================================================

        self.declare_parameter(
            'lock_duration',
            0.50
        )

        self.declare_parameter(
            'lock_min_samples',
            5
        )

        # =========================================================
        # Heading
        # =========================================================

        self.declare_parameter(
            'kp_heading',
            1.5
        )

        self.declare_parameter(
            'max_heading_speed',
            0.30
        )

        self.declare_parameter(
            'heading_tolerance',
            0.02
        )

        self.declare_parameter(
            'heading_realign_threshold',
            0.05
        )

        # =========================================================
        # Lateral alignment
        # =========================================================

        self.declare_parameter(
            'kp_lateral',
            0.08
        )

        self.declare_parameter(
            'max_lateral_speed',
            0.04
        )

        self.declare_parameter(
            'lateral_tolerance',
            0.04
        )

        self.declare_parameter(
            'lateral_realign_threshold',
            0.15
        )

        # =========================================================
        # Forward movement
        # =========================================================

        self.declare_parameter(
            'kp_linear',
            0.5
        )

        self.declare_parameter(
            'max_linear_speed',
            0.06
        )

        self.declare_parameter(
            'distance_tolerance',
            0.01
        )

        # =========================================================
        # LiDAR geometry
        # =========================================================

        # camera_link is 16 cm ahead of lidar_link.
        self.declare_parameter(
            'camera_x_minus_lidar_x',
            0.16
        )

        self.declare_parameter(
            'lidar_sector_half_angle_deg',
            4.0
        )

        # =========================================================
        # Freshness
        # =========================================================

        self.declare_parameter(
            'detection_timeout',
            0.35
        )

        self.declare_parameter(
            'scan_timeout',
            0.35
        )

        self.declare_parameter(
            'control_rate',
            20.0
        )

        # =========================================================
        # State
        # =========================================================

        self.lock = threading.Lock()

        self.latest_detections = {}

        self.latest_scan = None
        self.latest_scan_time_ns = None

        self.latest_odom = None

        # =========================================================
        # TF
        # =========================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # =========================================================
        # Subscribers
        # =========================================================

        self.create_subscription(
            ArucoDetectionArray,
            self.get_parameter(
                'detections_topic'
            ).value,
            self.detections_callback,
            10,
            callback_group=self.callback_group
        )

        self.create_subscription(
            LaserScan,
            self.get_parameter(
                'scan_topic'
            ).value,
            self.scan_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )

        self.create_subscription(
            Odometry,
            self.get_parameter(
                'odom_topic'
            ).value,
            self.odom_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )

        # =========================================================
        # Velocity
        # =========================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter(
                'cmd_vel_topic'
            ).value,
            10
        )

        # =========================================================
        # Action
        # =========================================================

        self.action_server = ActionServer(
            self,
            ArucoApproach,
            self.get_parameter(
                'action_name'
            ).value,
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            'ArUco + LiDAR geometric '
            'approach server started'
        )

    # =============================================================
    # Helpers
    # =============================================================

    def pf(self, name):
        return float(
            self.get_parameter(name).value
        )

    def detections_callback(self, msg):

        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        with self.lock:

            for detection in msg.detections:

                self.latest_detections[
                    int(detection.id)
                ] = (
                    detection,
                    now_ns
                )

    def scan_callback(self, msg):

        with self.lock:

            self.latest_scan = msg

            self.latest_scan_time_ns = (
                self.get_clock()
                .now()
                .nanoseconds
            )

    def odom_callback(self, msg):

        with self.lock:
            self.latest_odom = msg

    # =============================================================
    # Robot pose
    # =============================================================

    def get_robot_pose(self):

        with self.lock:
            odom = self.latest_odom

        if odom is None:
            return None

        x = float(
            odom.pose.pose.position.x
        )

        y = float(
            odom.pose.pose.position.y
        )

        yaw = yaw_from_quaternion(
            odom.pose.pose.orientation
        )

        return x, y, yaw

    # =============================================================
    # Current detection
    # =============================================================

    def get_detection(self, target_id):

        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        with self.lock:

            data = self.latest_detections.get(
                target_id
            )

        if data is None:
            return None

        detection, stamp_ns = data

        age = (
            now_ns - stamp_ns
        ) / 1e9

        if age > self.pf(
            'detection_timeout'
        ):
            return None

        return detection

    # =============================================================
    # Marker normal in odom
    #
    # IMPORTANT:
    # We use marker ORIENTATION, not marker distance.
    #
    # The physical marker size can therefore be wrong without
    # affecting the distance controller.
    # =============================================================

    def get_marker_normal(
        self,
        target_id
    ):

        robot_pose = self.get_robot_pose()

        if robot_pose is None:
            return None

        robot_x, robot_y, _ = robot_pose

        odom_frame = self.get_parameter(
            'odom_frame'
        ).value

        marker_frame = (
            f'aruco_{target_id}'
        )

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    odom_frame,
                    marker_frame,
                    Time(),
                    timeout=Duration(
                        seconds=0.1
                    )
                )
            )

        except Exception:
            return None

        q = transform.transform.rotation

        # Third column of rotation matrix:
        # marker local +Z axis transformed into odom.
        #
        # This is the normal vector of the ArUco plane.

        nx = 2.0 * (
            q.x * q.z +
            q.w * q.y
        )

        ny = 2.0 * (
            q.y * q.z -
            q.w * q.x
        )

        norm = math.hypot(
            nx,
            ny
        )

        if norm < 1e-6:
            return None

        nx /= norm
        ny /= norm

        # ---------------------------------------------------------
        # Choose the sign pointing ROBOT -> MARKER.
        #
        # Marker translation may have wrong magnitude if
        # marker_size is wrong, but its direction is sufficient
        # here simply to choose ±normal.
        # ---------------------------------------------------------

        marker_x = (
            transform.transform.translation.x
        )

        marker_y = (
            transform.transform.translation.y
        )

        to_marker_x = (
            marker_x - robot_x
        )

        to_marker_y = (
            marker_y - robot_y
        )

        dot = (
            nx * to_marker_x +
            ny * to_marker_y
        )

        if dot < 0.0:
            nx = -nx
            ny = -ny

        return nx, ny

    # =============================================================
    # Front LiDAR
    # =============================================================

    def get_front_lidar_range(self):

        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        with self.lock:

            scan = self.latest_scan
            stamp_ns = (
                self.latest_scan_time_ns
            )

        if scan is None:
            return None

        if stamp_ns is None:
            return None

        age = (
            now_ns - stamp_ns
        ) / 1e9

        if age > self.pf(
            'scan_timeout'
        ):
            return None

        half_angle = math.radians(
            self.pf(
                'lidar_sector_half_angle_deg'
            )
        )

        values = []

        for i, distance in enumerate(
            scan.ranges
        ):

            angle = (
                scan.angle_min +
                i * scan.angle_increment
            )

            if abs(angle) > half_angle:
                continue

            if not math.isfinite(
                distance
            ):
                continue

            if distance < scan.range_min:
                continue

            if distance > scan.range_max:
                continue

            values.append(
                float(distance)
            )

        if not values:
            return None

        return float(
            np.median(values)
        )

    # =============================================================
    # Commands
    # =============================================================

    def publish_cmd(
        self,
        vx=0.0,
        vy=0.0,
        wz=0.0
    ):

        msg = Twist()

        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)

        self.cmd_pub.publish(msg)

    def stop_robot(self):

        for _ in range(3):

            self.publish_cmd()

            time.sleep(0.02)

    # =============================================================
    # Heading controller
    # =============================================================

    def heading_control(
        self,
        desired_heading
    ):

        pose = self.get_robot_pose()

        if pose is None:
            return None, None

        _, _, current_yaw = pose

        error = normalize_angle(
            desired_heading -
            current_yaw
        )

        wz = (
            self.pf(
                'kp_heading'
            ) *
            error
        )

        wz = clamp(
            wz,
            -self.pf(
                'max_heading_speed'
            ),
            self.pf(
                'max_heading_speed'
            )
        )

        return error, wz

    # =============================================================
    # Feedback
    # =============================================================

    def send_feedback(
        self,
        goal_handle,
        state,
        distance,
        center_error,
        elapsed
    ):

        feedback = (
            ArucoApproach.Feedback()
        )

        feedback.state = state
        feedback.distance = float(
            distance
        )

        feedback.center_error = float(
            center_error
        )

        feedback.elapsed_sec = float(
            elapsed
        )

        goal_handle.publish_feedback(
            feedback
        )

    # =============================================================
    # Action
    # =============================================================

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

        timeout_sec = float(
            goal_handle.request.timeout_sec
        )

        state = 'SEARCHING'

        start_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        lock_start_ns = None
        normal_samples = []

        desired_heading = None

        final_distance = -1.0

        period = 1.0 / max(
            1.0,
            self.pf(
                'control_rate'
            )
        )

        self.get_logger().info(
            f'Starting target ID {target_id}'
        )

        while rclpy.ok():

            now_ns = (
                self.get_clock()
                .now()
                .nanoseconds
            )

            elapsed = (
                now_ns - start_ns
            ) / 1e9

            # =====================================================
            # Cancel / timeout
            # =====================================================

            if goal_handle.is_cancel_requested:

                self.stop_robot()
                goal_handle.canceled()

                result = (
                    ArucoApproach.Result()
                )

                result.success = False
                result.status = 'CANCELED'
                result.message = (
                    'Goal canceled'
                )

                result.final_distance = (
                    final_distance
                )

                return result

            if (
                timeout_sec > 0.0 and
                elapsed >= timeout_sec
            ):

                self.stop_robot()
                goal_handle.abort()

                result = (
                    ArucoApproach.Result()
                )

                result.success = False
                result.status = 'TIMEOUT'
                result.message = (
                    'Approach timeout'
                )

                result.final_distance = (
                    final_distance
                )

                return result

            detection = (
                self.get_detection(
                    target_id
                )
            )

            center_error = 0.0

            if detection is not None:

                center_error = float(
                    detection
                    .center_x_normalized
                )

            # =====================================================
            # SEARCHING
            # =====================================================

            if state == 'SEARCHING':

                if detection is None:

                    self.publish_cmd(
                        wz=self.pf(
                            'search_angular_speed'
                        )
                    )

                else:

                    self.stop_robot()

                    normal_samples = []

                    lock_start_ns = now_ns

                    state = 'LOCK_TARGET'

                    self.get_logger().info(
                        'Target found. '
                        'Locking marker normal.'
                    )

            # =====================================================
            # LOCK_TARGET
            # =====================================================

            elif state == 'LOCK_TARGET':

                if detection is None:

                    self.stop_robot()

                    state = 'SEARCHING'

                else:

                    normal = (
                        self.get_marker_normal(
                            target_id
                        )
                    )

                    if normal is not None:

                        normal_samples.append(
                            normal
                        )

                    lock_elapsed = (
                        now_ns -
                        lock_start_ns
                    ) / 1e9

                    if (
                        lock_elapsed >=
                        self.pf(
                            'lock_duration'
                        )
                        and
                        len(normal_samples) >=
                        int(
                            self.get_parameter(
                                'lock_min_samples'
                            ).value
                        )
                    ):

                        mean_x = sum(
                            n[0]
                            for n in normal_samples
                        )

                        mean_y = sum(
                            n[1]
                            for n in normal_samples
                        )

                        norm = math.hypot(
                            mean_x,
                            mean_y
                        )

                        if norm < 1e-6:

                            normal_samples = []
                            lock_start_ns = now_ns

                        else:

                            normal_x = (
                                mean_x / norm
                            )

                            normal_y = (
                                mean_y / norm
                            )

                            desired_heading = (
                                math.atan2(
                                    normal_y,
                                    normal_x
                                )
                            )

                            self.stop_robot()

                            state = (
                                'ALIGN_HEADING_TO_ARUCO'
                            )

                            self.get_logger().info(
                                'Marker normal locked. '
                                f'Heading='
                                f'{desired_heading:.3f} rad'
                            )

            # =====================================================
            # ALIGN HEADING TO MARKER NORMAL
            # =====================================================

            elif state == (
                'ALIGN_HEADING_TO_ARUCO'
            ):

                heading_error, wz = (
                    self.heading_control(
                        desired_heading
                    )
                )

                if heading_error is None:

                    self.publish_cmd()

                elif (
                    abs(heading_error) <=
                    self.pf(
                        'heading_tolerance'
                    )
                ):

                    self.stop_robot()

                    state = (
                        'ALIGNING_LATERAL'
                    )

                    self.get_logger().info(
                        'Heading aligned with '
                        'marker normal.'
                    )

                else:

                    self.publish_cmd(
                        wz=wz
                    )

            # =====================================================
            # ALIGNING LATERAL
            #
            # Robot Y is perpendicular to marker normal because
            # heading has already been aligned with normal.
            #
            # Therefore this motion is PARALLEL to ArUco plane.
            # =====================================================

            elif state == (
                'ALIGNING_LATERAL'
            ):

                heading_error, wz = (
                    self.heading_control(
                        desired_heading
                    )
                )

                if heading_error is None:

                    self.publish_cmd()

                elif (
                    abs(heading_error) >
                    self.pf(
                        'heading_realign_threshold'
                    )
                ):

                    self.stop_robot()

                    state = (
                        'ALIGN_HEADING_TO_ARUCO'
                    )

                elif detection is None:

                    # Need camera for lateral centering.
                    self.publish_cmd(
                        wz=wz
                    )

                elif (
                    abs(center_error) <=
                    self.pf(
                        'lateral_tolerance'
                    )
                ):

                    self.stop_robot()

                    state = 'APPROACHING'

                    self.get_logger().info(
                        'Lateral alignment complete. '
                        'Starting LiDAR approach.'
                    )

                else:

                    # center_error > 0:
                    # marker is to camera RIGHT.
                    #
                    # ROS base_link +Y = LEFT,
                    # therefore move with negative vy.

                    vy = (
                        -self.pf(
                            'kp_lateral'
                        )
                        *
                        center_error
                    )

                    vy = clamp(
                        vy,
                        -self.pf(
                            'max_lateral_speed'
                        ),
                        self.pf(
                            'max_lateral_speed'
                        )
                    )

                    self.publish_cmd(
                        vy=vy,
                        wz=wz
                    )

            # =====================================================
            # APPROACHING
            #
            # X axis follows locked marker normal.
            # LiDAR provides metric distance.
            # =====================================================

            elif state == 'APPROACHING':

                heading_error, wz = (
                    self.heading_control(
                        desired_heading
                    )
                )

                if heading_error is None:

                    self.publish_cmd()

                    time.sleep(period)
                    continue

                if (
                    abs(heading_error) >
                    self.pf(
                        'heading_realign_threshold'
                    )
                ):

                    self.stop_robot()

                    state = (
                        'ALIGN_HEADING_TO_ARUCO'
                    )

                    time.sleep(period)
                    continue

                lidar_range = (
                    self.get_front_lidar_range()
                )

                if lidar_range is None:

                    self.publish_cmd(
                        wz=wz
                    )

                    self.send_feedback(
                        goal_handle,
                        'WAITING_LIDAR',
                        -1.0,
                        center_error,
                        elapsed
                    )

                    time.sleep(period)
                    continue

                # Preserve old semantics:
                #
                # goal stop_distance represents approximately
                # camera -> target distance.
                #
                # LiDAR sits ~16 cm behind camera.

                camera_distance = max(
                    0.0,
                    lidar_range -
                    self.pf(
                        'camera_x_minus_lidar_x'
                    )
                )

                final_distance = (
                    camera_distance
                )

                # -----------------------------------------------
                # Finished
                # -----------------------------------------------

                if (
                    camera_distance <=
                    stop_distance +
                    self.pf(
                        'distance_tolerance'
                    )
                ):

                    self.stop_robot()

                    goal_handle.succeed()

                    result = (
                        ArucoApproach.Result()
                    )

                    result.success = True
                    result.status = 'REACHED'
                    result.message = (
                        'Reached marker using '
                        'locked normal + LiDAR'
                    )

                    result.final_distance = (
                        camera_distance
                    )

                    self.get_logger().info(
                        f'Target reached: '
                        f'lidar={lidar_range:.3f} m, '
                        f'camera-distance='
                        f'{camera_distance:.3f} m'
                    )

                    return result

                # -----------------------------------------------
                # Optional lateral correction while marker
                # remains visible.
                # -----------------------------------------------

                vy = 0.0

                if detection is not None:

                    if (
                        abs(center_error) >
                        self.pf(
                            'lateral_realign_threshold'
                        )
                    ):

                        self.stop_robot()

                        state = (
                            'ALIGNING_LATERAL'
                        )

                        time.sleep(period)
                        continue

                    vy = (
                        -self.pf(
                            'kp_lateral'
                        )
                        *
                        center_error
                    )

                    vy = clamp(
                        vy,
                        -self.pf(
                            'max_lateral_speed'
                        ),
                        self.pf(
                            'max_lateral_speed'
                        )
                    )

                distance_error = (
                    camera_distance -
                    stop_distance
                )

                vx = (
                    self.pf(
                        'kp_linear'
                    )
                    *
                    distance_error
                )

                vx = clamp(
                    vx,
                    0.0,
                    self.pf(
                        'max_linear_speed'
                    )
                )

                self.publish_cmd(
                    vx=vx,
                    vy=vy,
                    wz=wz
                )

            # =====================================================
            # Feedback
            # =====================================================

            self.send_feedback(
                goal_handle,
                state,
                final_distance,
                center_error,
                elapsed
            )

            time.sleep(period)

        # =========================================================
        # Shutdown
        # =========================================================

        self.stop_robot()

        result = (
            ArucoApproach.Result()
        )

        result.success = False
        result.status = 'SHUTDOWN'
        result.message = 'ROS shutdown'

        result.final_distance = (
            final_distance
        )

        return result


def main(args=None):

    rclpy.init(args=args)

    node = (
        ArucoLidarApproachServer()
    )

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_robot()

        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
