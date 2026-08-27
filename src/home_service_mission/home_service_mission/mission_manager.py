#!/usr/bin/env python3

import math
import os
import time

import yaml

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from home_service_interfaces.action import ArucoApproach


class MissionManager(Node):

    def __init__(self):

        super().__init__('home_service_mission_manager')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            'mission_file',
            ''
        )

        self.stop_on_failure = bool(
            self.mission.get(
                'stop_on_failure',
                True
            )
        )

        self.repeat_count = int(
            self.mission.get(
                'repeat',
                1
            )
        )

        if self.repeat_count == 0:

            self.get_logger().info(
                'Mission repetitions: INFINITE'
            )

        else:

            self.get_logger().info(
                f'Mission repetitions: '
                f'{self.repeat_count}'
            )

        self.get_logger().info(
            f'Mission repetitions: '
            f'{self.repeat_count}'
        )

        mission_file = self.get_parameter(
            'mission_file'
        ).value

        if not mission_file:
            raise RuntimeError(
                'Parameter "mission_file" is empty.'
            )

        mission_file = os.path.expanduser(
            mission_file
        )

        if not os.path.isfile(mission_file):
            raise RuntimeError(
                f'Mission file does not exist: '
                f'{mission_file}'
            )

        # ---------------------------------------------------------
        # Action clients
        # ---------------------------------------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.aruco_client = ActionClient(
            self,
            ArucoApproach,
            '/aruco_lidar_approach'
        )

        # Used only to avoid printing the same ArUco state at 20 Hz.
        self.last_aruco_state = None

        # ---------------------------------------------------------
        # Load YAML
        # ---------------------------------------------------------

        with open(
            mission_file,
            'r',
            encoding='utf-8'
        ) as file:

            data = yaml.safe_load(file)

        if data is None:
            raise RuntimeError(
                'Mission YAML is empty.'
            )

        self.mission = data.get(
            'mission',
            {}
        )

        self.steps = self.mission.get(
            'steps',
            []
        )

        if not self.steps:
            raise RuntimeError(
                'Mission contains no steps.'
            )

        self.mission_name = (
            self.mission.get(
                'name',
                'unnamed_mission'
            )
        )

        self.stop_on_failure = bool(
            self.mission.get(
                'stop_on_failure',
                True
            )
        )

        self.get_logger().info(
            f'Mission loaded: '
            f'{self.mission_name}'
        )

        self.get_logger().info(
            f'Number of steps: '
            f'{len(self.steps)}'
        )

    # =============================================================
    # Wait for servers
    # =============================================================

    def wait_for_action_servers(self):

        self.get_logger().info(
            'Waiting for Nav2 action server...'
        )

        while rclpy.ok():

            if self.nav_client.wait_for_server(
                timeout_sec=2.0
            ):
                break

            self.get_logger().warn(
                'Still waiting for '
                '/navigate_to_pose...'
            )

        self.get_logger().info(
            'Nav2 action server available.'
        )

        self.get_logger().info(
            'Waiting for ArUco LiDAR '
            'action server...'
        )

        while rclpy.ok():

            if self.aruco_client.wait_for_server(
                timeout_sec=2.0
            ):
                break

            self.get_logger().warn(
                'Still waiting for '
                '/aruco_lidar_approach...'
            )

        self.get_logger().info(
            'ArUco LiDAR action server available.'
        )

    # =============================================================
    # Navigation
    # =============================================================

    def execute_navigation(
        self,
        step
    ):

        name = step.get(
            'name',
            'navigate'
        )

        x = float(
            step['x']
        )

        y = float(
            step['y']
        )

        if 'yaw_deg' in step:

            yaw = math.radians(
                float(
                    step['yaw_deg']
                )
            )

        else:

            yaw = float(
                step.get(
                    'yaw',
                    0.0
                )
            )

        frame_id = step.get(
            'frame_id',
            'map'
        )

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = (
            frame_id
        )

        goal.pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0

        goal.pose.pose.orientation.z = (
            math.sin(
                yaw / 2.0
            )
        )

        goal.pose.pose.orientation.w = (
            math.cos(
                yaw / 2.0
            )
        )

        self.get_logger().info(
            '--------------------------------'
        )

        self.get_logger().info(
            f'NAVIGATION: {name}'
        )

        self.get_logger().info(
            f'Goal: '
            f'x={x:.3f}, '
            f'y={y:.3f}, '
            f'yaw={math.degrees(yaw):.1f} deg'
        )

        send_future = (
            self.nav_client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = (
            send_future.result()
        )

        if goal_handle is None:

            self.get_logger().error(
                'Nav2 did not return '
                'a goal handle.'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                'Nav2 rejected the goal.'
            )

            return False

        self.get_logger().info(
            'Nav2 goal accepted.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result_response = (
            result_future.result()
        )

        if result_response is None:

            self.get_logger().error(
                'Nav2 returned no result.'
            )

            return False

        status = (
            result_response.status
        )

        if (
            status ==
            GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().info(
                f'Navigation completed: '
                f'{name}'
            )

            return True

        self.get_logger().error(
            f'Navigation failed. '
            f'Action status={status}'
        )

        return False

    # =============================================================
    # ArUco feedback
    # =============================================================

    def aruco_feedback_callback(
        self,
        feedback_msg
    ):

        feedback = (
            feedback_msg.feedback
        )

        state = feedback.state

        if (
            state !=
            self.last_aruco_state
        ):

            self.last_aruco_state = (
                state
            )

            self.get_logger().info(
                f'ArUco state → '
                f'{state}'
            )

        # Print useful distance only when available.
        if (
            feedback.distance >= 0.0
            and
            state in (
                'APPROACHING',
                'WAITING_LIDAR'
            )
        ):

            self.get_logger().debug(
                f'ArUco/LiDAR distance: '
                f'{feedback.distance:.3f} m'
            )

    # =============================================================
    # ArUco + LiDAR
    # =============================================================

    def execute_aruco(
        self,
        step
    ):

        name = step.get(
            'name',
            'aruco'
        )

        target_id = int(
            step['id']
        )

        stop_distance = float(
            step.get(
                'stop_distance',
                0.12
            )
        )

        timeout_sec = float(
            step.get(
                'timeout_sec',
                0.0
            )
        )

        goal = ArucoApproach.Goal()

        goal.target_id = target_id
        goal.stop_distance = stop_distance
        goal.timeout_sec = timeout_sec

        self.last_aruco_state = None

        self.get_logger().info(
            '--------------------------------'
        )

        self.get_logger().info(
            f'ARUCO: {name}'
        )

        self.get_logger().info(
            f'Target ID={target_id}, '
            f'stop_distance='
            f'{stop_distance:.3f} m'
        )

        send_future = (
            self.aruco_client.send_goal_async(
                goal,
                feedback_callback=(
                    self.aruco_feedback_callback
                )
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = (
            send_future.result()
        )

        if goal_handle is None:

            self.get_logger().error(
                'ArUco server returned '
                'no goal handle.'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                'ArUco goal rejected.'
            )

            return False

        self.get_logger().info(
            'ArUco goal accepted.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result_response = (
            result_future.result()
        )

        if result_response is None:

            self.get_logger().error(
                'ArUco server returned '
                'no result.'
            )

            return False

        result = (
            result_response.result
        )

        action_status = (
            result_response.status
        )

        if (
            action_status ==
            GoalStatus.STATUS_SUCCEEDED
            and
            result.success
        ):

            self.get_logger().info(
                f'ArUco reached: '
                f'ID={target_id}'
            )

            self.get_logger().info(
                f'Final distance: '
                f'{result.final_distance:.3f} m'
            )

            return True

        self.get_logger().error(
            f'ArUco approach failed: '
            f'status={result.status}, '
            f'message="{result.message}"'
        )

        return False

    # =============================================================
    # Execute generic step
    # =============================================================

    def execute_step(
        self,
        step
    ):

        step_type = (
            str(
                step.get(
                    'type',
                    ''
                )
            )
            .strip()
            .lower()
        )

        if step_type == 'navigate':

            return self.execute_navigation(
                step
            )

        if step_type == 'aruco':

            return self.execute_aruco(
                step
            )

        self.get_logger().error(
            f'Unknown mission step type: '
            f'"{step_type}"'
        )

        return False

    # =============================================================
    # Mission
    # =============================================================

    def run_mission(self):

        self.wait_for_action_servers()

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            f'STARTING MISSION: '
            f'{self.mission_name}'
        )

        if self.repeat_count == 0:

            self.get_logger().info(
                'Repetitions: INFINITE'
            )

        else:

            self.get_logger().info(
                f'Repetitions: '
                f'{self.repeat_count}'
            )

        self.get_logger().info(
            '================================'
        )

        repetition = 0

        while rclpy.ok():

            # ---------------------------------------------------------
            # Stop when finite repetition count has been reached.
            #
            # repeat_count == 0 means infinite.
            # ---------------------------------------------------------

            if (
                self.repeat_count > 0
                and
                repetition >= self.repeat_count
            ):
                break

            repetition += 1

            if self.repeat_count == 0:

                loop_text = (
                    f'{repetition}/∞'
                )

            else:

                loop_text = (
                    f'{repetition}/'
                    f'{self.repeat_count}'
                )

            self.get_logger().info(
                '################################'
            )

            self.get_logger().info(
                f'MISSION LOOP '
                f'{loop_text}'
            )

            self.get_logger().info(
                '################################'
            )

            # =========================================================
            # Execute every step in this loop
            # =========================================================

            for index, step in enumerate(
                self.steps,
                start=1
            ):

                if not rclpy.ok():
                    return False

                name = step.get(
                    'name',
                    f'step_{index}'
                )

                retries = int(
                    step.get(
                        'retries',
                        0
                    )
                )

                success = False

                for attempt in range(
                    retries + 1
                ):

                    if not rclpy.ok():
                        return False

                    self.get_logger().info(
                        f'Loop {loop_text} '
                        f'- Step '
                        f'{index}/'
                        f'{len(self.steps)}: '
                        f'{name}'
                    )

                    if attempt > 0:

                        self.get_logger().warn(
                            f'Retry '
                            f'{attempt}/'
                            f'{retries}'
                        )

                    success = (
                        self.execute_step(
                            step
                        )
                    )

                    if success:
                        break

                    # ---------------------------------------------
                    # Small pause before retrying.
                    # ---------------------------------------------

                    if rclpy.ok():
                        time.sleep(1.0)

                # =====================================================
                # Step failed
                # =====================================================

                if not success:

                    self.get_logger().error(
                        f'Step failed: '
                        f'{name}'
                    )

                    if self.stop_on_failure:

                        self.get_logger().error(
                            'MISSION ABORTED'
                        )

                        return False

                    self.get_logger().warn(
                        'Continuing mission '
                        'despite failure.'
                    )

            # =========================================================
            # Whole loop completed
            # =========================================================

            self.get_logger().info(
                '--------------------------------'
            )

            self.get_logger().info(
                f'Mission loop '
                f'{loop_text} completed.'
            )

            self.get_logger().info(
                '--------------------------------'
            )

        # =============================================================
        # Finite mission completed
        # =============================================================

        if self.repeat_count > 0:

            self.get_logger().info(
                '================================'
            )

            self.get_logger().info(
                'ALL MISSION LOOPS COMPLETE'
            )

            self.get_logger().info(
                '================================'
            )

            return True

        return False

def main(args=None):

    rclpy.init(args=args)

    node = MissionManager()

    try:

        success = node.run_mission()

        if not success:
            node.get_logger().error(
                'Mission finished '
                'with failure.'
            )

    except KeyboardInterrupt:

        node.get_logger().warn(
            'Mission interrupted '
            'by user.'
        )

    except Exception as exc:

        node.get_logger().error(
            f'Mission exception: '
            f'{exc}'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
