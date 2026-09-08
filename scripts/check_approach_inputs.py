#!/usr/bin/env python3
"""Comprueba que la aproximacion recibe sus tres flujos ROS 2."""

import argparse
import time

import rclpy
from home_service_interfaces.msg import ArucoDetectionArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("approach_input_check")
    received: set[str] = set()
    sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [
        node.create_subscription(
            LaserScan, "/scan_filtered", lambda _: received.add("/scan_filtered"), sensor_qos
        ),
        node.create_subscription(
            ArucoDetectionArray,
            "/aruco/detections",
            lambda _: received.add("/aruco/detections"),
            10,
        ),
        node.create_subscription(Odometry, "/odom", lambda _: received.add("/odom"), 10),
    ]
    deadline = time.monotonic() + args.timeout
    expected = ("/scan_filtered", "/aruco/detections", "/odom")
    try:
        while time.monotonic() < deadline and len(received) < len(expected):
            rclpy.spin_once(node, timeout_sec=min(0.2, deadline - time.monotonic()))
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

    for topic in expected:
        print(f"  {'OK   ' if topic in received else 'FALTA'} {topic}")
    return 0 if len(received) == len(expected) else 1


if __name__ == "__main__":
    raise SystemExit(main())
