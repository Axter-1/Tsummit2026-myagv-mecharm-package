#!/usr/bin/env python3
"""Comprueba la accion remota de aproximacion con un cliente ROS 2."""

import argparse

import rclpy
from home_service_interfaces.action import ArucoApproach
from rclpy.action import ActionClient


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("aruco_approach_action_check")
    client = ActionClient(node, ArucoApproach, "/aruco_lidar_approach")
    try:
        ready = client.wait_for_server(timeout_sec=args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if ready:
        print("Servidor de aproximacion listo.")
        return 0
    print("ERROR: /aruco_lidar_approach no esta disponible.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
