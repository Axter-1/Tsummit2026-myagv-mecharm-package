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
        server_count = len(
            node.get_publishers_info_by_topic(
                "/aruco_lidar_approach/_action/status"
            )
        ) if ready else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if ready and server_count == 1:
        print("Servidor de aproximacion listo.")
        return 0
    if ready:
        print(
            "ERROR: /aruco_lidar_approach tiene "
            f"{server_count} servidores; debe haber exactamente uno."
        )
        return 1
    print("ERROR: /aruco_lidar_approach no esta disponible.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
