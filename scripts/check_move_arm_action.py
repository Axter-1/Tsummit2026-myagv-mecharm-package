#!/usr/bin/env python3
"""Comprueba /mecharm/move_arm con un cliente ROS 2 real."""

import argparse

import rclpy
from home_service_interfaces.action import MoveArm
from rclpy.action import ActionClient


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("move_arm_action_check")
    client = ActionClient(node, MoveArm, "/mecharm/move_arm")
    try:
        ready = client.wait_for_server(timeout_sec=args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if ready:
        print("Driver del MechArm listo.")
        return 0
    print("ERROR: /mecharm/move_arm no esta disponible.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
