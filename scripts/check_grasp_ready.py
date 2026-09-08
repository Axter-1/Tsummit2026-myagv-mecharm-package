#!/usr/bin/env python3
"""Comprueba rapidamente las acciones necesarias para ejecutar un grasp.

No arranca nodos ni inspecciona procesos. La preparacion ya comprobo los
topics de sensores; aqui solo se confirma que los tres servidores de accion
que usa la mision siguen disponibles en el DDS actual.
"""

import argparse
import time

import rclpy
from rclpy.action import ActionClient

from home_service_interfaces.action import ArucoApproach, MoveArm, PickPlace


TOPICS = (
    ("publishers", "/scan_filtered"),
    ("publishers", "/odom"),
    ("publishers", "/aruco/detections"),
    ("subscribers", "/cmd_vel_aruco"),
    ("publishers", "/cmd_vel"),
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=0.2)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("grasp_ready_check")
    clients = (
        ("/grasp_object", PickPlace),
        ("/aruco_lidar_approach", ArucoApproach),
        ("/mecharm/move_arm", MoveArm),
    )
    missing = []
    try:
        for name, action_type in clients:
            client = ActionClient(node, action_type, name)
            if client.wait_for_server(timeout_sec=args.timeout):
                status_topic = f"{name}/_action/status"
                server_count = len(node.get_publishers_info_by_topic(status_topic))
                if server_count == 1:
                    print(f"  OK    {name}")
                else:
                    print(
                        f"  FALTA {name}: {server_count} servidores detectados "
                        "(se requiere exactamente uno)"
                    )
                    missing.append(name)
            else:
                print(f"  FALTA {name}")
                missing.append(name)

        deadline = time.monotonic() + args.timeout
        remaining = set(TOPICS)
        while remaining and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            for endpoint_type, topic in tuple(remaining):
                if endpoint_type == "publishers":
                    available = node.get_publishers_info_by_topic(topic)
                else:
                    available = node.get_subscriptions_info_by_topic(topic)
                if available:
                    remaining.remove((endpoint_type, topic))
        for endpoint_type, topic in TOPICS:
            if (endpoint_type, topic) in remaining:
                print(f"  FALTA {endpoint_type} para {topic}")
                missing.append(topic)
            else:
                print(f"  OK    {endpoint_type} para {topic}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
