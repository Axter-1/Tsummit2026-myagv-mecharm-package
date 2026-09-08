#!/usr/bin/env python3
"""Mide entrega de detecciones y ausencia de un ArUco concreto."""

import argparse
import time

import rclpy
from home_service_interfaces.msg import ArucoDetectionArray


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("marker_id", type=int)
    parser.add_argument("seconds", type=float, nargs="?", default=30.0)
    parser.add_argument("--expected-hz", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("aruco_loss_measurement")
    total = 0
    visible = 0

    def callback(msg: ArucoDetectionArray) -> None:
        nonlocal total, visible
        total += 1
        if any(int(detection.id) == args.marker_id for detection in msg.detections):
            visible += 1

    subscription = node.create_subscription(
        ArucoDetectionArray, "/aruco/detections", callback, 10
    )
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        elapsed = time.monotonic() - started
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

    expected = elapsed * args.expected_hz
    delivery = min(100.0, 100.0 * total / expected) if expected else 0.0
    absence = 100.0 * (total - visible) / total if total else 100.0
    print(f"ArUco {args.marker_id}: {visible}/{total} detecciones visibles "
          f"en {elapsed:.1f} s")
    print(f"Flujo recibido: {total / elapsed:.1f} Hz "
          f"({delivery:.1f}% de {args.expected_hz:.1f} Hz esperados)")
    print(f"Ausencia del marcador: {absence:.1f}% de mensajes recibidos")
    return 0 if total else 1


if __name__ == "__main__":
    raise SystemExit(main())
