#!/usr/bin/env python3
"""Unit tests and mock verification for JARVIS components."""

import unittest
from voice_assistant.config import config, Waypoint
from voice_assistant.ros_bridge import JarvisRosBridge


class TestJarvisLogic(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import rclpy
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()

    def test_geofencing_and_safety_interlock(self):
        """Verifica que el geofencing y la regla de oro ALLOW_MOTION impidan movimientos inseguros."""
        bridge = JarvisRosBridge()

        # 1. Coordenadas fuera de límite (debe fallar por geofencing inmediatamente)
        res_out = bridge.send_goal_pose(x=50.0, y=50.0, yaw_deg=0.0)
        self.assertFalse(res_out["success"])
        self.assertIn("perímetro de seguridad", res_out["error"])

        # 2. Sin ALLOW_MOTION autorizado (debe fallar por regla de oro)
        config.allow_motion = False
        res_motion = bridge.send_goal_pose(x=1.0, y=1.0, yaw_deg=0.0)
        self.assertFalse(res_motion["success"])
        self.assertIn("ALLOW_MOTION", res_motion["error"])

        # 3. Con ALLOW_MOTION autorizado pero sin enlace Wi-Fi con el robot
        config.allow_motion = True
        res_conn = bridge.send_goal_pose(x=1.0, y=1.0, yaw_deg=0.0)
        self.assertFalse(res_conn["success"])
        self.assertIn("Enlace Wi-Fi", res_conn["error"])

        bridge.destroy_node()

    def test_known_locations_and_pieces(self):
        """Verifica que las ubicaciones del torneo y las piezas del catálogo estén registradas."""
        self.assertIn("inicio", config.locations)
        self.assertIn("clasificacion", config.locations)
        self.assertIn("kitting", config.locations)
        self.assertIn("ensamblaje", config.locations)
        self.assertIn("laberinto", config.locations)

        # Catálogo de piezas oficiales
        self.assertIn("engranaje", config.pieces)
        self.assertIn("poste", config.pieces)
        self.assertIn("rueda", config.pieces)

    def test_dds_link_diagnostic(self):
        """Verifica que el método de diagnóstico preflight funcione sin excepciones."""
        bridge = JarvisRosBridge()
        diag = bridge.check_dds_link()
        self.assertIn("dds_linked", diag)
        self.assertIn("diagnostic", diag)
        bridge.destroy_node()


if __name__ == "__main__":
    unittest.main()
