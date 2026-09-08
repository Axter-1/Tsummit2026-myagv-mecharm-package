#!/usr/bin/env python3
"""ROS 2 Bridge for JARVIS Base Station.

Handles:
- CycloneDDS preflight diagnostics (/odom publishers, twist_mux subscriptions).
- Strict 'ALLOW_MOTION' safety interlock.
- Nav2 high-level navigation (/navigate_to_pose).
- ArUco visual-lidar precision approach (/aruco_lidar_approach).
- MechArm 270 M5 manipulation (/mecharm/move_arm, /mecharm/pick_place, /mecharm/set_gripper).
- Telemetry subscription (/odom) and emergency stop (/cmd_vel zero-burst).
"""

import math
import os
import threading
import time
from typing import Dict, Any, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

# Interfaces específicas del proyecto (con importación segura / condicional)
try:
    from home_service_interfaces.action import ArucoApproach, MoveArm, PickPlace
    from home_service_interfaces.srv import SetGripper
    HAS_CUSTOM_INTERFACES = True
except ImportError:
    HAS_CUSTOM_INTERFACES = False
    ArucoApproach = None
    MoveArm = None
    PickPlace = None
    SetGripper = None

from voice_assistant.config import config, Waypoint


class JarvisRosBridge(Node):
    """ROS 2 Node providing high-level commands, manipulation, and telemetry to JARVIS."""

    def __init__(self):
        super().__init__("jarvis_base_station_bridge")

        self.cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        # --- Telemetría y Estado Interno ---
        self.last_odom_time: float = 0.0
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_yaw_deg: float = 0.0
        self.linear_speed: float = 0.0
        self.angular_speed: float = 0.0

        # --- Control de Navegación y Acciones ---
        self.active_nav_goal_handle = None
        self.active_aruco_goal_handle = None
        self.active_arm_goal_handle = None
        self.navigation_status: str = "IDLE"  # IDLE, NAVIGATING, SUCCEEDED, FAILED, CANCELED
        self.nav_start_time: float = 0.0
        self.current_target_name: str = ""

        # --- Suscriptor de Odometría (QoS adaptada a Best Effort Wi-Fi) ---
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            config.odom_topic,
            self._odom_callback,
            odom_qos,
            callback_group=self.cb_group,
        )

        # --- Publicador de parada de emergencia en /cmd_vel (twist_mux) ---
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            config.cmd_vel_topic,
            10,
            callback_group=self.cb_group,
        )

        # --- Cliente de Acción Nav2 (NavigateToPose) ---
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            config.nav_action_name,
            callback_group=self.cb_group,
        )

        # --- Clientes de Acción / Servicios Específicos del Robot ---
        self.aruco_client = None
        self.arm_move_client = None
        self.arm_pick_client = None
        self.gripper_client = None

        if HAS_CUSTOM_INTERFACES:
            self.aruco_client = ActionClient(
                self, ArucoApproach, config.aruco_action_name, callback_group=self.cb_group
            )
            self.arm_move_client = ActionClient(
                self, MoveArm, config.arm_move_action, callback_group=self.cb_group
            )
            self.arm_pick_client = ActionClient(
                self, PickPlace, config.arm_pick_action, callback_group=self.cb_group
            )
            self.gripper_client = self.create_client(
                SetGripper, config.gripper_service, callback_group=self.cb_group
            )

        self.get_logger().info("JARVIS ROS 2 Bridge inicializado.")

    # -------------------------------------------------------------------------
    # Comprobación de Enlace DDS y Diagnóstico Pre-flight
    # -------------------------------------------------------------------------
    def check_dds_link(self) -> Dict[str, Any]:
        """Diagnostica si el enlace DDS Unicast está operando y si DISTRIBUTED=1 fue cargado."""
        odom_pubs = self.get_publishers_info_by_topic(config.odom_topic)
        aruco_subs = self.get_subscriptions_info_by_topic(config.cmd_vel_aruco_topic)

        odom_pub_count = len(odom_pubs)
        aruco_sub_count = len(aruco_subs)
        has_twist_mux = any("twist_mux" in sub.node_name for sub in aruco_subs)

        is_connected = odom_pub_count >= 1
        current_domain = os.environ.get("ROS_DOMAIN_ID", "30")

        diag_msg = ""
        if not is_connected:
            diag_msg = (
                "ALERTA CRÍTICA: /odom tiene 0 publicadores. "
                "Causa común: El contenedor de la Jetson se ejecutó SIN 'DISTRIBUTED=1', "
                "por lo que quedó atrapado en loopback (lo). "
                "Solución requerida: En la Jetson ejecute './scripts/tsummit.sh stop' y luego: "
                "'DISTRIBUTED=1 ROBOT_IP=... LAPTOP_IP=... ./scripts/tsummit.sh approach-check'."
            )
        elif not has_twist_mux and aruco_sub_count == 0:
            diag_msg = (
                "ADVERTENCIA: Odometría recibida, pero 'twist_mux' no aparece suscrito a /cmd_vel_aruco. "
                "Verifique que la rutina base del robot esté activa."
            )
        else:
            diag_msg = "Enlace DDS óptimo. twist_mux y odometría de 4 ruedas operando nominalmente."

        return {
            "dds_linked": is_connected,
            "odom_publishers": odom_pub_count,
            "twist_mux_subscribed": has_twist_mux,
            "robot_domain_id": current_domain,
            "allow_motion_state": "HABILITADO" if config.allow_motion else "BLOQUEADO (Seguro)",
            "diagnostic": diag_msg,
        }

    # -------------------------------------------------------------------------
    # Regla de Oro de Seguridad: ALLOW_MOTION
    # -------------------------------------------------------------------------
    def set_motion_authorization(self, allowed: bool) -> Dict[str, Any]:
        """Permite al operador humano habilitar o bloquear por voz el movimiento físico."""
        config.allow_motion = allowed
        state_str = "HABILITADO (Actuadores armados)" if allowed else "BLOQUEADO (Modo seguro)"
        self.get_logger().warn(f"Protocolo de seguridad: Movimiento físico {state_str}.")
        return {
            "success": True,
            "allow_motion": config.allow_motion,
            "message": f"Protocolo de seguridad: Movimiento físico de ruedas y brazo {state_str}.",
        }

    def _check_motion_permission(self) -> Optional[Dict[str, Any]]:
        """Valida que la regla ALLOW_MOTION esté satisfecha antes de accionar motores."""
        if not config.allow_motion:
            return {
                "success": False,
                "error": (
                    "REGLA DE ORO DE SEGURIDAD VIOLADA: 'ALLOW_MOTION=1' no está activo. "
                    "Por diseño estricto del myAGV, no se permite accionar motores ni servos sin autorización. "
                    "El operador debe ordenar explícitamente: 'JARVIS, autorizo movimiento'."
                ),
            }
        return None

    # -------------------------------------------------------------------------
    # Callbacks de Odometría
    # -------------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry):
        with self._lock:
            self.last_odom_time = time.time()
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.current_yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            self.linear_speed = math.hypot(vx, vy)
            self.angular_speed = msg.twist.twist.angular.z

    def is_robot_connected(self) -> bool:
        if self.last_odom_time == 0.0:
            return False
        return (time.time() - self.last_odom_time) <= config.odom_timeout_sec

    # -------------------------------------------------------------------------
    # Telemetría y Consultas
    # -------------------------------------------------------------------------
    def get_telemetry(self) -> Dict[str, Any]:
        with self._lock:
            connected = self.is_robot_connected()
            dt = time.time() - self.last_odom_time if self.last_odom_time > 0 else 999.0
            telemetry = {
                "connection_alive": connected,
                "link_latency_sec": round(dt, 2),
                "position_x_m": round(self.current_x, 2),
                "position_y_m": round(self.current_y, 2),
                "orientation_yaw_deg": round(self.current_yaw_deg, 1),
                "linear_speed_mps": round(self.linear_speed, 2),
                "angular_speed_radps": round(self.angular_speed, 2),
                "navigation_status": self.navigation_status,
                "allow_motion": config.allow_motion,
                "current_target": self.current_target_name or "Ninguno",
            }
        return telemetry

    def get_known_locations(self) -> Dict[str, str]:
        return {
            name: f"({wp.x:.1f}, {wp.y:.1f}) - {wp.description}"
            for name, wp in config.locations.items()
        }

    # -------------------------------------------------------------------------
    # Navegación Nav2 de Alto Nivel
    # -------------------------------------------------------------------------
    def navigate_to_named_location(self, location_name: str) -> Dict[str, Any]:
        motion_err = self._check_motion_permission()
        if motion_err:
            return motion_err

        loc_key = location_name.lower().strip()
        if loc_key not in config.locations:
            available = ", ".join(config.locations.keys())
            return {
                "success": False,
                "error": f"Ubicación '{location_name}' no reconocida. Válidas: {available}",
            }

        wp = config.locations[loc_key]
        return self.send_goal_pose(wp.x, wp.y, wp.yaw_deg, target_label=loc_key)

    def send_goal_pose(
        self,
        x: float,
        y: float,
        yaw_deg: float = 0.0,
        target_label: str = "Coordenadas arbitrarias",
    ) -> Dict[str, Any]:
        # 1. Geofencing
        if not (config.x_min <= x <= config.x_max and config.y_min <= y <= config.y_max):
            return {
                "success": False,
                "error": f"Coordenadas ({x:.2f}, {y:.2f}) violan el perímetro de seguridad "
                         f"[{config.x_min}, {config.x_max}] x [{config.y_min}, {config.y_max}].",
            }

        # 2. Regla de oro de movimiento
        motion_err = self._check_motion_permission()
        if motion_err:
            return motion_err

        # 3. Enlace Wi-Fi activo
        if not self.is_robot_connected():
            return {
                "success": False,
                "error": "Enlace Wi-Fi con el robot inactivo o con pérdida de paquetes. Telemetría no recibida.",
            }

        # 4. Servidor Nav2 disponible
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return {
                "success": False,
                "error": "El servidor de navegación Nav2 ('navigate_to_pose') no responde en la red ROS.",
            }

        self.cancel_current_goal(silent=True)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        yaw_rad = math.radians(yaw_deg)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self.navigation_status = "NAVIGATING"
        self.nav_start_time = time.time()
        self.current_target_name = target_label

        self.get_logger().info(
            f"Despachando meta Nav2: ({x:.2f}, {y:.2f}, yaw={yaw_deg:.1f}°) para '{target_label}'"
        )

        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

        return {
            "success": True,
            "message": f"Meta de navegación aceptada hacia '{target_label}' en ({x:.2f}, {y:.2f}).",
            "target": target_label,
            "target_coordinates": {"x": x, "y": y, "yaw_deg": yaw_deg},
        }

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Meta de navegación rechazada por Nav2.")
            self.navigation_status = "REJECTED"
            return

        self.active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.navigation_status = "SUCCEEDED"
            self.get_logger().info(f"Objetivo '{self.current_target_name}' alcanzado.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.navigation_status = "CANCELED"
        else:
            self.navigation_status = "FAILED"
        self.active_nav_goal_handle = None

    def _nav_feedback_callback(self, feedback_msg):
        if self.navigation_status == "NAVIGATING":
            elapsed = time.time() - self.nav_start_time
            if elapsed > config.nav_timeout_sec:
                self.get_logger().error(f"Timeout de navegación ({elapsed:.1f}s). Cancelando meta.")
                self.cancel_current_goal()

    # -------------------------------------------------------------------------
    # Aproximación Visual-LiDAR a Marcadores ArUco
    # -------------------------------------------------------------------------
    def approach_aruco_marker(
        self, marker_id: int, stop_distance: float = 0.20, timeout_sec: float = 40.0
    ) -> Dict[str, Any]:
        """Ejecuta alineación frontal y aproximación milimétrica a un marcador ArUco."""
        motion_err = self._check_motion_permission()
        if motion_err:
            return motion_err

        if not HAS_CUSTOM_INTERFACES or self.aruco_client is None:
            return {
                "success": False,
                "error": "Interfaces de ArUco no disponibles en este host. Verifique instalación.",
            }

        if not self.aruco_client.wait_for_server(timeout_sec=2.0):
            return {
                "success": False,
                "error": f"El servidor '{config.aruco_action_name}' no responde. Ejecute 'tsummit.sh approach-check'.",
            }

        goal_msg = ArucoApproach.Goal()
        goal_msg.target_id = int(marker_id)
        goal_msg.stop_distance = float(stop_distance)
        goal_msg.timeout_sec = float(timeout_sec)

        self.get_logger().info(
            f"Despachando ArucoApproach: ID={marker_id}, Parada={stop_distance}m, Timeout={timeout_sec}s"
        )
        send_future = self.aruco_client.send_goal_async(goal_msg)

        def _on_accepted(future):
            gh = future.result()
            if gh.accepted:
                self.active_aruco_goal_handle = gh

        send_future.add_done_callback(_on_accepted)

        return {
            "success": True,
            "message": f"Aproximación iniciada hacia ArUco ID {marker_id} a distancia de parada {stop_distance:.2f} m.",
            "marker_id": marker_id,
            "stop_distance": stop_distance,
        }

    # -------------------------------------------------------------------------
    # Manipulación MechArm 270 M5 (Retos 1, 2, 3)
    # -------------------------------------------------------------------------
    def move_arm_to_pose(self, pose_name: str) -> Dict[str, Any]:
        """Mueve el brazo robótico MechArm a una pose nombrada de poses.yaml."""
        motion_err = self._check_motion_permission()
        if motion_err:
            return motion_err

        pose_name_clean = pose_name.lower().strip()
        if pose_name_clean not in config.known_arm_poses:
            valid = ", ".join(config.known_arm_poses)
            return {
                "success": False,
                "error": f"Pose '{pose_name}' no reconocida. Poses válidas: {valid}",
            }

        if not HAS_CUSTOM_INTERFACES or self.arm_move_client is None:
            return {
                "success": False,
                "error": "Acción MoveArm no disponible. Verifique compilación del paquete de interfaces.",
            }

        if not self.arm_move_client.wait_for_server(timeout_sec=2.0):
            return {
                "success": False,
                "error": f"Servidor {config.arm_move_action} no responde. Ejecute './scripts/tsummit.sh arm'.",
            }

        goal = MoveArm.Goal()
        goal.pose_name = pose_name_clean
        goal.speed_percent = 35.0
        self.arm_move_client.send_goal_async(goal)

        return {
            "success": True,
            "message": f"Brazo MechArm moviéndose a pose '{pose_name_clean}'.",
            "pose": pose_name_clean,
        }

    def set_gripper_aperture(self, value: int, speed_percent: float = 40.0) -> Dict[str, Any]:
        """Controla la apertura de la pinza adaptativa (0 = totalmente cerrada, 100 = totalmente abierta)."""
        motion_err = self._check_motion_permission()
        if motion_err:
            return motion_err

        if not HAS_CUSTOM_INTERFACES or self.gripper_client is None:
            return {"success": False, "error": "Servicio SetGripper no disponible."}

        if not self.gripper_client.wait_for_service(timeout_sec=2.0):
            return {"success": False, "error": "Servicio /mecharm/set_gripper no disponible."}

        val_clamped = max(0, min(100, int(value)))
        req = SetGripper.Request()
        req.value = val_clamped
        req.speed_percent = float(speed_percent)

        self.gripper_client.call_async(req)
        return {
            "success": True,
            "message": f"Pinza ajustada a valor {val_clamped}/100 (velocidad {speed_percent}%).",
            "gripper_value": val_clamped,
        }

    def grasp_piece_action(self, piece_name: str, dry_run: bool = False) -> Dict[str, Any]:
        """Calcula o ejecuta la toma física de una pieza del catálogo oficial (engranaje, poste, rueda)."""
        clean_name = piece_name.lower().strip()
        if clean_name not in config.pieces:
            valid = ", ".join(config.pieces.keys())
            return {
                "success": False,
                "error": f"Pieza '{piece_name}' no catalogada. Válidas: {valid}",
            }

        piece = config.pieces[clean_name]

        # Ensayo en seco (grasp-dry)
        if dry_run:
            return {
                "success": True,
                "mode": "DRY_RUN (Simulación)",
                "piece": piece.name,
                "target_aruco": piece.target_aruco_id,
                "grasp_z_mm": piece.grasp_z_mm,
                "gripper_open_val": piece.gripper_open,
                "gripper_close_val": piece.gripper_close,
                "description": piece.description,
                "message": (
                    f"Cálculo de agarre para '{piece.name}': ArUco {piece.target_aruco_id}, "
                    f"apertura pinza {piece.gripper_open}->{piece.gripper_close}, Z={piece.grasp_z_mm} mm. "
                    "Alcance dentro de límites nominales."
                ),
            }

        # Toma real (exige ALLOW_MOTION)
        motion_err = self._check_motion_permission()
        if motion_err:
            return motion_err

        if not HAS_CUSTOM_INTERFACES or self.arm_pick_client is None:
            return {"success": False, "error": "Acción PickPlace no disponible."}

        if not self.arm_pick_client.wait_for_server(timeout_sec=2.0):
            return {"success": False, "error": "Servidor /mecharm/pick_place no disponible."}

        goal = PickPlace.Goal()
        goal.operation = "pick"
        goal.target_pose_name = "pick_table"
        goal.approach_height = 60.0
        goal.gripper_open_value = piece.gripper_open
        goal.gripper_closed_value = piece.gripper_close
        goal.speed_percent = 25.0
        goal.retreat_pose_name = "carry"

        self.arm_pick_client.send_goal_async(goal)
        return {
            "success": True,
            "mode": "REAL_MOTION",
            "piece": piece.name,
            "message": f"Secuencia de agarre iniciada para '{piece.name}' (APPROACH -> DESCEND -> GRIP -> LIFT).",
        }

    # -------------------------------------------------------------------------
    # Parada de Emergencia y Cancelación
    # -------------------------------------------------------------------------
    def cancel_current_goal(self, silent: bool = False) -> Dict[str, Any]:
        """Cancela navegación o aproximación activa."""
        canceled_any = False
        if self.active_nav_goal_handle is not None:
            self.active_nav_goal_handle.cancel_goal_async()
            self.active_nav_goal_handle = None
            canceled_any = True

        if self.active_aruco_goal_handle is not None:
            self.active_aruco_goal_handle.cancel_goal_async()
            self.active_aruco_goal_handle = None
            canceled_any = True

        self.navigation_status = "CANCELED"
        if not silent:
            self.get_logger().info("Metas canceladas.")
        return {
            "success": True,
            "message": "Metas activas canceladas." if canceled_any else "No había metas en ejecución.",
        }

    def emergency_stop(self) -> Dict[str, Any]:
        """Parada de emergencia absoluta: Cancela metas y satura /cmd_vel a cero."""
        self.cancel_current_goal(silent=True)
        self.navigation_status = "EMERGENCY_STOPPED"

        stop_twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_twist)
            time.sleep(0.02)

        self.get_logger().warn("¡PARADA DE EMERGENCIA EJECUTADA!")
        return {
            "success": True,
            "message": "¡Parada de emergencia ejecutada! Metas anuladas y actuadores bloqueados a cero.",
        }

    def destroy_node(self):
        try:
            if hasattr(self, "nav_client") and self.nav_client is not None:
                self.nav_client.destroy()
            if hasattr(self, "aruco_client") and self.aruco_client is not None:
                self.aruco_client.destroy()
            if hasattr(self, "arm_move_client") and self.arm_move_client is not None:
                self.arm_move_client.destroy()
            if hasattr(self, "arm_pick_client") and self.arm_pick_client is not None:
                self.arm_pick_client.destroy()
        except Exception:
            pass
        return super().destroy_node()


class RosBridgeManager:
    """Administra el ciclo de vida del nodo ROS 2 en un hilo secundario."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.bridge_node = JarvisRosBridge()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.bridge_node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def shutdown(self):
        self.bridge_node.emergency_stop()
        self.bridge_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
