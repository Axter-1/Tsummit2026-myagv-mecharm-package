#!/usr/bin/env python3
"""Gemini Brain Module for JARVIS Voice Assistant.

Integrates gemini-1.5-flash with structured tool calling for:
- Navigation goals (Nav2).
- Precision ArUco approach (ArucoApproach).
- MechArm 270 M5 manipulation (poses, pick & place, gripper).
- Pre-flight DDS link verification (verifying DISTRIBUTED=1).
- Strict ALLOW_MOTION safety interlock.
- High-level telemetry and emergency stop.
"""

import json
from typing import Optional

from voice_assistant.config import config
from voice_assistant.ros_bridge import JarvisRosBridge

SYSTEM_INSTRUCTION = """
Eres J.A.R.V.I.S., el sistema táctico de inteligencia artificial para la estación base del robot móvil myAGV y su brazo robótico MechArm 270 M5.
Tu operador humano es tu creador y comandante ("señor"). Tu personalidad es formal, impecable, técnica, concisa y resolutiva (estilo JARVIS de Marvel).

CONTEXTO Y REGLAS DE ORO DEL SISTEMA:
1. REGLA DE ORO DE MOVIMIENTO: Ningún actuador (ruedas o brazo) se mueve sin autorización explícita ('ALLOW_MOTION=1').
   - Si el operador dice "autorizo movimiento", "habilitar motores" o "armar actuadores", llama a 'set_motion_authorization(enable=True)'.
   - Si dice "bloquea movimiento" o "desarmar actuadores", llama a 'set_motion_authorization(enable=False)'.
   - Si el operador te pide un movimiento y la autorización está inactiva, explícale con cortesía que requiere su autorización verbal primero.
2. ENLACE DISTRIBUIDO DDS: La Jetson Nano corre en un contenedor Docker en ROS_DOMAIN_ID=30. Si el enlace falla o no hay odometría, suele ser porque se arrancó sin 'DISTRIBUTED=1'. Usa 'check_system_and_dds_link' para diagnosticar.
3. PIEZAS Y MANIPULACIÓN:
   - Piezas oficiales: 'engranaje' (ArUco 1), 'poste' (ArUco 2), 'rueda' (ArUco 3).
   - Siempre que se proponga una toma de pieza, puedes ofrecer un ensayo en seco ('dry_run=True') antes del agarre real.
   - Poses del brazo: 'home' (plegado seguro), 'observe', 'carry', 'pick_table', 'place_table'.
4. PARADA DE EMERGENCIA: Si el operador dice "alto", "detente", "stop", "para", invoca INMEDIATAMENTE 'emergency_stop'.
5. ESTILO ORAL: Tus respuestas se emitirán por síntesis de voz (TTS). Sé conciso (1 a 3 frases fluidas), sin viñetas ni asteriscos en la respuesta hablada.
"""


class GeminiBrain:
    """Orchestrator powered by Gemini 1.5 Flash with structured tool calling."""

    def __init__(self, ros_bridge: JarvisRosBridge, api_key: Optional[str] = None):
        self.ros_bridge = ros_bridge
        self.api_key = api_key or config.gemini_api_key

        if not self.api_key:
            raise ValueError(
                "GEMINI_API_KEY no encontrada. Configure la variable: export GEMINI_API_KEY='tu_clave'"
            )

        import google.generativeai as genai

        genai.configure(api_key=self.api_key)

        # ---------------------------------------------------------------------
        # Declaración de Herramientas (Tools) Tipadas para Gemini
        # ---------------------------------------------------------------------
        def set_motion_authorization(enable: bool) -> str:
            """Habilita o deshabilita el protocolo de seguridad ALLOW_MOTION para autorizar o bloquear el accionamiento físico de ruedas y brazo.

            Args:
                enable: True para autorizar movimiento físico, False para bloquearlo en modo seguro.
            """
            print(f"\n[JARVIS Tool Call] -> set_motion_authorization(enable={enable})")
            res = self.ros_bridge.set_motion_authorization(enable)
            return json.dumps(res, ensure_ascii=False)

        def check_system_and_dds_link() -> str:
            """Verifica la salud del enlace de red DDS Unicast entre la PC y la Jetson Nano, comprobando que /odom esté publicando y que twist_mux esté activo (valida si la Jetson arrancó con DISTRIBUTED=1)."""
            print("\n[JARVIS Tool Call] -> check_system_and_dds_link()")
            res = self.ros_bridge.check_dds_link()
            return json.dumps(res, ensure_ascii=False)

        def navigate_to_location(location_name: str) -> str:
            """Envía el robot móvil myAGV a una ubicación conocida en el mapa de la pista (inicio, clasificacion, kitting, ensamblaje, laberinto, almacen, recarga).

            Args:
                location_name: Nombre de la estación o destino en el mapa.
            """
            print(f"\n[JARVIS Tool Call] -> navigate_to_location('{location_name}')")
            res = self.ros_bridge.navigate_to_named_location(location_name)
            return json.dumps(res, ensure_ascii=False)

        def navigate_to_coordinates(x: float, y: float, theta_deg: float = 0.0) -> str:
            """Envía el robot a coordenadas cartesianas arbitrarias (X, Y en metros, theta en grados) respetando el perímetro de seguridad.

            Args:
                x: Coordenada X en metros.
                y: Coordenada Y en metros.
                theta_deg: Orientación deseada en grados (-180 a 180).
            """
            print(f"\n[JARVIS Tool Call] -> navigate_to_coordinates(x={x}, y={y}, theta={theta_deg})")
            res = self.ros_bridge.send_goal_pose(x, y, theta_deg, target_label=f"Coords ({x:.2f}, {y:.2f})")
            return json.dumps(res, ensure_ascii=False)

        def approach_aruco_marker(marker_id: int, stop_distance: float = 0.20) -> str:
            """Inicia el servidor de aproximación visual y alineación LiDAR frontal hacia un marcador ArUco específico.

            Args:
                marker_id: Identificador numérico del marcador ArUco (0 a 9).
                stop_distance: Distancia de parada frontal en metros (por defecto 0.20 m).
            """
            print(f"\n[JARVIS Tool Call] -> approach_aruco_marker(id={marker_id}, stop={stop_distance}m)")
            res = self.ros_bridge.approach_aruco_marker(marker_id, stop_distance)
            return json.dumps(res, ensure_ascii=False)

        def move_arm_to_pose(pose_name: str) -> str:
            """Mueve el brazo robótico MechArm 270 a una postura calibrada ('home', 'observe', 'carry', 'pick_table', 'place_table').

            Args:
                pose_name: Nombre de la pose en poses.yaml.
            """
            print(f"\n[JARVIS Tool Call] -> move_arm_to_pose('{pose_name}')")
            res = self.ros_bridge.move_arm_to_pose(pose_name)
            return json.dumps(res, ensure_ascii=False)

        def control_gripper(aperture_percent: int) -> str:
            """Ajusta la apertura de la pinza adaptativa del MechArm 270.

            Args:
                aperture_percent: Porcentaje de apertura entre 0 (completamente cerrado) y 100 (totalmente abierto).
            """
            print(f"\n[JARVIS Tool Call] -> control_gripper({aperture_percent}%)")
            res = self.ros_bridge.set_gripper_aperture(aperture_percent)
            return json.dumps(res, ensure_ascii=False)

        def grasp_piece(piece_name: str, dry_run: bool = False) -> str:
            """Ejecuta o calcula la toma de una pieza de competencia ('engranaje', 'poste', 'rueda').

            Args:
                piece_name: Nombre de la pieza a tomar ('engranaje', 'poste', 'rueda').
                dry_run: Si es True, realiza solo el cálculo cinemático y verificación de alcance sin mover el robot ni el brazo.
            """
            print(f"\n[JARVIS Tool Call] -> grasp_piece('{piece_name}', dry_run={dry_run})")
            res = self.ros_bridge.grasp_piece_action(piece_name, dry_run=dry_run)
            return json.dumps(res, ensure_ascii=False)

        def get_robot_telemetry() -> str:
            """Consulta la posición actual (X, Y), orientación, velocidades, estado del enlace Wi-Fi y estado de autorización de movimiento."""
            print("\n[JARVIS Tool Call] -> get_robot_telemetry()")
            res = self.ros_bridge.get_telemetry()
            return json.dumps(res, ensure_ascii=False)

        def emergency_stop() -> str:
            """Detiene de inmediato y con prioridad absoluta el robot móvil myAGV y cancela cualquier meta activa de navegación o manipulación."""
            print("\n[JARVIS Tool Call] -> ¡EMERGENCY STOP!")
            res = self.ros_bridge.emergency_stop()
            return json.dumps(res, ensure_ascii=False)

        def cancel_current_goal() -> str:
            """Cancela la meta o trayectoria de navegación actual sin activar protocolo de choque crítico."""
            print("\n[JARVIS Tool Call] -> cancel_current_goal()")
            res = self.ros_bridge.cancel_current_goal()
            return json.dumps(res, ensure_ascii=False)

        def list_available_locations_and_pieces() -> str:
            """Devuelve la lista de ubicaciones del mapa y piezas del catálogo reconocidas por el sistema."""
            info = {
                "locations": self.ros_bridge.get_known_locations(),
                "pieces": {name: p.description for name, p in config.pieces.items()},
                "arm_poses": list(config.known_arm_poses),
            }
            return json.dumps(info, ensure_ascii=False)

        self.tools = [
            set_motion_authorization,
            check_system_and_dds_link,
            navigate_to_location,
            navigate_to_coordinates,
            approach_aruco_marker,
            move_arm_to_pose,
            control_gripper,
            grasp_piece,
            get_robot_telemetry,
            emergency_stop,
            cancel_current_goal,
            list_available_locations_and_pieces,
        ]

        self.model = genai.GenerativeModel(
            model_name=config.gemini_model,
            tools=self.tools,
            system_instruction=SYSTEM_INSTRUCTION,
            generation_config={
                "temperature": config.temperature,
                "max_output_tokens": 256,
            },
        )
        self.chat = self.model.start_chat(enable_automatic_function_calling=True)
        print(f"[GeminiBrain] Inicializado con {len(self.tools)} herramientas tácticas.")

    def process_user_query(self, user_text: str) -> str:
        if not user_text.strip():
            return ""
        try:
            response = self.chat.send_message(user_text)
            return response.text.strip()
        except Exception as e:
            err_msg = f"Señor, he detectado una anomalía al procesar su solicitud en mi núcleo de IA: {e}"
            print(f"[GeminiBrain] Error: {e}")
            return err_msg
