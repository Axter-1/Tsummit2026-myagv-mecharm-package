#!/usr/bin/env python3
"""Configuration module for JARVIS Voice Assistant (myAGV Base Station)."""

import os
from dataclasses import dataclass, field
from typing import Dict

@dataclass
class Waypoint:
    x: float
    y: float
    yaw_deg: float
    description: str

@dataclass
class PieceGraspInfo:
    name: str
    target_aruco_id: int
    span_mm: float
    gripper_open: int
    gripper_close: int
    grasp_z_mm: float
    description: str

@dataclass
class JarvisConfig:
    # --- LLM / Gemini Settings ---
    gemini_model: str = "gemini-3.8-flash"
    gemini_api_key: str = field(
        default_factory=lambda: os.environ.get("GEMINI_API_KEY", "AIzaSyCbe1HyDs9kbXhoFpmiif26GfdTHdz6iAE")
    )
    temperature: float = 0.2

    # --- Regla de Oro de Seguridad del myAGV ---
    # En consonancia con el runbook: todo movimiento exige ALLOW_MOTION=1
    allow_motion: bool = field(
        default_factory=lambda: os.environ.get("ALLOW_MOTION", "0") == "1"
    )

    # --- Waypoints Predefinidos del Mapa (T-SUMMIT) ---
    locations: Dict[str, Waypoint] = field(default_factory=lambda: {
        "inicio": Waypoint(0.0, 0.0, 0.0, "START: Zona de inicio / Home base"),
        "start": Waypoint(0.0, 0.0, 0.0, "START: Zona de inicio / Home base"),
        "clasificacion": Waypoint(1.5, 0.8, 0.0, "Cuadrante 1: Área de Clasificación (KOSTAL)"),
        "kitting": Waypoint(3.2, 0.8, 90.0, "Cuadrante 2: Área de Kitting (DENSO)"),
        "ensamblaje": Waypoint(1.5, -1.5, -90.0, "Cuadrante 3: Área de Ensamblaje (MICHELIN)"),
        "laberinto": Waypoint(3.7, -2.2, 0.0, "Cuadrante 4: FINISH del Laberinto (VCST)"),
        "almacen": Waypoint(0.5, 2.0, 180.0, "Estación de depósito / Almacén de suministros"),
        "recarga": Waypoint(0.0, -0.5, 0.0, "Estación de recarga y pits"),
    })

    # --- Catálogo de Piezas y Agarre (MechArm 270 M5) ---
    pieces: Dict[str, PieceGraspInfo] = field(default_factory=lambda: {
        "engranaje": PieceGraspInfo("engranaje", 1, 32.8, 89, 62, 98.0, "Engranaje Ø152.8mm - Pinza radial en alma anular"),
        "poste": PieceGraspInfo("poste", 2, 20.0, 71, 33, 160.0, "Poste base Ø110x20mm con tubo central Ø20mm"),
        "rueda": PieceGraspInfo("rueda", 3, 20.0, 71, 33, 105.0, "Rueda aro Ø150mm - Pinzado de pie rectangular 30x30mm"),
    })

    # Poses oficiales del brazo (poses.yaml)
    known_arm_poses: tuple = ("home", "observe", "carry", "pick_table", "place_table")

    # --- Geofencing & Safety ---
    x_min: float = -2.0
    x_max: float = 10.0
    y_min: float = -5.0
    y_max: float = 5.0

    # --- Timeouts y Tópicos ROS 2 ---
    nav_timeout_sec: float = 90.0
    odom_timeout_sec: float = 2.0
    nav_action_name: str = "navigate_to_pose"
    aruco_action_name: str = "/aruco_lidar_approach"
    arm_move_action: str = "/mecharm/move_arm"
    arm_pick_action: str = "/mecharm/pick_place"
    gripper_service: str = "/mecharm/set_gripper"
    free_move_service: str = "/mecharm/free_move"

    odom_topic: str = "/odom"
    cmd_vel_topic: str = "/cmd_vel"
    cmd_vel_aruco_topic: str = "/cmd_vel_aruco"

    # --- Audio / VAD Settings ---
    sample_rate: int = 16000
    audio_channels: int = 1
    chunk_size: int = 480
    vad_silence_duration_sec: float = 0.8
    vad_min_speech_duration_sec: float = 0.4
    vad_max_recording_sec: float = 15.0
    vad_energy_threshold_multiplier: float = 2.2

    # --- STT & TTS Settings ---
    stt_engine: str = "faster-whisper"
    whisper_model_size: str = "base"
    whisper_device: str = os.environ.get("WHISPER_DEVICE", "cpu")
    tts_engine: str = "edge-tts"
    tts_voice: str = "es-ES-AlvaroNeural"
    tts_rate: str = "+4%"

config = JarvisConfig()
