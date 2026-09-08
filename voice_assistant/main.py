#!/usr/bin/env python3
"""Main Entry Point for JARVIS Voice Assistant (myAGV Base Station).

Orchestrates audio capture with VAD, Speech-to-Text, Gemini Brain with Tool Calling,
ROS 2 Navigation Bridge, and Text-to-Speech playback.
"""

import argparse
import os
import signal
import sys
import time

from voice_assistant.config import config
from voice_assistant.audio_capture import AudioCapture
from voice_assistant.stt_engine import STTEngine
from voice_assistant.tts_engine import TTSEngine
from voice_assistant.gemini_brain import GeminiBrain
from voice_assistant.ros_bridge import RosBridgeManager


def parse_args():
    parser = argparse.ArgumentParser(description="Asistente por Voz JARVIS para myAGV")
    parser.add_argument(
        "--mode",
        choices=["voice", "text"],
        default="voice",
        help="Modo de interacción: 'voice' (micrófono con VAD) o 'text' (consola)",
    )
    parser.add_argument(
        "--stt",
        choices=["faster-whisper", "speech-recognition"],
        default=config.stt_engine,
        help="Motor de reconocimiento de voz",
    )
    parser.add_argument(
        "--whisper-device",
        choices=["cpu", "cuda", "auto"],
        default=config.whisper_device,
        help="Dispositivo de cómputo para faster-whisper ('cpu' recomendado en laptops/WSL)",
    )
    parser.add_argument(
        "--tts",
        choices=["edge-tts", "pyttsx3"],
        default=config.tts_engine,
        help="Motor de síntesis de voz",
    )
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="Habilita de inicio el accionamiento físico de motores (ALLOW_MOTION=1)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Modo de prueba simulado (sin requerir hardware ni conexión ROS activa)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    print("=" * 65)
    print("           J.A.R.V.I.S. - SISTEMA TÁCTICO myAGV           ")
    print("=" * 65)

    # 1. Validar clave de API de Gemini
    api_key = os.environ.get("GEMINI_API_KEY") or config.gemini_api_key
    if not api_key:
        print("\n[ERROR] No se encontró la variable GEMINI_API_KEY.")
        print("Por favor, ejecute: export GEMINI_API_KEY='tu_clave_de_google_ai'\n")
        sys.exit(1)

    # 2. Inicializar Puente ROS 2
    print("[1/5] Conectando con el entorno ROS 2 del myAGV...")
    bridge_mgr = None
    try:
        bridge_mgr = RosBridgeManager()
        ros_bridge = bridge_mgr.bridge_node
        print("[1/5] ROS 2 conectado exitosamente.")
    except Exception as e:
        print(f"[ERROR] No se pudo inicializar ROS 2: {e}")
        print("Asegúrate de haber hecho source de ROS 2 Humble antes de ejecutar.")
        sys.exit(1)

    # 3. Inicializar Motores de Audio y Voz
    print(f"[2/5] Configurando Text-to-Speech ({args.tts})...")
    tts = TTSEngine(engine=args.tts)

    audio_cap = None
    stt = None
    if args.mode == "voice":
        if not AudioCapture.is_available():
            print("\n" + "!" * 65)
            print("[AVISO] No se detectó micrófono o biblioteca de audio (sounddevice).")
            print("        Esto es habitual en WSL o máquinas sin micrófono conectado.")
            print("        Cambiando automáticamente a modo interactivo por texto (--mode text)...")
            print("!" * 65 + "\n")
            args.mode = "text"
        else:
            print(f"[3/5] Inicializando captura con VAD y STT ({args.stt} en {args.whisper_device})...")
            audio_cap = AudioCapture()
            stt = STTEngine(preferred_engine=args.stt, device=args.whisper_device)

    if args.mode == "text":
        print("[3/5] Modo texto activado. Ingrese sus comandos por consola.")

    # 4. Inicializar Cerebro Gemini
    print(f"[4/5] Despertando núcleo cognitivo Gemini ({config.gemini_model})...")
    try:
        brain = GeminiBrain(ros_bridge=ros_bridge, api_key=api_key)
        print("[4/5] Núcleo de IA y Function Calling en línea.")
    except Exception as e:
        print(f"[ERROR] Fallo al inicializar Gemini: {e}")
        if bridge_mgr:
            bridge_mgr.shutdown()
        sys.exit(1)

    # Manejo de parada limpia (Ctrl+C)
    def shutdown_handler(sig, frame):
        print("\n[JARVIS] Apagando sistemas y asegurando robot...")
        tts.speak("Desconectando enlace táctico, señor. Que tenga un buen día.", wait=True)
        if bridge_mgr:
            bridge_mgr.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    if args.allow_motion:
        config.allow_motion = True

    motion_text = "Actuadores autorizados y en línea." if config.allow_motion else "Movimiento físico bloqueado por seguridad (ALLOW_MOTION inactivo)."
    print(f"\n[Seguridad] Estado inicial: {motion_text}")

    # 5. Saludo Inicial de JARVIS
    greeting = f"Sistemas en línea, señor. Conexión establecida en el dominio 30. {motion_text} A la espera de sus órdenes."
    tts.speak(greeting, wait=False)

    print("\n" + "-" * 65)
    print("JARVIS está listo para recibir comandos.")
    print("Comandos de ejemplo:")
    print(" - 'JARVIS, realiza un diagnóstico del enlace DDS'")
    print(" - 'JARVIS, autorizo movimiento' / 'JARVIS, bloquea movimiento'")
    print(" - 'Lleva el robot a la estación de kitting'")
    print(" - 'Realiza un ensayo en seco para tomar el engranaje'")
    print(" - 'Aproxima al marcador ArUco 0'")
    print(" - 'Mueve el brazo a la postura home'")
    print(" - '¡JARVIS, alto el fuego! ¡Detén el robot ahora mismo!'")
    print("-" * 65 + "\n")

    # Bucle Principal de Interacción
    while True:
        try:
            user_text = ""
            if args.mode == "voice":
                print("\n[Escuchando...] (Hable al micrófono. Se detendrá al detectar silencio)")
                audio_bytes = audio_cap.record_utterance()
                if audio_bytes is None:
                    continue

                print("[Procesando voz con STT...]")
                user_text = stt.transcribe(audio_bytes)
                if not user_text.strip():
                    continue
            else:
                user_text = input("\n[Operador] > ").strip()
                if not user_text:
                    continue

            print(f"\n[Usuario]: \"{user_text}\"")

            # Procesar con Gemini (invocará las Tools de ROS si es necesario)
            jarvis_response = brain.process_user_query(user_text)

            # Sintetizar y reproducir respuesta
            tts.speak(jarvis_response, wait=True)

        except KeyboardInterrupt:
            shutdown_handler(None, None)
        except Exception as e:
            print(f"[ERROR en bucle principal]: {e}")
            time.sleep(1.0)


if __name__ == "__main__":
    main()
