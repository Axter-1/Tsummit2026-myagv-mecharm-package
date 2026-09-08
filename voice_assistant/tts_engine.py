#!/usr/bin/env python3
"""Text-to-Speech (TTS) module for JARVIS voice output.

Generates high-fidelity neural speech using edge-tts with offline fallback via pyttsx3,
playing the audio directly through the host PC speakers.
"""

import asyncio
import os
import shutil
import subprocess
import tempfile
from typing import Optional

from voice_assistant.config import config


class TTSEngine:
    """Text to Speech engine supporting edge-tts and pyttsx3."""

    def __init__(
        self,
        engine: str = config.tts_engine,
        voice: str = config.tts_voice,
        rate: str = config.tts_rate,
    ):
        self.engine_type = engine
        self.voice = voice
        self.rate = rate
        self.pyttsx_engine = None

        # Detectar reproductor disponible en el sistema
        self.player_cmd = None
        for cmd in ["mpv", "ffplay"]:
            if shutil.which(cmd):
                self.player_cmd = cmd
                break

        if self.engine_type == "pyttsx3" or self.player_cmd is None:
            self._init_pyttsx3()

    def _init_pyttsx3(self):
        try:
            import pyttsx3
            self.pyttsx_engine = pyttsx3.init()
            self.pyttsx_engine.setProperty("rate", 175)
        except Exception as e:
            print(f"[TTS] pyttsx3 no disponible: {e}")

    async def _edge_tts_synthesize(self, text: str, output_path: str):
        import edge_tts
        communicate = edge_tts.Communicate(text, self.voice, rate=self.rate)
        await communicate.save(output_path)

    def speak(self, text: str, wait: bool = True):
        """Synthesizes text and plays it via PC speakers.

        Args:
            text: Text to speak.
            wait: Whether to block until playback finishes.
        """
        if not text.strip():
            return

        print(f"\n[JARVIS]: {text}")

        # Intentar síntesis neural con edge-tts si hay reproductor de medios
        if self.engine_type == "edge-tts" and self.player_cmd:
            try:
                with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tf:
                    temp_mp3 = tf.name

                asyncio.run(self._edge_tts_synthesize(text, temp_mp3))

                if self.player_cmd == "mpv":
                    cmd = ["mpv", "--no-terminal", "--really-quiet", temp_mp3]
                elif self.player_cmd == "ffplay":
                    cmd = ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", temp_mp3]
                else:
                    cmd = ["play", temp_mp3]

                proc = subprocess.Popen(cmd)
                if wait:
                    proc.wait()
                    if os.path.exists(temp_mp3):
                        os.remove(temp_mp3)
                return
            except Exception as e:
                print(f"[TTS] Aviso: edge-tts falló ({e}). Usando fallback...")

        # Fallback con pyttsx3 local
        if self.pyttsx_engine:
            try:
                self.pyttsx_engine.say(text)
                self.pyttsx_engine.runAndWait()
            except Exception as e:
                print(f"[TTS] pyttsx3 error: {e}")
