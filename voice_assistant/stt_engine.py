#!/usr/bin/env python3
"""Speech-to-Text (STT) transcription module.

Provides high-speed local transcription using faster-whisper or cloud-based
fallback using Google Speech Recognition via speech_recognition library.
"""

import io
import os
import tempfile
from typing import Optional
from voice_assistant.config import config


class FasterWhisperEngine:
    """Ultra-fast local STT based on CTranslate2 and Whisper."""

    def __init__(self, model_size: str = config.whisper_model_size, device: str = config.whisper_device):
        self.model_size = model_size
        self.device = device
        self.model = None
        self._load_model(self.device)

    def _load_model(self, device: str):
        from faster_whisper import WhisperModel
        compute_type = "int8" if device == "cpu" else "auto"
        print(f"[STT] Inicializando faster-whisper (modelo: {self.model_size}, device: {device}, compute_type: {compute_type})...")
        self.model = WhisperModel(self.model_size, device=device, compute_type=compute_type)
        self.device = device
        print(f"[STT] faster-whisper ({device}) cargado correctamente.")

    def _run_transcribe(self, temp_path: str) -> str:
        segments, info = self.model.transcribe(
            temp_path,
            beam_size=1,
            language="es",
            vad_filter=True,
        )
        text = " ".join([segment.text for segment in segments]).strip()
        return text

    def transcribe(self, wav_bytes: bytes) -> str:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tf:
            tf.write(wav_bytes)
            temp_path = tf.name

        try:
            return self._run_transcribe(temp_path)
        except Exception as e:
            err_msg = str(e)
            # Si falla por librerías CUDA/cuBLAS faltantes (ej. libcublas.so.12 en WSL)
            if self.device != "cpu" and any(k in err_msg.lower() for k in ["libcublas", "cuda", "cudnn"]):
                print(f"[STT] Aviso: GPU/CUDA no disponible o faltan librerías ({err_msg}).")
                print("[STT] Conmutando automáticamente faster-whisper a CPU (modo seguro)...")
                try:
                    self._load_model("cpu")
                    return self._run_transcribe(temp_path)
                except Exception as e_cpu:
                    print(f"[STT] Error en fallback de CPU: {e_cpu}")
                    raise
            raise
        finally:
            if os.path.exists(temp_path):
                os.remove(temp_path)


class GoogleSpeechEngine:
    """Lightweight fallback using Google Speech Recognition API (no GPU/model required)."""

    def __init__(self):
        import speech_recognition as sr
        self.recognizer = sr.Recognizer()

    def transcribe(self, wav_bytes: bytes) -> str:
        import speech_recognition as sr
        with io.BytesIO(wav_bytes) as audio_file:
            with sr.AudioFile(audio_file) as source:
                audio_data = self.recognizer.record(source)
                try:
                    text = self.recognizer.recognize_google(audio_data, language="es-ES")
                    return text.strip()
                except sr.UnknownValueError:
                    return ""
                except Exception as e:
                    print(f"[STT] Error en SpeechRecognition: {e}")
                    return ""


class STTEngine:
    """Universal Speech-To-Text wrapper with automatic backend selection."""

    def __init__(self, preferred_engine: str = config.stt_engine, device: str = config.whisper_device):
        self.backend = None
        self.engine_name = preferred_engine
        self.device = device

        if preferred_engine == "faster-whisper":
            try:
                self.backend = FasterWhisperEngine(device=device)
                self.engine_name = "faster-whisper"
            except Exception as e:
                print(f"[STT] Aviso: faster-whisper no disponible ({e}). Intentando fallback...")

        if self.backend is None:
            try:
                self.backend = GoogleSpeechEngine()
                self.engine_name = "speech_recognition"
                print("[STT] Usando motor de fallback: speech_recognition (Google)")
            except Exception as e:
                print(f"[STT] Error: Ningún motor STT disponible ({e}).")

    def transcribe(self, wav_bytes: Optional[bytes]) -> str:
        if not wav_bytes or self.backend is None:
            return ""
        try:
            return self.backend.transcribe(wav_bytes)
        except Exception as e:
            print(f"[STT] Error durante la transcripción con {self.engine_name}: {e}")
            # Si falló faster-whisper, conmutar en caliente a speech_recognition
            if self.engine_name != "speech_recognition":
                try:
                    print("[STT] Conmutando a Google Speech Recognition como respaldo...")
                    fallback = GoogleSpeechEngine()
                    text = fallback.transcribe(wav_bytes)
                    self.backend = fallback
                    self.engine_name = "speech_recognition"
                    return text
                except Exception as e_fb:
                    print(f"[STT] Error también en motor de respaldo: {e_fb}")
            return ""
