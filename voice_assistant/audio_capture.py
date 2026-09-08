#!/usr/bin/env python3
"""Audio capture module with Voice Activity Detection (VAD).

Captures microphone audio on the host PC and identifies when the user starts
and finishes speaking using energy-based Voice Activity Detection.
"""

import io
import time
import wave
import numpy as np
from typing import Optional

try:
    import sounddevice as sd
    HAS_SOUNDDEVICE = True
except (ImportError, OSError):
    HAS_SOUNDDEVICE = False
    sd = None

from voice_assistant.config import config


class AudioCapture:
    """Microphone audio capturer with automatic Voice Activity Detection."""

    def __init__(
        self,
        sample_rate: int = config.sample_rate,
        chunk_size: int = config.chunk_size,
        silence_timeout_sec: float = config.vad_silence_duration_sec,
        min_speech_sec: float = config.vad_min_speech_duration_sec,
        max_record_sec: float = config.vad_max_recording_sec,
    ):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.silence_timeout_sec = silence_timeout_sec
        self.min_speech_sec = min_speech_sec
        self.max_record_sec = max_record_sec

    @classmethod
    def is_available(cls) -> bool:
        """Verifica si sounddevice está instalado y si hay al menos un micrófono disponible."""
        if not HAS_SOUNDDEVICE or sd is None:
            return False
        try:
            devices = sd.query_devices()
            if not devices:
                return False
            inputs = [d for d in devices if d.get("max_input_channels", 0) > 0]
            return len(inputs) > 0
        except Exception:
            return False

    def _compute_rms(self, audio_chunk: np.ndarray) -> float:
        """Calculates Root Mean Square (RMS) energy of an audio frame."""
        if len(audio_chunk) == 0:
            return 0.0
        return float(np.sqrt(np.mean(np.square(audio_chunk.astype(np.float32)))))

    def calibrate_noise_floor(self, duration_sec: float = 0.5) -> float:
        """Samples ambient room noise to compute baseline energy threshold."""
        if not self.is_available():
            return 50.0

        samples_to_read = int(self.sample_rate * duration_sec)
        try:
            recording = sd.rec(
                samples_to_read,
                samplerate=self.sample_rate,
                channels=1,
                dtype="int16",
                blocking=True,
            )
            rms = self._compute_rms(recording)
            return max(rms, 25.0)
        except Exception as e:
            print(f"[AudioCapture] Advertencia al calibrar micrófono: {e}")
            return 50.0

    def record_utterance(self) -> Optional[bytes]:
        """Listens to the microphone until user stops speaking (VAD).

        Returns:
            bytes: Audio content encoded in standard 16kHz mono WAV format,
                   or None if no speech was detected.
        """
        if not self.is_available():
            print("[AudioCapture] 'sounddevice' o dispositivo de micrófono no disponible. Modo texto requerido.")
            return None

        noise_floor = self.calibrate_noise_floor(0.4)
        threshold = noise_floor * config.vad_energy_threshold_multiplier
        print(f"[AudioCapture] Listo. Escuchando... (Ruido base: {noise_floor:.1f}, Umbral voz: {threshold:.1f})")

        frames = []
        is_speaking = False
        silence_start_time = None
        start_time = time.time()
        speech_start_time = None

        # Bloque de streaming con baja latencia
        try:
            with sd.InputStream(
                samplerate=self.sample_rate,
                channels=1,
                dtype="int16",
                blocksize=self.chunk_size,
            ) as stream:
                while True:
                    chunk, overflowed = stream.read(self.chunk_size)
                    now = time.time()
                    energy = self._compute_rms(chunk)

                    if energy > threshold:
                        if not is_speaking:
                            is_speaking = True
                            speech_start_time = now
                            print("[AudioCapture] Voz detectada... Grabando.")
                        silence_start_time = None
                        frames.append(chunk.copy())
                    else:
                        if is_speaking:
                            frames.append(chunk.copy())
                            if silence_start_time is None:
                                silence_start_time = now
                            elif (now - silence_start_time) >= self.silence_timeout_sec:
                                # Silencio sostenido detectado tras haber hablado
                                speech_duration = now - speech_start_time
                                if speech_duration >= self.min_speech_sec:
                                    print(f"[AudioCapture] Fin de locución detectado ({speech_duration:.1f}s).")
                                    break
                                else:
                                    # Ruido espurio muy corto, descartar
                                    is_speaking = False
                                    silence_start_time = None
                                    frames.clear()

                    # Timeout de seguridad global
                    if is_speaking and (now - start_time) >= self.max_record_sec:
                        print("[AudioCapture] Límite de tiempo alcanzado.")
                        break

                    # Si no ha empezado a hablar tras 20s, salir para no colgar el proceso
                    if not is_speaking and (now - start_time) >= 25.0:
                        return None
        except Exception as e:
            print(f"[AudioCapture] Error al acceder al micrófono: {e}")
            return None

        if not frames:
            return None

        # Empaquetar a WAV en memoria
        audio_data = np.concatenate(frames, axis=0)
        wav_io = io.BytesIO()
        with wave.open(wav_io, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 16-bit
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_data.tobytes())

        wav_io.seek(0)
        return wav_io.read()
