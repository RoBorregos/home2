#!/usr/bin/env python3

import threading
import numpy as np
import pyaudio
import rclpy
import torch
import scipy.signal
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils
from frida_interfaces.msg import AudioData

# Intentar importar DeepFilterNet
DF_AVAILABLE = False
DF_MODULE = None
try:
    import df as df_module

    DF_MODULE = df_module
    DF_AVAILABLE = True
except ImportError:
    try:
        import deepfilternet as df_module

        DF_MODULE = df_module
        DF_AVAILABLE = True
    except ImportError:
        DF_AVAILABLE = False


class AudioCapturer(Node):
    def __init__(self):
        super().__init__("audio_capturer")

        # Parámetros ROS2
        self.declare_parameter("publish_topic", "/rawAudioChunk")
        self.declare_parameter("MIC_DEVICE_NAME", "default")
        self.declare_parameter("MIC_INPUT_CHANNELS", 32)
        self.declare_parameter("MIC_OUT_CHANNELS", 32)
        self.declare_parameter("CHUNK_SIZE", 1024)
        self.declare_parameter("ENABLE_ANC", True)
        self.declare_parameter(
            "GAIN", 1.0
        )  # Boost de amplitud post-DF (1.0 = sin cambio)

        self.enable_anc = self.get_parameter("ENABLE_ANC").value
        self.chunk_size = self.get_parameter("CHUNK_SIZE").value
        self.gain = self.get_parameter("GAIN").value
        self.use_respeaker = SpeechApiUtils.respeaker_available()
        self.RATE = 16000

        # DeepFilterNet state — inicializado en segundo plano
        self.df_model = None
        self.df_state = None
        self.df_ready = False  # True cuando el modelo está listo
        self.use_df = bool(DF_AVAILABLE and self.enable_anc)

        # Publisher y dispositivo se configuran ANTES de cargar DF
        # para que record() arranque de inmediato sin esperar al modelo.
        self.publisher_ = self.create_publisher(
            AudioData, self.get_parameter("publish_topic").value, 20
        )

        mic_name = self.get_parameter("MIC_DEVICE_NAME").value
        in_ch = self.get_parameter("MIC_INPUT_CHANNELS").value
        out_ch = self.get_parameter("MIC_OUT_CHANNELS").value
        self.input_device_index = SpeechApiUtils.getIndexByNameAndChannels(
            mic_name, in_ch, out_ch
        )

        # Inicializar DeepFilterNet en segundo plano para no bloquear record()
        if self.use_df:
            self.get_logger().info(
                "Iniciando DeepFilterNet en segundo plano (esto puede tardar)..."
            )
            threading.Thread(target=self._init_df_async, daemon=True).start()
        else:
            self.get_logger().info(
                "ENABLE_ANC=False o DF no disponible — ANC desactivado."
            )

    def _init_df_async(self):
        """Carga DeepFilterNet en un hilo separado para no bloquear record()."""
        try:
            self.df_model, self.df_state, _ = DF_MODULE.init_df()
            if torch.cuda.is_available():
                self.df_model = self.df_model.to("cuda")
                self.get_logger().info("CUDA activado para supresión de ruido.")
            self.df_ready = True  # Marcar listo DESPUÉS de mover el modelo al device
            self.get_logger().info("DeepFilterNet listo. Supresión de ruido activa.")
        except Exception as e:
            self.get_logger().error(f"Fallo al inicializar DeepFilterNet: {e}")
            self.use_df = False

    def enhance_audio(self, audio_int16):
        """Procesa audio: 16k -> 48k -> IA -> 16k -> Ganancia"""
        if not self.use_df or not self.df_ready or self.df_model is None:
            return audio_int16

        try:
            # 1. Normalización a float32
            audio_float = audio_int16.astype(np.float32) / 32768.0

            # 2. Resampling de alta calidad (16k -> 48k)
            # Esto evita que la IA confunda el audio con ruido digital
            audio_48k = scipy.signal.resample_poly(audio_float, 3, 1)

            # 3. Preparar Tensor en dispositivo (GPU/CPU)
            device = next(self.df_model.parameters()).device
            tensor = torch.from_numpy(audio_48k).unsqueeze(0).to(device)

            # 4. Procesamiento neuronal
            with torch.no_grad():
                enhanced_48k_t = DF_MODULE.enhance(self.df_model, self.df_state, tensor)

            enhanced_48k = enhanced_48k_t.cpu().numpy().squeeze()

            # 5. Downsample (48k -> 16k)
            enhanced_16k = scipy.signal.resample_poly(enhanced_48k, 1, 3)

            # Ganancia de compensación: Whisper necesita buena amplitud para detectar voz.
            output_float = enhanced_16k * self.gain

            return np.clip(output_float * 32768.0, -32768, 32767).astype(np.int16)
        except Exception as e:
            self.get_logger().error(f"Error procesando frame con DF: {e}")
            return audio_int16

    def record(self):
        p = pyaudio.PyAudio()
        channels = 6 if self.use_respeaker else 1

        try:
            stream = p.open(
                input_device_index=self.input_device_index,
                format=pyaudio.paInt16,
                channels=channels,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.chunk_size,
            )

            self.get_logger().info(
                f"--- Escuchando en índice {self.input_device_index} ---"
            )

            while rclpy.ok():
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                if not data:
                    continue

                audio_arr = np.frombuffer(data, dtype=np.int16)

                # Si es Respeaker, extraer canal 0 (micrófono principal)
                if self.use_respeaker:
                    audio_arr = audio_arr[0::6]

                # Aplicar limpieza si está activada
                if self.enable_anc:
                    audio_arr = self.enhance_audio(audio_arr)

                self.publisher_.publish(AudioData(data=audio_arr.tobytes()))

        except Exception as e:
            self.get_logger().error(f"Error crítico en PyAudio: {e}")
        finally:
            self.get_logger().info("Cerrando AudioCapturer...")
            try:
                stream.stop_stream()
                stream.close()
            except Exception:
                pass
            p.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = AudioCapturer()
    try:
        node.record()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
