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

        self.declare_parameter("publish_topic", "/rawAudioChunk")
        self.declare_parameter("MIC_DEVICE_NAME", "default")
        self.declare_parameter("MIC_INPUT_CHANNELS", 32)
        self.declare_parameter("MIC_OUT_CHANNELS", 32)
        self.declare_parameter("CHUNK_SIZE", 1024)
        self.declare_parameter("ENABLE_ANC", True)
        self.declare_parameter("GAIN", 1.0)

        self.enable_anc = self.get_parameter("ENABLE_ANC").value
        self.chunk_size = self.get_parameter("CHUNK_SIZE").value
        self.gain = self.get_parameter("GAIN").value
        self.use_respeaker = SpeechApiUtils.respeaker_available()
        self.RATE = 16000

        self.df_model = None
        self.df_state = None
        self.df_ready = False  # True when model is ready
        self.use_df = bool(DF_AVAILABLE and self.enable_anc)

        self.publisher_ = self.create_publisher(
            AudioData, self.get_parameter("publish_topic").value, 20
        )

        mic_name = self.get_parameter("MIC_DEVICE_NAME").value
        in_ch = self.get_parameter("MIC_INPUT_CHANNELS").value
        out_ch = self.get_parameter("MIC_OUT_CHANNELS").value
        self.input_device_index = SpeechApiUtils.getIndexByNameAndChannels(
            mic_name, in_ch, out_ch
        )
        if self.use_df:
            self.get_logger().info(
                "Starting DeepFilterNet in background thread. ANC will be active once ready..."
            )
            threading.Thread(target=self._init_df_async, daemon=True).start()
        else:
            self.get_logger().info("ENABLE_ANC=False or DeepFilterNet not available.")

    def _init_df_async(self):
        try:
            self.df_model, self.df_state, _ = DF_MODULE.init_df()
            if torch.cuda.is_available():
                self.df_model = self.df_model.to("cuda")
                self.get_logger().info("CUDA enabled for noise suppression.")
            self.df_ready = True  # Mark ready AFTER moving model to device
            self.get_logger().info("DeepFilterNet ready. Noise suppression active.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize DeepFilterNet: {e}")
            self.use_df = False

    def enhance_audio(self, audio_int16):
        """Process audio: 16k -> 48k -> AI -> 16k -> Gain"""
        if not self.use_df or not self.df_ready or self.df_model is None:
            return audio_int16

        try:
            # 1. Normalize to float32
            audio_float = audio_int16.astype(np.float32) / 32768.0

            # 2. High-quality resampling (16k -> 48k)
            audio_48k = scipy.signal.resample_poly(audio_float, 3, 1)

            # 3. Prepare Tensor on device (GPU/CPU)
            device = next(self.df_model.parameters()).device
            tensor = torch.from_numpy(audio_48k).unsqueeze(0).to(device)

            # 4. Neural processing
            with torch.no_grad():
                enhanced_48k_t = DF_MODULE.enhance(self.df_model, self.df_state, tensor)

            enhanced_48k = enhanced_48k_t.cpu().numpy().squeeze()

            # 5. Downsample (48k -> 16k)
            enhanced_16k = scipy.signal.resample_poly(enhanced_48k, 1, 3)

            output_float = enhanced_16k * self.gain

            return np.clip(output_float * 32768.0, -32768, 32767).astype(np.int16)
        except Exception as e:
            self.get_logger().error(f"Error processing frame with DF: {e}")
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
                f"--- Listening on device index {self.input_device_index} ---"
            )

            while rclpy.ok():
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                if not data:
                    continue

                audio_arr = np.frombuffer(data, dtype=np.int16)

                # If ReSpeaker, extract channel 0 (main microphone)
                if self.use_respeaker:
                    audio_arr = audio_arr[0::6]

                # Apply noise suppression if enabled
                if self.enable_anc:
                    audio_arr = self.enhance_audio(audio_arr)

                self.publisher_.publish(AudioData(data=audio_arr.tobytes()))

        except Exception as e:
            self.get_logger().error(f"Critical error in PyAudio: {e}")
        finally:
            self.get_logger().info("Closing AudioCapturer...")
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
