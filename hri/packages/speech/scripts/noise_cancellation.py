#!/usr/bin/env python3

import threading

import numpy as np
import rclpy
import scipy.signal
import torch
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

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


class NoiseCancellation(Node):
    def __init__(self):
        super().__init__("noise_cancellation")

        self.declare_parameter("INPUT_TOPIC", "/rawAudioChunk")
        self.declare_parameter("PROCESSED_AUDIO_TOPIC", "/hri/processedAudioChunk")
        self.declare_parameter("ENABLE_ANC", True)
        self.declare_parameter("GAIN", 1.0)

        input_topic = self.get_parameter("INPUT_TOPIC").value
        output_topic = self.get_parameter("PROCESSED_AUDIO_TOPIC").value
        self.enable_anc = self.get_parameter("ENABLE_ANC").value
        self.gain = self.get_parameter("GAIN").value

        self.df_model = None
        self.df_state = None
        self.df_ready = False
        self.use_df = bool(DF_AVAILABLE and self.enable_anc)

        self.publisher_ = self.create_publisher(AudioData, output_topic, 20)
        self.create_subscription(AudioData, input_topic, self._audio_callback, 20)

        self.get_logger().info(
            f"NoiseCancellation node ready. {input_topic} → {output_topic}"
        )

        if self.use_df:
            self.get_logger().info(
                "Starting DeepFilterNet in background thread. ANC will be active once ready..."
            )
            threading.Thread(target=self._init_df_async, daemon=True).start()
        else:
            self.get_logger().info(
                "ENABLE_ANC=False or DeepFilterNet not available. Passing audio through."
            )

    def _init_df_async(self):
        try:
            self.df_model, self.df_state, _ = DF_MODULE.init_df()
            if torch.cuda.is_available():
                self.df_model = self.df_model.to("cuda")
                self.get_logger().info("CUDA enabled for noise suppression.")
            self.df_ready = True
            self.get_logger().info("DeepFilterNet ready. Noise suppression active.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize DeepFilterNet: {e}")
            self.use_df = False

    def _audio_callback(self, msg):
        audio_arr = np.frombuffer(msg.data, dtype=np.int16)

        if self.use_df and self.df_ready:
            audio_arr = self._enhance(audio_arr)
        elif self.gain != 1.0:
            audio_arr = np.clip(
                audio_arr.astype(np.float32) * self.gain, -32768, 32767
            ).astype(np.int16)

        self.publisher_.publish(AudioData(data=audio_arr.tobytes()))

    def _enhance(self, audio_int16):
        """Process audio: 16k -> 48k -> DeepFilterNet -> 16k -> Gain"""
        try:
            # 1. Normalize to float32
            audio_float = audio_int16.astype(np.float32) / 32768.0

            # 2. Resample 16k -> 48k
            audio_48k = scipy.signal.resample_poly(audio_float, 3, 1)

            # 3. Prepare tensor on device
            device = next(self.df_model.parameters()).device
            tensor = torch.from_numpy(audio_48k).unsqueeze(0).to(device)

            # 4. Neural noise suppression
            with torch.no_grad():
                enhanced_48k_t = DF_MODULE.enhance(self.df_model, self.df_state, tensor)

            enhanced_48k = enhanced_48k_t.cpu().numpy().squeeze()

            # 5. Resample 48k -> 16k
            enhanced_16k = scipy.signal.resample_poly(enhanced_48k, 1, 3)

            return np.clip(enhanced_16k * self.gain * 32768.0, -32768, 32767).astype(
                np.int16
            )
        except Exception as e:
            self.get_logger().error(f"Error enhancing audio frame: {e}")
            return audio_int16


def main(args=None):
    rclpy.init(args=args)
    node = NoiseCancellation()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
