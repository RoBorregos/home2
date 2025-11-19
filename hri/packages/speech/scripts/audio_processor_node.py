#!/usr/bin/env python3
"""ROS2 node that subscribes to raw audio chunks and publishes processed chunks.

Listens to `input_topic` (AudioData), applies a configurable pipeline and republishes
to `output_topic` as AudioData.
"""
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from frida_interfaces.msg import AudioData
from speech.audio_processing import (
    reduce_noise,
    regulate_gain,
    compress_dynamic_range,
    adaptive_agc,
    dereverb_spectral,
    write_wav,
)


class AudioProcessorNode(Node):
    def __init__(self):
        super().__init__("audio_processor")
        self.declare_parameter("input_topic", "/rawAudioChunk")
        self.declare_parameter("output_topic", "/processedAudioChunk")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("pipeline", ["reduce_noise", "regulate_gain"])
        self.declare_parameter("debug_dir", "/tmp/speech_audio")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.sr = self.get_parameter("sample_rate").get_parameter_value().integer_value
        self.pipeline = self.get_parameter("pipeline").get_parameter_value().string_array_value
        self.debug_dir = self.get_parameter("debug_dir").get_parameter_value().string_value

        os.makedirs(self.debug_dir, exist_ok=True)

        self.pub = self.create_publisher(AudioData, output_topic, 10)
        self.sub = self.create_subscription(AudioData, input_topic, self.cb, 20)

        self.counter = 0
        self.get_logger().info(f"AudioProcessor initialized: in={input_topic} out={output_topic}")

    def cb(self, msg: AudioData):
        # msg.data is bytes of int16 PCM
        try:
            arr = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        except Exception:
            self.get_logger().warning("Failed to parse incoming audio data as int16")
            return

        y = arr
        # apply configured pipeline
        if "reduce_noise" in self.pipeline:
            try:
                y = reduce_noise(y, self.sr)
            except Exception as e:
                self.get_logger().warning(f"reduce_noise failed: {e}")

        if "regulate_gain" in self.pipeline:
            y = regulate_gain(y, target_rms=0.05)

        if "compress" in self.pipeline:
            y = compress_dynamic_range(y, threshold_db=-20.0, ratio=2.0)

        if "agc" in self.pipeline:
            y, _ = adaptive_agc(y, target_rms=0.05)

        if "dereverb" in self.pipeline:
            try:
                y = dereverb_spectral(y, self.sr)
            except Exception as e:
                self.get_logger().warning(f"dereverb failed: {e}")

        # write debug file every 200 messages
        self.counter += 1
        if self.counter % 200 == 0:
            ts = int(time.time())
            path = os.path.join(self.debug_dir, f"processed_{ts}.wav")
            try:
                write_wav(path, y, self.sr)
                self.get_logger().info(f"Wrote debug wav: {path}")
            except Exception as e:
                self.get_logger().warning(f"Failed to write debug wav: {e}")

        # convert back to int16 bytes
        y_out = np.clip(y, -1.0, 1.0) * 32767.0
        y_out = y_out.astype(np.int16).tobytes()
        out_msg = AudioData()
        out_msg.data = bytes(y_out)
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down audio_processor")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
