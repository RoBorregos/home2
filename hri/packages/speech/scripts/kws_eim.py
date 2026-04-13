#!/usr/bin/env python3
"""
Keyword Spotting node using Edge Impulse .eim runner.
Replaces OpenWakeWord with an Edge Impulse classification model.
Classes: yes, no, stop, frida, noise
"""

import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from frida_interfaces.msg import AudioData

try:
    from edge_impulse_linux.runner import ImpulseRunner
except ImportError:
    ImpulseRunner = None


class EdgeImpulseKWSNode(Node):
    def __init__(self):
        super().__init__("kws_eim")
        self.get_logger().info("Initializing Edge Impulse KWS node.")

        self.declare_parameter(
            "model_path",
            "/workspace/src/hri/packages/speech/assets/oww-runner-linux-aarch64-jetson-orin-6-0-v3-impulse-1.eim",
        )
        self.declare_parameter("PROCESSED_AUDIO_TOPIC", "/hri/processedAudioChunk")
        self.declare_parameter("WAKEWORD_TOPIC", "/speech/oww")
        self.declare_parameter("detection_cooldown", 1.0)
        self.declare_parameter("SENSITIVITY_THRESHOLD", 0.7)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("noise_label", "noise")

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        audio_topic = (
            self.get_parameter("PROCESSED_AUDIO_TOPIC")
            .get_parameter_value()
            .string_value
        )
        wakeword_topic = (
            self.get_parameter("WAKEWORD_TOPIC").get_parameter_value().string_value
        )
        self.detection_cooldown = (
            self.get_parameter("detection_cooldown").get_parameter_value().double_value
        )
        self.sensitivity_threshold = (
            self.get_parameter("SENSITIVITY_THRESHOLD")
            .get_parameter_value()
            .double_value
        )
        self.sample_rate = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )
        self.noise_label = (
            self.get_parameter("noise_label").get_parameter_value().string_value
        )

        # Initialize Edge Impulse runner
        if ImpulseRunner is None:
            self.get_logger().error(
                "edge_impulse_linux package not installed. "
                "Install with: pip install edge_impulse_linux"
            )
            raise RuntimeError("edge_impulse_linux not available")

        self.get_logger().info(f"Loading EIM model from: {model_path}")

        # Make sure the .eim file is executable
        if os.path.exists(model_path):
            os.chmod(model_path, 0o755)
        else:
            self.get_logger().error(f"Model file not found: {model_path}")
            raise FileNotFoundError(f"EIM model not found: {model_path}")

        self.runner = ImpulseRunner(model_path)
        model_info = self.runner.init()

        self.get_logger().info(f"Model info: {json.dumps(model_info, indent=2)}")

        # Extract model parameters
        self.labels = model_info["model_parameters"]["labels"]
        self.input_features_count = model_info["model_parameters"][
            "input_features_count"
        ]
        model_sample_rate = model_info["model_parameters"].get("frequency", 16000)
        self.window_size_ms = model_info["model_parameters"].get("window_size_ms", 1000)
        self.window_size_samples = int(model_sample_rate * self.window_size_ms / 1000)

        self.get_logger().info(
            f"Labels: {self.labels}, "
            f"Input features: {self.input_features_count}, "
            f"Window size: {self.window_size_ms}ms ({self.window_size_samples} samples)"
        )

        # Audio buffer to accumulate samples (int16 as expected by EIM runner)
        self.audio_buffer = np.array([], dtype=np.int16)

        # Publisher and subscriber
        self.publisher = self.create_publisher(String, wakeword_topic, 10)
        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)

        self.last_detection_time = time.time() - self.detection_cooldown
        self.get_logger().info("Edge Impulse KWS node initialized and ready.")

    def audio_callback(self, msg):
        """Process incoming audio data and detect keywords."""
        # Keep audio as int16 (Edge Impulse runner expects raw int16 samples)
        audio_data = np.frombuffer(msg.data, dtype=np.int16)

        # Append to buffer
        self.audio_buffer = np.concatenate([self.audio_buffer, audio_data])

        # Process when we have enough samples for the model's window
        if len(self.audio_buffer) >= self.window_size_samples:
            # Take the most recent window
            window = self.audio_buffer[-self.window_size_samples :]

            # Run classification
            try:
                results = self.runner.classify(window.tolist())
            except Exception as e:
                self.get_logger().error(f"Classification error: {e}")
                self.audio_buffer = self.audio_buffer[-self.window_size_samples // 2 :]
                return

            classification = results.get("result", {}).get("classification", {})

            if not classification:
                self.audio_buffer = self.audio_buffer[-self.window_size_samples // 2 :]
                return

            # Find the best keyword (excluding noise)
            best_keyword = None
            best_score = 0.0

            for label, score in classification.items():
                if label.lower() == self.noise_label:
                    continue
                if score > best_score:
                    best_score = score
                    best_keyword = label

            # Publish if above threshold
            if best_keyword and best_score >= self.sensitivity_threshold:
                current_time = time.time()
                if current_time - self.last_detection_time >= self.detection_cooldown:
                    self.get_logger().info(
                        f"Keyword '{best_keyword}' detected with score {best_score:.2f}"
                    )
                    detection_info = {"keyword": best_keyword, "score": best_score}
                    self.publisher.publish(String(data=str(detection_info)))
                    self.last_detection_time = current_time

            # Keep half the window for overlap (sliding window)
            self.audio_buffer = self.audio_buffer[-self.window_size_samples // 2 :]

    def __del__(self):
        """Clean up the runner process."""
        if hasattr(self, "runner") and self.runner:
            try:
                self.runner.stop()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EdgeImpulseKWSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
