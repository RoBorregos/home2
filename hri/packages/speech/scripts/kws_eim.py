#!/usr/bin/env python3
"""
Keyword Spotting node using Edge Impulse HTTP inference container.
Sends audio features to an Edge Impulse inference server and publishes detections.
Classes: yes, no, stop, frida, noise
"""

import time

import numpy as np
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from frida_interfaces.msg import AudioData


class EdgeImpulseKWSNode(Node):
    def __init__(self):
        super().__init__("kws_eim")
        self.get_logger().info("Initializing Edge Impulse KWS node (HTTP mode).")

        self.declare_parameter("EI_SERVER_URL", "http://localhost:1338")
        self.declare_parameter("PROCESSED_AUDIO_TOPIC", "/hri/processedAudioChunk")
        self.declare_parameter("WAKEWORD_TOPIC", "/speech/oww")
        self.declare_parameter("detection_cooldown", 1.0)
        self.declare_parameter("SENSITIVITY_THRESHOLD", 0.7)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("window_size_s", 1.0)
        self.declare_parameter("noise_label", "noise")

        self.ei_url = (
            self.get_parameter("EI_SERVER_URL").get_parameter_value().string_value
        )
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
        window_size_s = (
            self.get_parameter("window_size_s").get_parameter_value().double_value
        )
        self.noise_label = (
            self.get_parameter("noise_label").get_parameter_value().string_value
        )

        self.window_size_samples = int(self.sample_rate * window_size_s)

        self.get_logger().info(
            f"EI server: {self.ei_url} | "
            f"Window: {self.window_size_samples} samples ({window_size_s}s) | "
            f"Threshold: {self.sensitivity_threshold}"
        )

        # Audio buffer to accumulate samples
        self.audio_buffer = np.array([], dtype=np.int16)

        # Publisher and subscriber
        self.publisher = self.create_publisher(String, wakeword_topic, 10)
        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)

        self.last_detection_time = time.time() - self.detection_cooldown
        self.get_logger().info("Edge Impulse KWS node initialized and ready.")

    def audio_callback(self, msg):
        """Process incoming audio data and detect keywords."""
        audio_data = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_buffer = np.concatenate([self.audio_buffer, audio_data])

        # Process when we have enough samples
        if len(self.audio_buffer) >= self.window_size_samples:
            window = self.audio_buffer[-self.window_size_samples :]
            self.audio_buffer = self.audio_buffer[-self.window_size_samples // 2 :]

            self._classify(window)

    def _classify(self, window: np.ndarray):
        """Send audio window to EI inference server and process results."""
        features = window.astype(float).tolist()

        try:
            resp = requests.post(
                f"{self.ei_url}/api/features",
                json={"features": features},
                timeout=5.0,
            )
            resp.raise_for_status()
        except requests.RequestException as e:
            self.get_logger().error(f"EI inference request failed: {e}")
            return

        result = resp.json()
        classification = result.get("result", {}).get("classification", {})

        if not classification:
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

        # Publish if above threshold and cooldown elapsed
        if best_keyword and best_score >= self.sensitivity_threshold:
            current_time = time.time()
            if current_time - self.last_detection_time >= self.detection_cooldown:
                self.get_logger().info(
                    f"Keyword '{best_keyword}' detected with score {best_score:.2f}"
                )
                detection_info = {"keyword": best_keyword, "score": best_score}
                self.publisher.publish(String(data=str(detection_info)))
                self.last_detection_time = current_time


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
