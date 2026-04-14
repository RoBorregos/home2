#!/usr/bin/env python3
"""
Keyword Spotting node using Edge Impulse HTTP inference container.
Sends audio features to an Edge Impulse inference server and publishes detections.
Classes: yes, no, stop, frida, noise
"""

import os
import time
import wave
from datetime import datetime

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
        self.declare_parameter("WAKEWORD_TOPIC", "/hri/speech/oww")
        self.declare_parameter("detection_cooldown", 1.0)
        self.declare_parameter("SENSITIVITY_THRESHOLD", 0.5)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("window_size_s", 1.0)
        self.declare_parameter("noise_label", "noise")
        self.declare_parameter("SAVE_DETECTION_AUDIO", False)
        self.declare_parameter(
            "AUDIO_SAVE_DIR",
            "/workspace/src/hri/packages/speech/assets/kws_detections",
        )

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
        self.save_detection_audio = (
            self.get_parameter("SAVE_DETECTION_AUDIO").get_parameter_value().bool_value
        )
        self.audio_save_dir = (
            self.get_parameter("AUDIO_SAVE_DIR").get_parameter_value().string_value
        )

        if self.save_detection_audio:
            os.makedirs(self.audio_save_dir, exist_ok=True)

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

        # Wait for the EI inference server to be ready before accepting audio
        self._wait_for_server()
        self.get_logger().info("Edge Impulse KWS node initialized and ready.")

    def _wait_for_server(self, poll_interval: float = 5.0, max_retries: int = 60):
        """Block until the EI inference server is reachable and responding."""
        self.get_logger().info(f"Waiting for EI server at {self.ei_url} to be ready...")
        for attempt in range(1, max_retries + 1):
            try:
                resp = requests.get(f"{self.ei_url}/api/info", timeout=3.0)
                if resp.ok:
                    self.get_logger().info(f"EI server is ready (attempt {attempt}).")
                    return
            except requests.RequestException:
                pass
            self.get_logger().info(
                f"EI server not ready yet (attempt {attempt}/{max_retries}), "
                f"retrying in {poll_interval}s..."
            )
            time.sleep(poll_interval)
        self.get_logger().warn(
            "EI server did not become ready in time. "
            "Proceeding anyway — inference requests may fail initially."
        )

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

        # Debug: log all classification results
        best_class = max(classification, key=classification.get)
        self.get_logger().debug(
            f"Classification: {best_class}={classification[best_class]:.2f} | {classification}"
        )

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
                if self.save_detection_audio:
                    self._save_audio_window(window, best_keyword, best_score)
                detection_info = {"keyword": best_keyword, "score": best_score}
                self.publisher.publish(String(data=str(detection_info)))
                self.last_detection_time = current_time

    def _save_audio_window(self, audio_window: np.ndarray, label: str, score: float):
        """Save the audio window that triggered a detection as a .wav file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        safe_label = "".join(
            ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in label
        )
        filename = f"{timestamp}_{safe_label}_{score:.2f}.wav"
        file_path = os.path.join(self.audio_save_dir, filename)

        try:
            with wave.open(file_path, "wb") as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(self.sample_rate)
                wav_file.writeframes(audio_window.astype(np.int16).tobytes())
            self.get_logger().info(f"Saved detection audio: {file_path}")
        except OSError as e:
            self.get_logger().error(
                f"Failed to save detection audio to '{file_path}': {e}"
            )


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
