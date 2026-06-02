#!/usr/bin/env python3
"""
Unified Edge Impulse Audio Node.
Sends audio features to an Edge Impulse inference server and publishes detections.
Supports overlapping windows, sensitivity thresholds, and detection cooldowns.
"""

import json
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


class EIAudioNode(Node):
    def __init__(self):
        super().__init__("ei_audio_node")

        self.declare_parameter("audio_topic", "/hri/processedAudioChunk")
        self.declare_parameter("KEYWORD_TOPIC", "/hri/speech/kws")
        self.declare_parameter("ei_server_url", "http://localhost:1337")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("window_size_s", 1.0)
        self.declare_parameter("hop_ratio", 1.0)
        self.declare_parameter("sensitivity_threshold", 0.7)
        self.declare_parameter("detection_cooldown", 0.0)
        self.declare_parameter("audio_gain", 1.0)
        self.declare_parameter("noise_label", "noise")
        self.declare_parameter("save_detection_audio", False)
        self.declare_parameter(
            "audio_save_dir",
            "/workspace/src/hri/packages/speech/assets/detections",
        )

        audio_topic = (
            self.get_parameter("audio_topic").get_parameter_value().string_value
        )
        result_topic = (
            self.get_parameter("KEYWORD_TOPIC").get_parameter_value().string_value
        )
        self.ei_url = (
            self.get_parameter("ei_server_url").get_parameter_value().string_value
        )
        self.sample_rate = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )
        window_size_s = (
            self.get_parameter("window_size_s").get_parameter_value().double_value
        )
        hop_ratio = self.get_parameter("hop_ratio").get_parameter_value().double_value
        self.sensitivity_threshold = (
            self.get_parameter("sensitivity_threshold")
            .get_parameter_value()
            .double_value
        )
        self.detection_cooldown = (
            self.get_parameter("detection_cooldown").get_parameter_value().double_value
        )
        self.audio_gain = (
            self.get_parameter("audio_gain").get_parameter_value().double_value
        )
        self.noise_label = (
            self.get_parameter("noise_label").get_parameter_value().string_value
        )
        self.save_detection_audio = (
            self.get_parameter("save_detection_audio").get_parameter_value().bool_value
        )
        self.audio_save_dir = (
            self.get_parameter("audio_save_dir").get_parameter_value().string_value
        )

        if self.save_detection_audio:
            os.makedirs(self.audio_save_dir, exist_ok=True)

        self.window_samples = int(self.sample_rate * window_size_s)
        self.hop_samples = max(1, int(self.window_samples * hop_ratio))
        self.audio_buffer = np.array([], dtype=np.int16)

        self.publisher = self.create_publisher(String, result_topic, 10)
        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)

        self.last_detection_time = time.time() - self.detection_cooldown
        self._server_error_logged = False

        # Wait for the EI inference server to be ready before accepting audio
        self._wait_for_server()

        self.get_logger().info(
            f"EIAudioNode '{self.get_name()}' ready | in: {audio_topic} | out: {result_topic} | "
            f"window: {self.window_samples} samples ({window_size_s}s) | hop: {self.hop_samples} samples"
        )

    def _wait_for_server(self, poll_interval: float = 5.0, max_retries: int = 60):
        """Block until the EI inference server is reachable and responding."""
        self.get_logger().info(f"Waiting for EI server at {self.ei_url} to be ready...")
        for attempt in range(1, max_retries + 1):
            try:
                resp = requests.get(f"{self.ei_url}/api/info", timeout=3.0)
                if resp.ok:
                    self.get_logger().info(f"EI server is ready (attempt {attempt}).")
                    self._server_error_logged = False
                    return
            except requests.RequestException:
                pass
            time.sleep(poll_interval)
        self.get_logger().warn(
            "EI server did not become ready in time. "
            "Proceeding anyway — inference requests may fail initially."
        )
        self._server_error_logged = True

    def audio_callback(self, msg):
        chunk = np.frombuffer(bytes(msg.data), dtype=np.int16)
        self.audio_buffer = np.concatenate([self.audio_buffer, chunk])

        while len(self.audio_buffer) >= self.window_samples:
            window = self.audio_buffer[: self.window_samples]
            self.audio_buffer = self.audio_buffer[self.hop_samples :]

            if self.audio_gain != 1.0:
                window = np.clip(
                    window.astype(np.float32) * self.audio_gain,
                    np.iinfo(np.int16).min,
                    np.iinfo(np.int16).max,
                ).astype(np.int16)

            self.classify(window)

    def classify(self, audio_window: np.ndarray):
        features = audio_window.astype(float).tolist()

        try:
            resp = requests.post(
                f"{self.ei_url}/api/features",
                json={"features": features},
                timeout=5.0,
            )
            resp.raise_for_status()
            if self._server_error_logged:
                self.get_logger().info("EI inference server is reachable again.")
                self._server_error_logged = False
        except requests.RequestException as e:
            if not self._server_error_logged:
                self.get_logger().error(
                    f"EI inference request failed (suppressing further errors): {e}"
                )
                self._server_error_logged = True
            return

        result = resp.json()
        classification = result.get("result", {}).get("classification", {})

        if not classification:
            return

        # Find the best non-noise label
        best_label = None
        best_score = 0.0
        for label, score in classification.items():
            if label.lower() == self.noise_label.lower():
                continue
            if score > best_score:
                best_score = score
                best_label = label

        if best_label and best_score >= self.sensitivity_threshold:
            current_time = time.time()
            if current_time - self.last_detection_time >= self.detection_cooldown:
                self.get_logger().info(
                    f"Detection: {best_label} ({best_score:.2f}) | {classification}"
                )
                if self.save_detection_audio:
                    self._save_audio_window(audio_window, best_label, best_score)

                detection_info = {"label": best_label, "score": float(best_score)}
                self.publisher.publish(String(data=json.dumps(detection_info)))
                self.last_detection_time = current_time

    def _save_audio_window(self, audio_window: np.ndarray, label: str, score: float):
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
        except OSError as e:
            self.get_logger().error(
                f"Failed to save detected audio to '{file_path}': {e}"
            )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EIAudioNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
