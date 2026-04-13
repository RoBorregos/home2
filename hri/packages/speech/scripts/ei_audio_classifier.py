#!/usr/bin/env python3

import os
from datetime import datetime
import wave

import numpy as np
import requests
import rclpy
from rclpy.node import Node
from frida_interfaces.msg import AudioData
from std_msgs.msg import String


class EIAudioClassifier(Node):
    def __init__(self):
        super().__init__("ei_audio_classifier")

        self.declare_parameter("RAW_AUDIO_TOPIC", "/hri/rawAudioChunk")
        self.declare_parameter("DETECTION_TOPIC", "/hri/speech/ei_detection")
        self.declare_parameter("EI_SERVER_URL", "http://localhost:1337")
        self.declare_parameter("SAMPLE_RATE", 16000)
        self.declare_parameter("WINDOW_SIZE_S", 1.0)
        self.declare_parameter("NOISE_THRESHOLD", 0.8)
        self.declare_parameter("SAVE_DETECTION_AUDIO", False)
        self.declare_parameter("AUDIO_SAVE_DIR", "/workspace/src/hri/packages/speech/assets/event_detections")

        audio_topic = self.get_parameter("RAW_AUDIO_TOPIC").get_parameter_value().string_value
        detection_topic = self.get_parameter("DETECTION_TOPIC").get_parameter_value().string_value
        self.ei_url = self.get_parameter("EI_SERVER_URL").get_parameter_value().string_value
        self.sample_rate = self.get_parameter("SAMPLE_RATE").get_parameter_value().integer_value
        window_size_s = self.get_parameter("WINDOW_SIZE_S").get_parameter_value().double_value
        self.noise_threshold = self.get_parameter("NOISE_THRESHOLD").get_parameter_value().double_value
        self.save_detection_audio = (
            self.get_parameter("SAVE_DETECTION_AUDIO").get_parameter_value().bool_value
        )
        self.audio_save_dir = self.get_parameter("AUDIO_SAVE_DIR").get_parameter_value().string_value

        if self.save_detection_audio:
            os.makedirs(self.audio_save_dir, exist_ok=True)

        self.window_samples = int(self.sample_rate * window_size_s)
        self.audio_buffer = np.array([], dtype=np.int16)

        self.publisher = self.create_publisher(String, detection_topic, 10)
        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)

        self.get_logger().info(
            f"EIAudioClassifier ready | in: {audio_topic} | out: {detection_topic} | "
            f"window: {self.window_samples} samples ({window_size_s}s)"
        )

    def audio_callback(self, msg):
        chunk = np.frombuffer(bytes(msg.data), dtype=np.int16)
        self.audio_buffer = np.concatenate([self.audio_buffer, chunk])

        while len(self.audio_buffer) >= self.window_samples:
            window = self.audio_buffer[: self.window_samples]
            self.audio_buffer = self.audio_buffer[self.window_samples :]
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
        except requests.RequestException as e:
            self.get_logger().error(f"EI inference request failed: {e}")
            return

        result = resp.json()
        classification = result.get("result", {}).get("classification", {})

        if not classification:
            return

        best_label = max(classification, key=classification.get)
        best_score = classification[best_label]

        # Skip if the best label is noise above the threshold
        if best_label == "noise" and best_score >= self.noise_threshold:
            return

        # Skip if noise is the only non-zero class
        if best_label == "noise":
            return

        if self.save_detection_audio:
            self.get_logger().info(f"Saving detected audio: {best_label} ({best_score:.2f})")
            self._save_audio_window(audio_window, best_label, best_score)

        self.get_logger().info(f"Detection: {best_label} ({best_score:.2f}) | {classification}")
        self.publisher.publish(String(data=str(classification)))

    def _save_audio_window(self, audio_window: np.ndarray, label: str, score: float):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        safe_label = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in label)
        filename = f"{timestamp}_{safe_label}_{score:.2f}.wav"
        file_path = os.path.join(self.audio_save_dir, filename)

        try:
            with wave.open(file_path, "wb") as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(self.sample_rate)
                wav_file.writeframes(audio_window.astype(np.int16).tobytes())
        except OSError as e:
            self.get_logger().error(f"Failed to save detected audio to '{file_path}': {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(EIAudioClassifier())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
