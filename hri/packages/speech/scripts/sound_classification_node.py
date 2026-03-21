#!/usr/bin/env python3
import os
import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from frida_interfaces.msg import AudioData, SoundEvent

from edge_impulse_linux.runner import ImpulseRunner

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
MODEL_PATH = os.path.abspath(
    os.path.join(SCRIPT_DIR, "..", "assets", "edge_impulse", "hric_inits_model.eim")
)

SAMPLE_RATE = 16000
WINDOW_SIZE_MS = 1000
WINDOW_STRIDE_MS = 500
WINDOW_SIZE_SAMPLES = (SAMPLE_RATE * WINDOW_SIZE_MS) // 1000
WINDOW_STRIDE_SAMPLES = (SAMPLE_RATE * WINDOW_STRIDE_MS) // 1000
CONFIDENCE_THRESHOLD = 0.8
TARGET_LABELS = ["doorBell", "knock"]


class SoundClassificationNode(Node):
    def __init__(self):
        super().__init__("sound_classification_node")
        self.get_logger().info("Initializing Sound Classification node.")

        self.declare_parameter("PROCESSED_AUDIO_TOPIC", "/hri/processedAudioChunk")
        self.declare_parameter("SOUND_EVENTS_TOPIC", "/audio/sound_events")
        self.declare_parameter("model_path", MODEL_PATH)

        audio_topic = (
            self.get_parameter("PROCESSED_AUDIO_TOPIC")
            .get_parameter_value()
            .string_value
        )
        events_topic = (
            self.get_parameter("SOUND_EVENTS_TOPIC").get_parameter_value().string_value
        )
        model_path = self.get_parameter("model_path").get_parameter_value().string_value

        self.model_path = model_path
        self.load_model()

        self.audio_buffer = np.array([], dtype=np.float32)
        self._audio_queue = queue.Queue(maxsize=10)
        self.inference_executor = ThreadPoolExecutor(max_workers=1)

        self.audio_publisher = self.create_publisher(SoundEvent, events_topic, 10)
        self.create_subscription(AudioData, audio_topic, self._audio_callback, 10)

        threading.Thread(target=self._process_loop, daemon=True).start()

        self.get_logger().info(
            f"Sound Classification node ready. Model: {os.path.basename(model_path)}"
        )

    def load_model(self):
        try:
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"Model not found at {self.model_path}")
                raise FileNotFoundError(f"Model file not found: {self.model_path}")

            self.runner = ImpulseRunner(self.model_path)
            self.model_info = self.runner.init()
            labels = self.model_info["model_parameters"]["labels"]
            self.get_logger().info(f"Edge Impulse model loaded. Labels: {labels}")
        except Exception as e:
            self.get_logger().error(f"Failed to load Edge Impulse model: {e}")
            raise

    def _audio_callback(self, msg):
        try:
            self._audio_queue.put_nowait(msg)
        except queue.Full:
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                pass
            self._audio_queue.put_nowait(msg)

    def _process_loop(self):
        while True:
            msg = self._audio_queue.get()
            audio_int16 = np.frombuffer(msg.data, dtype=np.int16)
            audio_float32 = audio_int16.astype(np.float32) / 32768.0

            self.audio_buffer = np.concatenate([self.audio_buffer, audio_float32])

            while len(self.audio_buffer) >= WINDOW_SIZE_SAMPLES:
                window = self.audio_buffer[:WINDOW_SIZE_SAMPLES].copy()
                self.inference_executor.submit(self._classify_window, window)
                self.audio_buffer = self.audio_buffer[WINDOW_STRIDE_SAMPLES:]

    def _classify_window(self, audio_window):
        try:
            # The model requires float casting
            audio_list = audio_window.astype(np.float32).tolist()
            result = self.runner.classify(audio_list)

            if result and "result" in result and "classification" in result["result"]:
                classifications = result["result"]["classification"]

                timestamp_ms = int(time.time() * 1000)

                for label, confidence in classifications.items():
                    if label in TARGET_LABELS and confidence > CONFIDENCE_THRESHOLD:
                        self.get_logger().info(
                            f"SOUND DETECTED: {label} (confidence: {confidence:.3f})"
                        )

                        event = SoundEvent(
                            label=label,
                            confidence=float(confidence),
                            timestamp_ms=timestamp_ms,
                        )
                        self.audio_publisher.publish(event)

        except Exception as e:
            self.get_logger().error(f"Classification error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SoundClassificationNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
