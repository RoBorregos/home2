#!/usr/bin/env python3
import os
import shutil
import time

import numpy as np
import openwakeword
import openwakeword.utils as utils
import rclpy
from openwakeword.model import Model
from rclpy.node import Node
from std_msgs.msg import String

from frida_interfaces.msg import AudioData

CURRENT_FILE_PATH = os.path.abspath(__file__)

FILE_DIR = CURRENT_FILE_PATH[: CURRENT_FILE_PATH.index("install")]
ASSETS_DIR = os.path.join(
    FILE_DIR, "src", "hri", "packages", "speech", "assets", "downloads"
)


class OpenWakeWordNode(Node):
    def __init__(self):
        super().__init__("kws_oww")
        self.get_logger().info("Initializing OpenWakeWord node.")

        self.declare_parameter(
            "model_path", "/workspace/src/hri/packages/speech/assets/oww"
        )
        self.declare_parameter("inference_framework", "onnx")
        # listen to processed audio by default
        self.declare_parameter("audio_topic", "/processedAudioChunk")
        self.declare_parameter("WAKEWORD_TOPIC", "/speech/oww")
        self.declare_parameter("chunk_size", 1280)
        self.declare_parameter("detection_cooldown", 1.0)
        self.declare_parameter("SENSITIVITY_THRESHOLD", 0.5)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        inference_framework = (
            self.get_parameter("inference_framework").get_parameter_value().string_value
        )
        audio_topic = (
            self.get_parameter("audio_topic").get_parameter_value().string_value
        )
        wakeword_topic = (
            self.get_parameter("WAKEWORD_TOPIC").get_parameter_value().string_value
        )
        self.chunk_size = (
            self.get_parameter("chunk_size").get_parameter_value().integer_value
        )
        self.detection_cooldown = (
            self.get_parameter("detection_cooldown").get_parameter_value().double_value
        )
        self.sensitivity_threshold = (
            self.get_parameter("SENSITIVITY_THRESHOLD")
            .get_parameter_value()
            .double_value
        )
        self.download_models()
        # Handle model loading dynamically from a directory
        if model_path:
            self.get_logger().info(f"Loading models from {model_path}")
            model_files = [f for f in os.listdir(model_path) if f.endswith(".onnx")]
            if model_files:
                self.oww_model = Model(
                    wakeword_models=[os.path.join(model_path, f) for f in model_files],
                    inference_framework=inference_framework,
                )
                self.get_logger().info(f"Loaded {len(model_files)} models.")
            else:
                self.get_logger().warn(f"No models found in directory: {model_path}")
        else:
            self.get_logger().info("Loading default model.")
            self.oww_model = Model(inference_framework=inference_framework)

        self.get_logger().info("OpenWakeWord model loaded successfully.")
        self.keywords = list(self.oww_model.models.keys())

        self.publisher = self.create_publisher(String, wakeword_topic, 10)
        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)

        # Flag to prevent rapid repeated publishing
        self.last_detection_time = time.time() - self.detection_cooldown
        self.get_logger().info("OpenWakeWord node initialized and ready.")

    def audio_callback(self, msg):
        """Process incoming audio data and detect wakewords."""
        # Convert audio data from ROS to NumPy array (to obtain directly from microphone)
        audio_data = np.frombuffer(msg.data, dtype=np.int16)
        # Perform prediction using OpenWakeWord (returns numerical value)
        self.oww_model.predict(audio_data)

        # Check for wakeword detection
        for keyword, buffer in self.oww_model.prediction_buffer.items():
            scores = list(buffer)
            if scores[-1] > self.sensitivity_threshold:
                # Only publish if the score is above a certain threshold
                current_time = time.time()
                if current_time - self.last_detection_time >= self.detection_cooldown:
                    self.get_logger().info(
                        f"Wakeword '{keyword}' detected with score {scores[-1]:.2f}"
                    )
                    detection_info = {"keyword": keyword, "score": scores[-1]}
                    self.publisher.publish(String(data=str(detection_info)))
                    self.last_detection_time = current_time
                break

    def download_models(self):
        # Download melospectogram

        melospectogram_download_path = os.path.join(ASSETS_DIR, "melspectrogram.onnx")
        melospectogram_execution_path = openwakeword.FEATURE_MODELS["melspectrogram"][
            "model_path"
        ].replace(".tflite", ".onnx")

        embedding_download_path = os.path.join(ASSETS_DIR, "embedding_model.onnx")
        embedding_execution_path = openwakeword.FEATURE_MODELS["embedding"][
            "model_path"
        ].replace(".tflite", ".onnx")

        # Download models if they do not exist

        if not os.path.exists(melospectogram_download_path):
            self.get_logger().info("Downloading melospectogram model.")

            utils.download_file(
                openwakeword.FEATURE_MODELS["melspectrogram"]["download_url"].replace(
                    ".tflite", ".onnx"
                ),
                ASSETS_DIR,
            )

        if not os.path.exists(embedding_download_path):
            self.get_logger().info("Downloading embedding model.")

            utils.download_file(
                openwakeword.FEATURE_MODELS["embedding"]["download_url"].replace(
                    ".tflite", ".onnx"
                ),
                ASSETS_DIR,
            )

        # Copy models to execution path if they do not exist

        if not os.path.exists(melospectogram_execution_path):
            self.get_logger().info("Copying model to execution path.")
            self.get_logger().info(
                f"model_download {melospectogram_download_path}. model_execution {melospectogram_execution_path}"
            )
            os.makedirs(os.path.dirname(melospectogram_execution_path), exist_ok=True)
            shutil.copyfile(melospectogram_download_path, melospectogram_execution_path)

        if not os.path.exists(embedding_execution_path):
            self.get_logger().info("Copying model to execution path.")
            self.get_logger().info(
                f"model_download {embedding_download_path}. model_execution {embedding_execution_path}"
            )
            os.makedirs(os.path.dirname(embedding_execution_path), exist_ok=True)
            shutil.copyfile(embedding_download_path, embedding_execution_path)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = OpenWakeWordNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
