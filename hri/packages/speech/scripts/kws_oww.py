#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from frida_interfaces.msg import AudioData
from openwakeword.model import Model


class OpenWakeWordNode(Node):
    def __init__(self):
        super().__init__("openwakeword_node")
        self.get_logger().info("Initializing OpenWakeWord node.")

        self.declare_parameter(
            "model_path", "/workspace/src/hri/packages/speech/assets"
        )
        self.declare_parameter("inference_framework", "onnx")
        self.declare_parameter("audio_topic", "/rawAudioChunk")
        self.declare_parameter("detection_topic", "/wakeword_detected")
        self.declare_parameter("chunk_size", 1280)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        inference_framework = (
            self.get_parameter("inference_framework").get_parameter_value().string_value
        )
        audio_topic = (
            self.get_parameter("audio_topic").get_parameter_value().string_value
        )
        detection_topic = (
            self.get_parameter("detection_topic").get_parameter_value().string_value
        )
        self.chunk_size = (
            self.get_parameter("chunk_size").get_parameter_value().integer_value
        )

        # Initialize OpenWakeWord
        if model_path:
            self.get_logger().info(f"Loading model from {model_path}")
            self.oww_model = Model(
                wakeword_models=[model_path],
                inference_framework=inference_framework,
            )
        else:
            self.get_logger().info("Loading default model.")
            self.oww_model = Model(inference_framework=inference_framework)

        self.get_logger().info("OpenWakeWord model loaded successfully.")
        self.keywords = list(self.oww_model.models.keys())

        self.publisher = self.create_publisher(Bool, detection_topic, 10)
        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)

        self.get_logger().info("OpenWakeWord node initialized and ready.")

    def audio_callback(self, msg):
        """Process incoming audio data and detect wakewords."""
        # Convert audio data from ROS to NumPy array (to obtain directly from microphone)
        # audio_data = np.frombuffer(msg.data, dtype=np.int16)
        # Perform prediction using OpenWakeWord (returns numerical value)
        # prediction = self.oww_model.predict(audio_data)

        # Check for wakeword detection ("YES", "NO", "Frida")
        for keyword, buffer in self.oww_model.prediction_buffer.items():
            scores = list(buffer)
            if (
                scores[-1] > 0.5
            ):  # NOTE: This value can be decreased for more sensitive yet less effective detection
                self.get_logger().info(
                    f"Wakeword '{keyword}' detected with score {scores[-1]:.2f}"
                )
                self.publisher.publish(Bool(data=True))
                break


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
