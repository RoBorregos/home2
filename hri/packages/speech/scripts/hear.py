#!/usr/bin/env python3

import os
import sys

import grpc
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils
from std_msgs.msg import String

from frida_interfaces.msg import AudioData
from frida_interfaces.srv import STT

# Add the directory containing the protos to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), "stt"))

import speech_pb2
import speech_pb2_grpc


class WhisperClient:
    def __init__(self, address):
        self.channel = grpc.insecure_channel(address)
        self.stub = speech_pb2_grpc.SpeechServiceStub(self.channel)

    def transcribe(self, audio_data):
        try:
            request = speech_pb2.AudioRequest(audio_data=audio_data)
            response = self.stub.Transcribe(request)
            return response.text
        except grpc.RpcError as e:
            raise RuntimeError(f"gRPC error: {e.code()}, {e.details()}")


class HearNode(Node):
    def __init__(self):
        super().__init__("hear_node")
        self.get_logger().info("*Starting Hear Node*")

        server_ip = (
            self.declare_parameter("STT_SERVER_IP", "127.0.0.1:50051")
            .get_parameter_value()
            .string_value
        )
        start_service = (
            self.declare_parameter("START_SERVICE", False)
            .get_parameter_value()
            .bool_value
        )
        service_name = (
            self.declare_parameter("STT_SERVICE_NAME", "stt_service")
            .get_parameter_value()
            .string_value
        )

        # Initialize the Whisper gRPC client
        self.client = WhisperClient(server_ip)

        # Create a publisher for the transcriptions
        self.transcription_publisher = self.create_publisher(
            String, "/speech/raw_command", 10
        )

        # Create groups for the subscription and service
        subscription_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()

        # Subscribe to audio data
        self.audio_subscription = self.create_subscription(
            AudioData,
            "UsefulAudio",
            self.callback_audio,
            10,
            callback_group=subscription_group,
        )

        # Create a service
        self.service_active = False
        if start_service:
            self.service_text = ""
            detection_publish_topic = (
                self.declare_parameter("detection_publish_topic", "/keyword_detected")
                .get_parameter_value()
                .string_value
            )
            self.KWS_publisher_mock = self.create_publisher(
                String, detection_publish_topic, 10
            )
            self.stt_service = self.create_service(
                STT,
                service_name,
                self.stt_service_callback,
                callback_group=service_group,
            )

        self.get_logger().info("*Hear Node is ready*")

    def callback_audio(self, data):
        self.get_logger().info("Received audio data, sending to Whisper gRPC server...")

        try:
            # Resample the audio to 16 kHz (if needed)
            resampled = SpeechApiUtils.resample_ratecv(data.data, 16000, 16000)
            all_samples = SpeechApiUtils.get_all_samples(resampled[0])

            # Convert samples to bytes
            audio_bytes = bytes(all_samples)

            # Send audio to Whisper gRPC server
            transcription = self.client.transcribe(audio_bytes)
            self.get_logger().info(f"Transcription received: {transcription}")

            # Publish the transcription
            msg = String()
            msg.data = transcription

            if self.service_active:
                # If the service is active, store the transcription
                self.service_text = transcription
                self.service_active = False
            else:
                # If the service is not active, publish the transcription
                self.transcription_publisher.publish(msg)
                self.get_logger().info("Transcription published to ROS topic.")
        except grpc.RpcError as e:
            self.get_logger().error(f"gRPC error: {e.code()}, {e.details()}")
        except Exception as ex:
            self.get_logger().error(f"Error during transcription: {str(ex)}")

    def stt_service_callback(self, request, response):
        self.get_logger().info("Keyword mock service activated, recording audio...")
        self.service_active = True
        self.KWS_publisher_mock.publish(String(data="frida"))
        while self.service_active:
            pass
        response.text_heard = self.service_text
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HearNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
