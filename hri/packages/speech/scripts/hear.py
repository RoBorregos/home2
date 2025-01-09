#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import grpc
from frida_interfaces.msg import AudioData
from std_msgs.msg import String
from speech.speech_api_utils import SpeechApiUtils

import sys
import os

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

        # Get the gRPC server address from parameters
        server_ip = (
            self.declare_parameter("STT_SERVER_IP", "localhost:50051")
            .get_parameter_value()
            .string_value
        )
        self.client = WhisperClient(server_ip)

        # Create a publisher for the transcriptions
        self.transcription_publisher = self.create_publisher(
            String, "/speech/raw_command", 10
        )

        # Subscribe to audio data
        self.audio_subscription = self.create_subscription(
            AudioData, "UsefulAudio", self.callback_audio, 10
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
            self.transcription_publisher.publish(msg)
            self.get_logger().info("Transcription published to ROS topic.")
        except grpc.RpcError as e:
            self.get_logger().error(f"gRPC error: {e.code()}, {e.details()}")
        except Exception as ex:
            self.get_logger().error(f"Error during transcription: {str(ex)}")


def main(args=None):
    rclpy.init(args=args)
    node = HearNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
