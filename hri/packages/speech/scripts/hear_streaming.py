#!/usr/bin/env python3

import collections
import os
import sys
import time

import grpc
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils

from frida_interfaces.msg import AudioData

sys.path.append(os.path.join(os.path.dirname(__file__), "stt"))
import threading

import speech_pb2
import speech_pb2_grpc

SAVE_PATH = "/workspace/src/hri/packages/speech/debug/"
run_frames = []
SAVE_IT = 100


class HearStreaming(Node):
    def __init__(self):
        super().__init__("hear_streaming_node")
        self.get_logger().info("*Starting Hear Streaming Node*")
        self.declare_parameter("publish_topic", "/rawAudioChunk")
        self.declare_parameter("MIC_DEVICE_NAME", "default")
        self.declare_parameter("MIC_INPUT_CHANNELS", 32)
        self.declare_parameter("MIC_OUT_CHANNELS", 32)
        self.declare_parameter("USE_RESPEAKER", False)

        audio_topic = (
            self.declare_parameter("AUDIO_TOPIC", "/rawAudioChunk")
            .get_parameter_value()
            .string_value
        )
        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)

        publish_topic = (
            self.get_parameter("publish_topic").get_parameter_value().string_value
        )

        self.use_respeaker = (
            self.get_parameter("USE_RESPEAKER").get_parameter_value().bool_value
        )

        mic_device_name = (
            self.get_parameter("MIC_DEVICE_NAME").get_parameter_value().string_value
        )
        mic_input_channels = (
            self.get_parameter("MIC_INPUT_CHANNELS").get_parameter_value().integer_value
        )
        mic_out_channels = (
            self.get_parameter("MIC_OUT_CHANNELS").get_parameter_value().integer_value
        )

        self.publisher_ = self.create_publisher(AudioData, publish_topic, 20)
        self.input_device_index = SpeechApiUtils.getIndexByNameAndChannels(
            mic_device_name, mic_input_channels, mic_out_channels
        )

        self.input_device_index = None
        self.audio_buffer = collections.deque(maxlen=10000)
        self.get_logger().info("Input device index: " + str(self.input_device_index))

        if self.input_device_index is None:
            self.get_logger().warn(
                "Input device index not found, using system default."
            )

        self.get_logger().info("HearStreaming node initialized.")

    def audio_callback(self, msg):
        audio_data = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_buffer.append(audio_data)

    def record_subscribed(self):
        self.get_logger().info("HearStreaming node recording.")

        # GRPC client setup
        grpc_channel = grpc.insecure_channel("100.108.245.54:50051")
        stub = speech_pb2_grpc.SpeechStreamStub(grpc_channel)

        stop_flag = threading.Event()

        def request_generator():
            while not stop_flag.is_set() and rclpy.ok():
                try:
                    if not self.audio_buffer:
                        time.sleep(0.1)
                        continue
                    local_audio = self.audio_buffer.popleft()

                    # Validate audio length
                    if len(local_audio) < 10:
                        continue

                    grpc_audio = local_audio.tobytes()
                    yield speech_pb2.AudioRequest(
                        audio_data=grpc_audio, hotwords="Roborregos"
                    )

                except IOError as e:
                    self.get_logger().error(f"I/O error({e.errno}): {e.strerror}")
                    break

        def handle_transcripts(responses):
            self.get_logger().info("In handling transcripts.")
            try:
                for response in responses:
                    self.get_logger().info(f"Transcript: {response.text}")
            except grpc.RpcError as e:
                self.get_logger().error(f"gRPC stream error: {e}")

        responses = stub.Transcribe(request_generator())
        response_thread = threading.Thread(target=handle_transcripts, args=(responses,))
        response_thread.start()

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.get_logger().info("Stopping on user interrupt.")
        finally:
            stop_flag.set()
            response_thread.join()
            self.get_logger().info("Audio subscriber closed.")


def main(args=None):
    rclpy.init(args=args)
    try:
        n = HearStreaming()
        n.record_subscribed()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
