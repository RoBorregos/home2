#!/usr/bin/env python3

import collections
import os
import sys
import threading
import time

import grpc
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from frida_interfaces.action import SpeechStream
from frida_interfaces.msg import AudioData

# Add the directory containing the protos to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), "stt"))

import speech_pb2
import speech_pb2_grpc


class HearNode(Node):
    def __init__(self):
        super().__init__("hear_streaming_node")
        self.get_logger().info("*Starting Hear Streaming Node*")

        server_ip = (
            self.declare_parameter("STT_SERVER_IP", "127.0.0.1:50051")
            .get_parameter_value()
            .string_value
        )

        audio_topic = (
            self.declare_parameter("AUDIO_TOPIC", "/rawAudioChunk")
            .get_parameter_value()
            .string_value
        )

        self.action_server_name = (
            self.declare_parameter("STT_ACTION_SERVER_NAME", "stt_streaming")
            .get_parameter_value()
            .string_value
        )

        default_hotwords = (
            self.declare_parameter("DEFAULT_HOTWORDS", "Frida RoBorregos")
            .get_parameter_value()
            .string_value
        )

        self.hotwords = default_hotwords
        self.active_transcription = False
        self.current_transcription = ""
        self.stop_flag = threading.Event()

        # gRPC Stub
        channel = grpc.insecure_channel(server_ip)
        self.stub = speech_pb2_grpc.SpeechServiceStub(channel)

        self.audio_buffer = collections.deque(maxlen=10000)

        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)
        self._action_server = ActionServer(
            self, SpeechStream, self.action_server_name, self.execute_callback
        )

        self.get_logger().info("*Hear Streaming Node is ready*")

    def audio_callback(self, msg):
        if not self.active_transcription:
            return
        audio_data = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_buffer.append(audio_data)

    def request_generator(self):
        while not self.stop_flag.is_set() and rclpy.ok() and self.active_transcription:
            if self.audio_buffer:
                chunk = self.audio_buffer.popleft()
                audio_bytes = bytes(chunk)
                print("audio_bytes size:", len(audio_bytes))
                self.get_logger().debug(
                    f"Sending chunk of type {type(audio_bytes)}, length: {len(audio_bytes)}"
                )

                yield speech_pb2.AudioRequest(audio_data=audio_bytes)
            else:
                time.sleep(0.01)  # Avoid busy waiting

    def handle_transcripts(self, responses, goal_handle):
        try:
            for response in responses:
                self.get_logger().info(f"Transcript: {response.text}")
                self.current_transcription = response.text
                feedback_msg = SpeechStream.Feedback()
                feedback_msg.partial_text = response.text
                goal_handle.publish_feedback(feedback_msg)
        except grpc.RpcError as e:
            self.get_logger().error(f"gRPC error: {e}")

    def execute_callback(self, goal_handle):
        self.active_transcription = True
        self.stop_flag.clear()
        self.audio_buffer.clear()
        self.current_transcription = ""

        start_time = time.time()

        if goal_handle.request.hotwords:
            self.hotwords = goal_handle.request.hotwords
            self.get_logger().info(f"Updated hotwords: {self.hotwords}")

        self.get_logger().info("Calling stub.Transcribe()")
        responses = self.stub.Transcribe(self.request_generator())
        self.get_logger().info("After stub.Transcribe()")
        response_thread = threading.Thread(
            target=self.handle_transcripts, args=(responses, goal_handle)
        )
        response_thread.start()

        try:
            while (
                not goal_handle.is_cancel_requested
                and time.time() - start_time < goal_handle.request.timeout
            ):
                rclpy.spin_once(self, timeout_sec=0.1)
        except Exception as e:
            self.get_logger().error(f"Execution interrupted: {e}")

        self.stop_flag.set()
        self.active_transcription = False
        response_thread.join()

        goal_handle.succeed()
        result = SpeechStream.Result()
        result.transcription = self.current_transcription.strip()
        return result


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
