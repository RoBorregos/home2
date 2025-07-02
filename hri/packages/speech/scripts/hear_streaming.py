#!/usr/bin/env python3

import collections
import os
import sys
import time

import grpc
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from frida_interfaces.action import SpeechStream
from frida_interfaces.msg import AudioData

sys.path.append(os.path.join(os.path.dirname(__file__), "stt"))
import threading

import speech_pb2
import speech_pb2_grpc


class HearStreaming(Node):
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
        self.stop_flag.set()
        self.transcript_thread = None

        # gRPC Stub
        channel = grpc.insecure_channel(server_ip)
        self.stub = speech_pb2_grpc.SpeechStreamStub(channel)

        self.audio_buffer = collections.deque(maxlen=10000)

        subscription_group = MutuallyExclusiveCallbackGroup()
        action_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            AudioData,
            audio_topic,
            self.audio_callback,
            10,
            callback_group=subscription_group,
        )
        self._action_server = ActionServer(
            self,
            SpeechStream,
            self.action_server_name,
            self.execute_callback,
            callback_group=action_group,
        )

        self.transcription_publisher = self.create_publisher(
            String, "/speech/raw_command", 10
        )

        self.get_logger().info("*Hear Streaming Node is ready*")

    def audio_callback(self, msg):
        audio_data = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_buffer.append(audio_data)

    def record_subscribed(self, hotwords):
        self.get_logger().info("HearStreaming node recording.")
        # TODO: unsure if this is the best way to cancel the stream request
        call = None

        def request_generator():
            while not self.stop_flag.is_set() and rclpy.ok():
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
                        audio_data=grpc_audio, hotwords=hotwords
                    )

                except IOError as e:
                    self.get_logger().error(f"I/O error({e.errno}): {e.strerror}")
                    break
            # Cancel the grpc request if the stop flag is set
            call.cancel()

        def handle_transcripts(responses):
            try:
                for response in responses:
                    if self.stop_flag.is_set():
                        break
                    self.get_logger().info(f"Transcript: {response.text}")
                    self.current_transcription = response.text
            except grpc.RpcError as e:
                if "locally cancelled" not in e.details().lower():
                    self.get_logger().error(f"gRPC stream error: {e}")

        responses = self.stub.Transcribe(request_generator())
        call = responses
        self.transcript_thread = threading.Thread(
            target=handle_transcripts, args=(responses,)
        )
        self.transcript_thread.start()

    def execute_callback(self, goal_handle):
        self.get_logger().info("In execute callback")

        self.active_transcription = True
        self.stop_flag.clear()
        self.audio_buffer.clear()
        self.current_transcription = ""
        self.prev_transcription = ""

        start_time = time.time()
        last_word_time = start_time

        if goal_handle.request.hotwords:
            self.hotwords = goal_handle.request.hotwords
            self.get_logger().info(f"Updated hotwords: {self.hotwords}")

        self.record_subscribed(self.hotwords)

        try:
            while (
                not goal_handle.is_cancel_requested
                and time.time() - start_time < goal_handle.request.timeout
                and (
                    (
                        time.time() - start_time
                        < goal_handle.request.start_silence_time
                        and (
                            len(self.current_transcription) == 0
                            or self.current_transcription != self.hotwords
                        )
                    )
                    or time.time() - last_word_time < goal_handle.request.silence_time
                )
            ):
                if self.prev_transcription != self.current_transcription:
                    last_word_time = time.time()
                    self.prev_transcription = self.current_transcription
                    self.get_logger().info(
                        f"New transcription: {self.current_transcription}"
                    )
                    feedback_msg = SpeechStream.Feedback()
                    feedback_msg.current_transcription = self.prev_transcription
                    goal_handle.publish_feedback(feedback_msg)
                    self.transcription_publisher.publish(feedback_msg)

                # rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Execution interrupted: {e}")

        if time.time() - last_word_time >= goal_handle.request.silence_time:
            self.get_logger().info(
                f"Silence detected for more than {goal_handle.request.silence_time}, stopping transcription."
            )
        elif time.time() - start_time >= goal_handle.request.timeout:
            self.get_logger().info(
                f"Timeout of {goal_handle.request.timeout} seconds reached, stopping transcription."
            )
        elif goal_handle.is_cancel_requested:
            self.get_logger().info("Action goal cancelled, stopping transcription.")
        else:
            self.get_logger().info("Transcription completed successfully.")

        self.stop_flag.set()
        self.transcript_thread.join()
        goal_handle.succeed()
        result = SpeechStream.Result()
        result.transcription = self.current_transcription.strip()
        self.get_logger().info(f"Final transcription: {result.transcription}")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = HearStreaming()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.stop_flag.set()
        if node.transcript_thread and node.transcript_thread.is_alive():
            node.transcript_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
