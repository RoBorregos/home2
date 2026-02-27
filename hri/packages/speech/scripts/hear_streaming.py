#!/usr/bin/env python3

import collections
import os
import sys
import time

import grpc
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
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
            self.declare_parameter("AUDIO_TOPIC", "/processedAudioChunk")
            .get_parameter_value()
            .string_value
        )

        self.action_server_name = (
            self.declare_parameter("STT_ACTION_SERVER_NAME", "stt_streaming")
            .get_parameter_value()
            .string_value
        )

        self.default_hotwords = (
            self.declare_parameter("DEFAULT_HOTWORDS", "Frida RoBorregos")
            .get_parameter_value()
            .string_value
        )

        self.hotwords = self.default_hotwords
        self.current_transcription = ""
        self.current_words = []
        self.stop_flag = threading.Event()
        self.stop_flag.set()
        self.transcript_thread = None
        self.cancel_requested = False

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
            cancel_callback=self.cancel_callback,
        )

        self.transcription_publisher = self.create_publisher(
            String, "/speech/raw_command", 10
        )

        self.get_logger().info("*Hear Streaming Node is ready*")

    def cancel_callback(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info("Received cancel request")
        self.cancel_requested = True
        return CancelResponse.ACCEPT

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
                    self.current_words = [
                        {"word": w.word, "confidence": w.confidence}
                        for w in response.words
                    ]
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

        self.stop_flag.clear()
        self.audio_buffer.clear()
        self.current_transcription = ""
        self.current_words = []
        self.prev_transcription = ""
        self.cancel_requested = False

        start_time = time.time()
        last_word_time = start_time

        if goal_handle.request.hotwords:
            self.hotwords = goal_handle.request.hotwords
            self.get_logger().info(f"Updated hotwords: {self.hotwords}")
        else:
            self.hotwords = self.default_hotwords

        self.record_subscribed(self.hotwords)

        try:
            while (
                not goal_handle.is_cancel_requested
                and not self.cancel_requested
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
                    self.transcription_publisher.publish(
                        String(data=self.prev_transcription)
                    )

                # Use a short sleep to avoid busy waiting while still being responsive to cancellation
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Execution interrupted: {e}")

        if time.time() - last_word_time >= goal_handle.request.silence_time:
            self.get_logger().info(
                f"Silence detected for more than {goal_handle.request.silence_time}, stopping transcription."
            )
            reason = "silence"
        elif time.time() - start_time >= goal_handle.request.timeout:
            self.get_logger().info(
                f"Timeout of {goal_handle.request.timeout} seconds reached, stopping transcription."
            )
            reason = "timeout"
        elif goal_handle.is_cancel_requested or self.cancel_requested:
            self.get_logger().info("Action goal cancelled, stopping transcription.")
            reason = "cancelled"
        else:
            self.get_logger().info("Transcription completed successfully.")
            reason = "completed"

        self.stop_flag.set()
        self.transcript_thread.join()

        # Handle the goal result based on the reason for stopping
        if reason == "cancelled":
            goal_handle.canceled()
        else:
            goal_handle.succeed()

        result = SpeechStream.Result()
        result.transcription = self.current_transcription.strip()
        if hasattr(result, "words"):
            result.words = [w["word"] for w in self.current_words]
            result.confidences = [w["confidence"] for w in self.current_words]
        self.get_logger().info(f"Final transcription: {result.transcription}")
        if self.current_words:
            self.get_logger().info(
                "Word confidences: "
                + ", ".join(
                    f"{w['word']}({w['confidence']:.2f})" for w in self.current_words
                )
            )

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
