#!/usr/bin/env python3

"""
Simple debug runner for AudioCapturer + HearStreaming in one process.
Run inside the hri-ros container:
  python3 /workspace/src/hri/packages/speech/debug/audio_system.py
"""

import os
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

import numpy as np

from frida_interfaces.msg import AudioData

from frida_interfaces.action import SpeechStream


def _add_scripts_to_path():
    scripts_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "scripts")
    )
    if scripts_dir not in sys.path:
        sys.path.append(scripts_dir)


_add_scripts_to_path()

# Import ROS nodes from scripts folder
from audio_capturer import AudioCapturer  # noqa: E402
from hear_streaming import HearStreaming  # noqa: E402


class TranscriptListener(Node):
    def __init__(self):
        super().__init__("transcript_listener")
        self.create_subscription(String, "/speech/raw_command", self._cb, 10)
        self.create_subscription(AudioData, "/processedAudioChunk", self._audio_cb, 10)
        self._action_client = ActionClient(self, SpeechStream, "stt_streaming")
        self._audio_samples = 0
        self._audio_rms = 0.0
        self._log_counter = 0

    def _cb(self, msg: String):
        self.get_logger().info(f"Heard: {msg.data}")

    def _audio_cb(self, msg: AudioData):
        data = np.frombuffer(msg.data, dtype=np.int16)
        if data.size == 0:
            return
        rms = float(np.sqrt(np.mean(np.square(data.astype(np.float32)))))
        self._audio_samples += data.size
        self._audio_rms = rms
        self._log_counter += 1
        if self._log_counter % 50 == 0:
            self.get_logger().info(
                f"Audio OK: samples={self._audio_samples}, rms={self._audio_rms:.2f}"
            )

    def start_stream(self, hotwords: str = "Frida"):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("STT action server not available")
            return

        goal = SpeechStream.Goal()
        goal.timeout = 20.0
        goal.hotwords = hotwords
        goal.silence_time = 3.0
        goal.start_silence_time = 3.0

        self.get_logger().info("Sending STT streaming goal...")
        self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)

    def _on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.current_transcription:
            self.get_logger().info(f"Partial: {feedback.current_transcription}")


def main():
    rclpy.init()

    audio_node = AudioCapturer()
    hear_node = HearStreaming()
    listener_node = TranscriptListener()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(audio_node)
    executor.add_node(hear_node)
    executor.add_node(listener_node)

    record_thread = threading.Thread(target=audio_node.record, daemon=True)
    record_thread.start()

    listener_node.start_stream()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        audio_node.destroy_node()
        hear_node.destroy_node()
        listener_node.destroy_node()
        rclpy.shutdown()
        record_thread.join(timeout=1)


if __name__ == "__main__":
    main()
