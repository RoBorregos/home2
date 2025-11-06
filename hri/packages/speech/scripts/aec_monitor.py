#!/usr/bin/env python3
"""
AEC Monitor - Real-time monitoring tool for the AEC node.

This script subscribes to AEC topics and displays real-time status.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from frida_interfaces.msg import AudioData
import time
import numpy as np


class AECMonitor(Node):
    def __init__(self):
        super().__init__("aec_monitor")

        self.get_logger().info("AEC Monitor started")

        # Counters
        self.mic_count = 0
        self.robot_count = 0
        self.output_count = 0
        self.robot_speaking = False

        # Energy tracking
        self.mic_energy = 0.0
        self.output_energy = 0.0

        # Timestamps
        self.last_update = time.time()

        # Subscribers
        self.create_subscription(AudioData, "/rawAudioChunk", self.mic_callback, 10)
        self.create_subscription(
            AudioData, "/robot_audio_output", self.robot_callback, 10
        )
        self.create_subscription(
            AudioData, "/aec_audio_output", self.output_callback, 10
        )
        self.create_subscription(String, "/aec_status", self.status_callback, 10)
        self.create_subscription(Bool, "/saying", self.saying_callback, 10)

        # Timer for periodic updates
        self.create_timer(1.0, self.print_stats)

    def mic_callback(self, msg):
        self.mic_count += 1
        audio = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        self.mic_energy = np.sqrt(np.mean(audio**2))

    def robot_callback(self, msg):
        self.robot_count += 1

    def output_callback(self, msg):
        self.output_count += 1
        audio = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        self.output_energy = np.sqrt(np.mean(audio**2))

    def status_callback(self, msg):
        self.get_logger().info(f"AEC Status: {msg.data}")

    def saying_callback(self, msg):
        self.robot_speaking = msg.data

    def print_stats(self):
        now = time.time()
        dt = now - self.last_update

        mic_rate = self.mic_count / dt if dt > 0 else 0
        robot_rate = self.robot_count / dt if dt > 0 else 0
        output_rate = self.output_count / dt if dt > 0 else 0

        status_emoji = "🔊" if self.robot_speaking else "🤫"

        print("\n" + "=" * 60)
        print(
            f"AEC Monitor - {status_emoji} Robot {'SPEAKING' if self.robot_speaking else 'SILENT'}"
        )
        print("=" * 60)
        print(
            f"Mic input:      {mic_rate:6.1f} msgs/s  | Energy: {self.mic_energy:.4f}"
        )
        print(f"Robot ref:      {robot_rate:6.1f} msgs/s")
        print(
            f"AEC output:     {output_rate:6.1f} msgs/s  | Energy: {self.output_energy:.4f}"
        )

        if self.mic_energy > 1e-6 and self.output_energy > 1e-6:
            reduction = 20 * np.log10(self.mic_energy / self.output_energy)
            print(f"Echo reduction: {reduction:6.1f} dB")

        # Reset counters
        self.mic_count = 0
        self.robot_count = 0
        self.output_count = 0
        self.last_update = now


def main(args=None):
    rclpy.init(args=args)
    node = AECMonitor()

    try:
        print("\nAEC Monitor - Press Ctrl+C to exit\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
