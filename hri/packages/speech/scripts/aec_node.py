#!/usr/bin/env python3
"""
Acoustic Echo Cancellation (AEC) Node using STFT-based processing.

This node subscribes to both the robot's audio output (reference/far-end)
and the microphone input (near-end + echo), then publishes echo-cancelled audio.
"""

import collections
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Bool, String

from frida_interfaces.msg import AudioData
from speech.stft_aec import STFTEchoCanceller, normalize_audio


class AECNode(Node):
    def __init__(self):
        super().__init__("aec_node")
        self.get_logger().info("*Starting AEC Node*")

        # Declare parameters
        self.declare_parameter("enabled", True)
        self.declare_parameter("mic_topic", "/rawAudioChunk")
        self.declare_parameter("robot_audio_topic", "/robot_audio_output")
        self.declare_parameter("output_topic", "/aec_audio_output")
        self.declare_parameter("status_topic", "/aec_status")

        self.declare_parameter("frame_size", 1024)
        self.declare_parameter("hop_size", 256)
        self.declare_parameter("beta", 0.9)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("max_delay_ms", 200.0)

        self.declare_parameter("buffer_size", 5000)
        self.declare_parameter("enable_normalization", True)
        self.declare_parameter("sync_tolerance_ms", 50.0)

        # Get parameters
        self.enabled = self.get_parameter("enabled").get_parameter_value().bool_value
        mic_topic = self.get_parameter("mic_topic").get_parameter_value().string_value
        robot_audio_topic = (
            self.get_parameter("robot_audio_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        status_topic = (
            self.get_parameter("status_topic").get_parameter_value().string_value
        )

        frame_size = (
            self.get_parameter("frame_size").get_parameter_value().integer_value
        )
        hop_size = self.get_parameter("hop_size").get_parameter_value().integer_value
        beta = self.get_parameter("beta").get_parameter_value().double_value
        sample_rate = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )
        max_delay_ms = (
            self.get_parameter("max_delay_ms").get_parameter_value().double_value
        )

        buffer_size = (
            self.get_parameter("buffer_size").get_parameter_value().integer_value
        )
        self.enable_normalization = (
            self.get_parameter("enable_normalization").get_parameter_value().bool_value
        )
        self.sync_tolerance_ms = (
            self.get_parameter("sync_tolerance_ms").get_parameter_value().double_value
        )

        # Initialize AEC
        self.aec = STFTEchoCanceller(
            frame_size=frame_size,
            hop_size=hop_size,
            beta=beta,
            sample_rate=sample_rate,
        )
        self.max_delay_samples = int(max_delay_ms * 1e-3 * sample_rate)

        # Audio buffers with timestamps: (timestamp_ns, audio_data)
        self.mic_buffer = collections.deque(maxlen=buffer_size)
        self.robot_buffer = collections.deque(maxlen=buffer_size)
        self.buffer_lock = threading.Lock()

        # State tracking
        self.robot_speaking = False
        self.last_robot_audio_time = 0.0
        self.processing_enabled = self.enabled

        # Callback groups for parallel execution
        subscription_group = MutuallyExclusiveCallbackGroup()
        publisher_group = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.create_subscription(
            AudioData,
            mic_topic,
            self.mic_callback,
            10,
            callback_group=subscription_group,
        )

        self.create_subscription(
            AudioData,
            robot_audio_topic,
            self.robot_audio_callback,
            10,
            callback_group=subscription_group,
        )

        # Subscribe to robot speaking status
        self.create_subscription(
            Bool,
            "/saying",
            self.robot_speaking_callback,
            10,
            callback_group=subscription_group,
        )

        # Publishers
        self.output_publisher = self.create_publisher(
            AudioData, output_topic, 10, callback_group=publisher_group
        )

        self.status_publisher = self.create_publisher(
            String, status_topic, 10, callback_group=publisher_group
        )

        # Processing thread
        self.processing_thread = None
        self.should_process = True
        self.start_processing_thread()

        self.get_logger().info(
            f"*AEC Node initialized* (enabled={self.enabled}, "
            f"frame_size={frame_size}, hop_size={hop_size}, beta={beta})"
        )

    def robot_speaking_callback(self, msg):
        """Callback for robot speaking status."""
        self.robot_speaking = msg.data
        if self.robot_speaking:
            self.last_robot_audio_time = time.time()
            self.get_logger().debug("Robot started speaking")
        else:
            self.get_logger().debug("Robot stopped speaking")

    def mic_callback(self, msg):
        """Callback for microphone audio data."""
        audio_data = (
            np.frombuffer(msg.data, dtype=np.int16).astype(np.float64) / 32768.0
        )
        timestamp = self.get_clock().now().nanoseconds

        with self.buffer_lock:
            self.mic_buffer.append((timestamp, audio_data))

    def robot_audio_callback(self, msg):
        """Callback for robot audio output (reference signal)."""
        audio_data = (
            np.frombuffer(msg.data, dtype=np.int16).astype(np.float64) / 32768.0
        )
        timestamp = self.get_clock().now().nanoseconds

        with self.buffer_lock:
            self.robot_buffer.append((timestamp, audio_data))
            self.last_robot_audio_time = time.time()

    def start_processing_thread(self):
        """Start the audio processing thread."""
        self.should_process = True
        self.processing_thread = threading.Thread(
            target=self.process_audio_loop, daemon=True
        )
        self.processing_thread.start()
        self.get_logger().info("AEC processing thread started")

    def process_audio_loop(self):
        """Main processing loop that runs in a separate thread."""
        while self.should_process and rclpy.ok():
            try:
                if not self.processing_enabled:
                    time.sleep(0.1)
                    continue

                # Check if we have enough data to process
                with self.buffer_lock:
                    if len(self.mic_buffer) == 0:
                        time.sleep(0.01)
                        continue

                    mic_timestamp, mic_chunk = self.mic_buffer.popleft()

                    # Check if robot is speaking or spoke recently
                    time_since_robot_audio = time.time() - self.last_robot_audio_time
                    robot_active = self.robot_speaking or time_since_robot_audio < 1.0

                    # Find the robot audio chunk closest in time to the mic chunk
                    robot_chunk = None
                    if robot_active and len(self.robot_buffer) > 0:
                        # Look for robot audio within sync tolerance
                        sync_tolerance_ns = int(self.sync_tolerance_ms * 1e6)
                        best_match = None
                        best_diff = float("inf")

                        # Find closest matching timestamp
                        for i, (robot_ts, robot_data) in enumerate(self.robot_buffer):
                            time_diff = abs(robot_ts - mic_timestamp)
                            if time_diff < best_diff:
                                best_diff = time_diff
                                best_match = i

                        # Use the match if within tolerance
                        if best_match is not None and best_diff < sync_tolerance_ns:
                            _, robot_chunk = self.robot_buffer[best_match]
                            # Remove used and older chunks
                            for _ in range(best_match + 1):
                                if len(self.robot_buffer) > 0:
                                    self.robot_buffer.popleft()

                # Process audio
                if robot_chunk is not None and len(robot_chunk) > 0:
                    # AEC mode: cancel echo
                    output = self.aec.process_streaming(robot_chunk, mic_chunk)

                    if len(output) > 0:
                        # Normalize if enabled
                        if self.enable_normalization:
                            output = normalize_audio(output, target_level=0.8)

                        # Convert back to int16 and publish
                        output_int16 = (output * 32768.0).astype(np.int16)
                        self.output_publisher.publish(
                            AudioData(data=output_int16.tobytes())
                        )
                else:
                    # Pass-through mode: no robot audio, just forward mic
                    if self.enable_normalization:
                        mic_chunk = normalize_audio(mic_chunk, target_level=0.8)

                    mic_int16 = (mic_chunk * 32768.0).astype(np.int16)
                    self.output_publisher.publish(AudioData(data=mic_int16.tobytes()))

            except Exception as e:
                self.get_logger().error(f"Error in AEC processing loop: {e}")
                time.sleep(0.1)

    def publish_status(self, message: str):
        """Publish status message."""
        self.status_publisher.publish(String(data=message))
        self.get_logger().info(f"AEC Status: {message}")

    def destroy_node(self):
        """Clean up resources."""
        self.should_process = False
        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AECNode()
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
