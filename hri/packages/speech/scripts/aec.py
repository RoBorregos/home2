import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import Vector3
import threading
from collections import deque


class NLMSFilter:
    """Normalized Least Mean Squares adaptive filter for acoustic echo cancellation"""

    def __init__(self, filter_length=512, step_size=0.1, regularization=1e-6):
        self.filter_length = filter_length
        self.step_size = step_size
        self.regularization = regularization

        # Initialize filter coefficients
        self.w = np.zeros(filter_length, dtype=np.float32)

        # Input signal buffer (reference signal from speaker)
        self.x_buffer = np.zeros(filter_length, dtype=np.float32)

        # Performance metrics
        self.echo_return_loss = 0.0
        self.convergence_factor = 0.0

    def update(self, reference_signal, microphone_signal):
        """
        Update filter and return echo-cancelled signal

        Args:
            reference_signal: Speaker output signal (what we played)
            microphone_signal: Microphone input signal (what we recorded)

        Returns:
            echo_cancelled_signal: Microphone signal with echo removed
        """
        # Shift buffer and add new reference sample
        self.x_buffer[1:] = self.x_buffer[:-1]
        self.x_buffer[0] = reference_signal

        # Estimate echo using current filter
        echo_estimate = np.dot(self.w, self.x_buffer)

        # Calculate error (desired signal after echo removal)
        error_signal = microphone_signal - echo_estimate

        # Normalized step size
        power = np.dot(self.x_buffer, self.x_buffer) + self.regularization
        normalized_step = self.step_size / power

        # Update filter coefficients (NLMS algorithm)
        self.w += normalized_step * error_signal * self.x_buffer

        # Calculate performance metrics
        self._update_metrics(microphone_signal, echo_estimate, error_signal)

        return error_signal

    def _update_metrics(self, mic_signal, echo_estimate, error_signal):
        """Update performance metrics"""
        # Echo Return Loss Enhancement (ERLE)
        if abs(mic_signal) > 1e-10:
            self.echo_return_loss = 20 * np.log10(
                abs(mic_signal) / (abs(error_signal) + 1e-10)
            )

        # Simple convergence measure based on filter norm change
        self.convergence_factor = np.linalg.norm(self.w)

    def reset(self):
        """Reset filter to initial state"""
        self.w.fill(0.0)
        self.x_buffer.fill(0.0)
        self.echo_return_loss = 0.0
        self.convergence_factor = 0.0


class AECNode(Node):
    """ROS2 node for Acoustic Echo Cancellation using NLMS algorithm"""

    def __init__(self):
        super().__init__("aec_node")

        # Declare parameters
        self.declare_parameter("filter_length", 512)
        self.declare_parameter("step_size", 0.1)
        self.declare_parameter("regularization", 1e-6)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("chunk_size", 1024)
        self.declare_parameter("enable_aec", True)
        self.declare_parameter("adaptation_enabled", True)

        # Get parameters
        filter_length = (
            self.get_parameter("filter_length").get_parameter_value().integer_value
        )
        step_size = self.get_parameter("step_size").get_parameter_value().double_value
        regularization = (
            self.get_parameter("regularization").get_parameter_value().double_value
        )
        self.sample_rate = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )
        self.chunk_size = (
            self.get_parameter("chunk_size").get_parameter_value().integer_value
        )
        self.enable_aec = (
            self.get_parameter("enable_aec").get_parameter_value().bool_value
        )
        self.adaptation_enabled = (
            self.get_parameter("adaptation_enabled").get_parameter_value().bool_value
        )

        # Initialize NLMS filter
        self.nlms_filter = NLMSFilter(filter_length, step_size, regularization)

        # Audio buffers for synchronization
        self.reference_buffer = deque(maxlen=self.sample_rate * 2)  # 2 seconds buffer
        self.microphone_buffer = deque(maxlen=self.sample_rate * 2)

        # Synchronization
        self.buffer_lock = threading.Lock()
        self.processing_active = False

        # Publishers
        self.clean_audio_pub = self.create_publisher(
            Float32MultiArray, "/audio/clean_audio", 10
        )

        self.aec_status_pub = self.create_publisher(
            Vector3,  # Using Vector3 for metrics: x=ERLE, y=convergence, z=enabled
            "/audio/aec_status",
            10,
        )

        # Subscribers
        self.reference_sub = self.create_subscription(
            Float32MultiArray,
            "/audio/speaker_output",  # Reference signal from speaker
            self.reference_callback,
            10,
        )

        self.microphone_sub = self.create_subscription(
            Float32MultiArray,
            "/audio/raw_microphone",  # Raw microphone input from audio_capturer
            self.microphone_callback,
            10,
        )

        # Timer for publishing status
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"AEC Node initialized with filter_length={filter_length}, step_size={step_size}"
        )
        self.get_logger().info(
            f"AEC enabled: {self.enable_aec}, Adaptation enabled: {self.adaptation_enabled}"
        )

    def reference_callback(self, msg):
        """Callback for reference signal (speaker output)"""
        if not self.enable_aec:
            return

        try:
            reference_data = np.array(msg.data, dtype=np.float32)

            with self.buffer_lock:
                self.reference_buffer.extend(reference_data)

            # Trigger processing if we have enough data
            self.process_audio()

        except Exception as e:
            self.get_logger().error(f"Error in reference callback: {e}")

    def microphone_callback(self, msg):
        """Callback for microphone signal (to be cleaned)"""
        try:
            microphone_data = np.array(msg.data, dtype=np.float32)

            with self.buffer_lock:
                self.microphone_buffer.extend(microphone_data)

            # If AEC is disabled, just pass through the signal
            if not self.enable_aec:
                self.publish_clean_audio(
                    microphone_data, msg.header if hasattr(msg, "header") else None
                )
                return

            # Trigger processing if we have enough data
            self.process_audio()

        except Exception as e:
            self.get_logger().error(f"Error in microphone callback: {e}")

    def process_audio(self):
        """Process audio for echo cancellation"""
        if self.processing_active or not self.enable_aec:
            return

        with self.buffer_lock:
            if (
                len(self.reference_buffer) < self.chunk_size
                or len(self.microphone_buffer) < self.chunk_size
            ):
                return

            # Get audio chunks
            ref_chunk = np.array(
                [
                    self.reference_buffer.popleft()
                    for _ in range(min(self.chunk_size, len(self.reference_buffer)))
                ]
            )
            mic_chunk = np.array(
                [
                    self.microphone_buffer.popleft()
                    for _ in range(min(self.chunk_size, len(self.microphone_buffer)))
                ]
            )

        self.processing_active = True

        try:
            # Ensure same length
            min_length = min(len(ref_chunk), len(mic_chunk))
            ref_chunk = ref_chunk[:min_length]
            mic_chunk = mic_chunk[:min_length]

            # Apply NLMS filter sample by sample
            clean_chunk = np.zeros_like(mic_chunk)

            for i in range(len(mic_chunk)):
                if self.adaptation_enabled:
                    clean_chunk[i] = self.nlms_filter.update(ref_chunk[i], mic_chunk[i])
                else:
                    # Just apply current filter without adaptation
                    self.nlms_filter.x_buffer[1:] = self.nlms_filter.x_buffer[:-1]
                    self.nlms_filter.x_buffer[0] = ref_chunk[i]
                    echo_estimate = np.dot(
                        self.nlms_filter.w, self.nlms_filter.x_buffer
                    )
                    clean_chunk[i] = mic_chunk[i] - echo_estimate

            # Publish cleaned audio
            self.publish_clean_audio(clean_chunk)

        except Exception as e:
            self.get_logger().error(f"Error in audio processing: {e}")
        finally:
            self.processing_active = False

    def publish_clean_audio(self, audio_data, header=None):
        """Publish echo-cancelled audio"""
        try:
            msg = Float32MultiArray()
            if header:
                msg.header = header
            else:
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "microphone"

            msg.data = audio_data.tolist()
            self.clean_audio_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing clean audio: {e}")

    def publish_status(self):
        """Publish AEC status and metrics"""
        try:
            msg = Vector3()
            msg.x = float(self.nlms_filter.echo_return_loss)  # ERLE in dB
            msg.y = float(self.nlms_filter.convergence_factor)  # Convergence metric
            msg.z = float(self.enable_aec)  # AEC enabled flag

            self.aec_status_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

    def reset_filter(self):
        """Reset the NLMS filter"""
        with self.buffer_lock:
            self.nlms_filter.reset()
            self.reference_buffer.clear()
            self.microphone_buffer.clear()

        self.get_logger().info("AEC filter reset")

    def set_aec_enabled(self, enabled):
        """Enable/disable AEC processing"""
        self.enable_aec = enabled
        if enabled:
            self.get_logger().info("AEC enabled")
        else:
            self.get_logger().info("AEC disabled - audio passthrough mode")

    def set_adaptation_enabled(self, enabled):
        """Enable/disable filter adaptation"""
        self.adaptation_enabled = enabled
        if enabled:
            self.get_logger().info("AEC adaptation enabled")
        else:
            self.get_logger().info("AEC adaptation disabled - using fixed filter")


def main(args=None):
    rclpy.init(args=args)

    try:
        aec_node = AECNode()

        # Use MultiThreadedExecutor for better performance
        from rclpy.executors import MultiThreadedExecutor

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(aec_node)

        aec_node.get_logger().info("AEC Node started. Listening for audio streams...")

        try:
            executor.spin()
        except KeyboardInterrupt:
            aec_node.get_logger().info("Shutting down AEC Node...")
        finally:
            executor.shutdown()
            aec_node.destroy_node()

    except Exception as e:
        print(f"Error starting AEC node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
