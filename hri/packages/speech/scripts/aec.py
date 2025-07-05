#!/usr/bin/env python3

import ctypes
import os
import queue
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from frida_interfaces.msg import AudioData


class SpeexAEC:
    """Speex Acoustic Echo Cancellation wrapper."""

    def __init__(self, frame_size, filter_length, sample_rate=16000):
        # Load SpeexDSP library
        library_paths = [
            # "/opt/homebrew/lib/libspeexdsp.dylib",  # macOS Homebrew (ARM)
            # "/usr/local/lib/libspeexdsp.dylib",     # macOS Homebrew (Intel)
            # "/usr/lib/libspeexdsp.so",              # Linux
            # "/usr/local/lib/libspeexdsp.so",        # Linux (local install)
            # "/usr/lib/x86_64-linux-gnu/libspeexdsp.so",  # Ubuntu/Debian
            # "/usr/lib/aarch64-linux-gnu/libspeexdsp.so"  # Ubuntu/Debian (ARM)
        ]

        speex_lib = None
        for path in library_paths:
            if os.path.exists(path):
                try:
                    speex_lib = ctypes.cdll.LoadLibrary(path)
                    break
                except Exception:
                    continue

        if speex_lib is None:
            raise RuntimeError(
                "SpeexDSP library not found. Please install it using: ./install_speexdsp.sh"
            )

        self.speex = speex_lib

        # Setup function prototypes
        self._setup_prototypes()

        # Initialize AEC state
        self.frame_size = frame_size
        self.filter_length = filter_length
        self.sample_rate = sample_rate

        self.state = self.speex.speex_echo_state_init(frame_size, filter_length)
        sr = ctypes.c_int(sample_rate)
        self.speex.speex_echo_ctl(
            self.state, 24, ctypes.byref(sr)
        )  # SPEEX_ECHO_SET_SAMPLING_RATE

        # Initialize preprocessor
        self.prep_state = self.speex.speex_preprocess_state_init(
            frame_size, sample_rate
        )

        # Enable denoise and AGC
        val = ctypes.c_int(1)
        self.speex.speex_preprocess_ctl(
            self.prep_state, 0, ctypes.byref(val)
        )  # DENOISE
        self.speex.speex_preprocess_ctl(self.prep_state, 2, ctypes.byref(val))  # AGC

        # Connect AEC to preprocessor
        aec_ptr = ctypes.c_void_p(self.state)
        self.speex.speex_preprocess_ctl(self.prep_state, 24, aec_ptr)  # SET_ECHO_STATE

    def _setup_prototypes(self):
        """Setup C function prototypes for SpeexDSP."""
        # Echo Canceller
        self.speex.speex_echo_state_init.restype = ctypes.c_void_p
        self.speex.speex_echo_state_init.argtypes = [ctypes.c_int, ctypes.c_int]

        self.speex.speex_echo_ctl.restype = ctypes.c_int
        self.speex.speex_echo_ctl.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
        ]

        self.speex.speex_echo_cancellation.restype = None
        self.speex.speex_echo_cancellation.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_short),
            ctypes.POINTER(ctypes.c_short),
            ctypes.POINTER(ctypes.c_short),
        ]

        self.speex.speex_echo_state_destroy.restype = None
        self.speex.speex_echo_state_destroy.argtypes = [ctypes.c_void_p]

        # Preprocessor
        self.speex.speex_preprocess_state_init.restype = ctypes.c_void_p
        self.speex.speex_preprocess_state_init.argtypes = [ctypes.c_int, ctypes.c_int]

        self.speex.speex_preprocess_ctl.restype = ctypes.c_int
        self.speex.speex_preprocess_ctl.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
        ]

        self.speex.speex_preprocess_run.restype = ctypes.c_int
        self.speex.speex_preprocess_run.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_short),
        ]

    def process_frame(self, near_frame, far_frame):
        """Process a frame through AEC and preprocessor."""
        # Ensure frames are the correct size
        if len(near_frame) != self.frame_size:
            near_frame = np.pad(near_frame, (0, self.frame_size - len(near_frame)))
        if len(far_frame) != self.frame_size:
            far_frame = np.pad(far_frame, (0, self.frame_size - len(far_frame)))

        # Convert to int16 and make contiguous
        near_int = np.ascontiguousarray(near_frame.astype(np.int16))
        far_int = np.ascontiguousarray(far_frame.astype(np.int16))

        # Create output buffer
        out = np.zeros_like(near_int, dtype=np.int16)

        # Get pointers
        near_ptr = near_int.ctypes.data_as(ctypes.POINTER(ctypes.c_short))
        far_ptr = far_int.ctypes.data_as(ctypes.POINTER(ctypes.c_short))
        out_ptr = out.ctypes.data_as(ctypes.POINTER(ctypes.c_short))

        # Apply echo cancellation
        self.speex.speex_echo_cancellation(self.state, near_ptr, far_ptr, out_ptr)

        # Apply preprocessing (denoise, AGC)
        self.speex.speex_preprocess_run(self.prep_state, out_ptr)

        return out

    def __del__(self):
        """Clean up resources."""
        if hasattr(self, "state"):
            self.speex.speex_echo_state_destroy(self.state)


class AECNode(Node):
    """ROS2 Node that applies Acoustic Echo Cancellation to audio streams."""

    def __init__(self):
        super().__init__("aec")
        self.get_logger().info("Initializing AEC node.")

        # Parameters
        self.declare_parameter("frame_size", 512)
        self.declare_parameter("filter_length", 2048)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("raw_audio_topic", "/rawAudioChunk")
        self.declare_parameter("far_audio_topic", "/farAudioChunk")
        self.declare_parameter("clean_audio_topic", "/cleanAudioChunk")
        self.declare_parameter("speaker_state_topic", "/saying")
        self.declare_parameter("buffer_size", 10)
        self.declare_parameter("interrupt_enabled", True)

        # Get parameters
        frame_size = (
            self.get_parameter("frame_size").get_parameter_value().integer_value
        )
        filter_length = (
            self.get_parameter("filter_length").get_parameter_value().integer_value
        )
        sample_rate = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )
        raw_audio_topic = (
            self.get_parameter("raw_audio_topic").get_parameter_value().string_value
        )
        far_audio_topic = (
            self.get_parameter("far_audio_topic").get_parameter_value().string_value
        )
        clean_audio_topic = (
            self.get_parameter("clean_audio_topic").get_parameter_value().string_value
        )
        speaker_state_topic = (
            self.get_parameter("speaker_state_topic").get_parameter_value().string_value
        )
        buffer_size = (
            self.get_parameter("buffer_size").get_parameter_value().integer_value
        )
        self.interrupt_enabled = (
            self.get_parameter("interrupt_enabled").get_parameter_value().bool_value
        )

        # Initialize AEC
        self.aec = SpeexAEC(frame_size, filter_length, sample_rate)

        # Audio buffers
        self.near_buffer = deque(maxlen=buffer_size)
        self.far_buffer = deque(maxlen=buffer_size)
        self.buffer_lock = threading.Lock()

        # Speaker state tracking
        self.is_speaking = False
        self.last_speech_time = time.time()
        self.speech_timeout = 2.0  # seconds

        # Queue for interrupt detection
        self.interrupt_queue = queue.Queue()

        # Publishers and subscribers
        self.clean_audio_publisher = self.create_publisher(
            AudioData, clean_audio_topic, 20
        )
        self.interrupt_publisher = self.create_publisher(
            String, "/speech/interrupt", 10
        )

        self.create_subscription(
            AudioData, raw_audio_topic, self.near_audio_callback, 20
        )
        self.create_subscription(
            AudioData, far_audio_topic, self.far_audio_callback, 20
        )
        self.create_subscription(
            Bool, speaker_state_topic, self.speaker_state_callback, 10
        )

        # Start processing thread
        self.processing_thread = threading.Thread(
            target=self.process_audio_loop, daemon=True
        )
        self.processing_thread.start()

        self.get_logger().info("AEC node initialized.")

    def near_audio_callback(self, msg):
        """Handle incoming near-end audio (microphone)."""
        audio_data = np.frombuffer(msg.data, dtype=np.int16)

        with self.buffer_lock:
            self.near_buffer.append(audio_data)

    def far_audio_callback(self, msg):
        """Handle incoming far-end audio (speaker output)."""
        audio_data = np.frombuffer(msg.data, dtype=np.int16)

        with self.buffer_lock:
            self.far_buffer.append(audio_data)

    def speaker_state_callback(self, msg):
        """Handle speaker state changes."""
        self.is_speaking = msg.data
        if self.is_speaking:
            self.last_speech_time = time.time()

    def process_audio_loop(self):
        """Main processing loop for AEC."""
        while rclpy.ok():
            try:
                # Check if we have audio to process
                with self.buffer_lock:
                    if len(self.near_buffer) == 0:
                        time.sleep(0.001)
                        continue

                    near_frame = self.near_buffer.popleft()

                    # Use silence for far-end if no far audio available
                    if len(self.far_buffer) > 0:
                        far_frame = self.far_buffer.popleft()
                    else:
                        far_frame = np.zeros_like(near_frame, dtype=np.int16)

                # Apply AEC processing
                clean_frame = self.aec.process_frame(near_frame, far_frame)

                # Publish clean audio
                clean_audio_msg = AudioData(data=clean_frame.tobytes())
                self.clean_audio_publisher.publish(clean_audio_msg)

                # Check for interrupt if enabled and robot is speaking
                if self.interrupt_enabled and self.is_speaking:
                    self.detect_interrupt(clean_frame)

            except Exception as e:
                self.get_logger().error(f"Error in AEC processing: {e}")
                time.sleep(0.01)

    def detect_interrupt(self, clean_frame):
        """Detect voice activity in clean audio to interrupt speech."""
        # Simple energy-based voice activity detection
        energy = np.sum(clean_frame.astype(np.float32) ** 2)
        threshold = 1000000  # Adjust based on your setup

        if energy > threshold:
            try:
                # Non-blocking queue operation
                self.interrupt_queue.put_nowait(time.time())

                # Check if we should trigger interrupt
                current_time = time.time()
                interrupt_count = 0

                # Count recent interrupts
                while not self.interrupt_queue.empty():
                    try:
                        interrupt_time = self.interrupt_queue.get_nowait()
                        if current_time - interrupt_time < 0.5:  # Within 500ms
                            interrupt_count += 1
                    except queue.Empty:
                        break

                # Trigger interrupt if enough voice activity detected
                if interrupt_count >= 3:  # 3 detections in 500ms
                    self.get_logger().info("Speech interrupt detected!")
                    self.interrupt_publisher.publish(String(data="interrupt"))

                    # Clear the queue
                    while not self.interrupt_queue.empty():
                        try:
                            self.interrupt_queue.get_nowait()
                        except queue.Empty:
                            break

            except queue.Full:
                pass  # Queue is full, ignore


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AECNode()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
