#!/usr/bin/env python3

"""
Record both RAW and PROCESSED audio to .wav files for comparison.
Captures audio from Respeaker, applies ANC processing, and saves both versions.

Run inside the hri-ros container:
  python3 /workspace/src/hri/packages/speech/debug/record_audio.py

Output:
  - raw_audio.wav: Audio directly from microphone (with channel extraction)
  - processed_audio.wav: Audio after noise reduction (ANC)
"""

import os
import sys
import wave
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np

from frida_interfaces.msg import AudioData


def _add_scripts_to_path():
    scripts_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "scripts")
    )
    if scripts_dir not in sys.path:
        sys.path.append(scripts_dir)


_add_scripts_to_path()

from audio_capturer import AudioCapturer  # noqa: E402


class AudioRecorder(Node):
    """Records both raw and processed audio streams to separate .wav files"""

    def __init__(self, duration_seconds=10):
        super().__init__("audio_recorder")

        self.duration = duration_seconds
        self.sample_rate = 16000
        self.sample_width = 2  # int16 = 2 bytes

        # Storage for audio frames
        self.raw_frames = []
        self.processed_frames = []

        self.recording = True
        self.start_time = time.time()

        # Subscribe to processed audio (output of audio_capturer with ANC)
        self.create_subscription(
            AudioData, "/processedAudioChunk", self._processed_callback, 10
        )

        # Check if raw audio topic exists (may not be published)
        # If audio_capturer doesn't publish raw, we'll only save processed
        self.raw_subscriber = None
        try:
            self.raw_subscriber = self.create_subscription(
                AudioData, "/rawAudioChunk", self._raw_callback, 10
            )
        except Exception as e:
            self.get_logger().warn(f"Raw audio topic not available: {e}")

        self.get_logger().info(f"Recording for {duration_seconds} seconds...")
        self.get_logger().info("Speak into the Respeaker microphone now!")

    def _raw_callback(self, msg: AudioData):
        """Store raw audio chunks"""
        if not self.recording:
            return

        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            self.recording = False
            return

        self.raw_frames.append(bytes(msg.data))

        if len(self.raw_frames) % 50 == 0:
            samples = sum(len(f) // 2 for f in self.raw_frames)
            self.get_logger().info(f"Raw: {samples} samples ({elapsed:.1f}s)")

    def _processed_callback(self, msg: AudioData):
        """Store processed audio chunks"""
        if not self.recording:
            return

        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            self.recording = False
            self.get_logger().info("Recording complete!")
            self._save_audio()
            rclpy.shutdown()
            return

        self.processed_frames.append(bytes(msg.data))

        if len(self.processed_frames) % 50 == 0:
            samples = sum(len(f) // 2 for f in self.processed_frames)
            rms = self._calculate_rms(msg.data)
            self.get_logger().info(
                f"Processed: {samples} samples ({elapsed:.1f}s) RMS={rms:.2f}"
            )

    def _calculate_rms(self, audio_bytes):
        """Calculate RMS level of audio chunk"""
        data = np.frombuffer(audio_bytes, dtype=np.int16)
        if data.size == 0:
            return 0.0
        return float(np.sqrt(np.mean(np.square(data.astype(np.float32)))))

    def _save_audio(self):
        """Save collected audio to .wav files"""
        output_dir = "/workspace/src/hri/packages/speech/debug/"

        # Save processed audio (always available)
        if self.processed_frames:
            processed_file = os.path.join(output_dir, "processed_audio.wav")
            with wave.open(processed_file, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(self.sample_width)
                wf.setframerate(self.sample_rate)
                wf.writeframes(b"".join(self.processed_frames))

            samples = sum(len(f) // 2 for f in self.processed_frames)
            duration = samples / self.sample_rate
            self.get_logger().info(
                f"✓ Saved processed_audio.wav: {samples} samples, {duration:.2f}s"
            )

        # Save raw audio if available
        if self.raw_frames:
            raw_file = os.path.join(output_dir, "raw_audio.wav")
            with wave.open(raw_file, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(self.sample_width)
                wf.setframerate(self.sample_rate)
                wf.writeframes(b"".join(self.raw_frames))

            samples = sum(len(f) // 2 for f in self.raw_frames)
            duration = samples / self.sample_rate
            self.get_logger().info(
                f"✓ Saved raw_audio.wav: {samples} samples, {duration:.2f}s"
            )
        else:
            self.get_logger().warn(
                "No raw audio captured (topic /rawAudioChunk not available)"
            )

        self.get_logger().info(f"Audio files saved to: {output_dir}")
        self.get_logger().info("You can play them with: aplay <filename>.wav")


def main():
    rclpy.init()

    # Parse duration argument
    duration = 10  # default 10 seconds
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            print("Usage: python3 record_audio.py [duration_in_seconds]")
            sys.exit(1)

    # Start audio capturer node
    audio_capturer = AudioCapturer()
    recorder = AudioRecorder(duration_seconds=duration)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(audio_capturer)
    executor.add_node(recorder)

    # Start recording thread
    record_thread = threading.Thread(target=audio_capturer.record, daemon=True)
    record_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        audio_capturer.destroy_node()
        recorder.destroy_node()
        rclpy.shutdown()
        record_thread.join(timeout=1)


if __name__ == "__main__":
    main()
