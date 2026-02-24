#!/usr/bin/env python3

import os
import sys
import time
import wave
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from frida_interfaces.msg import AudioData

# Import reduce_noise only (add scripts dir to path for direct execution)
SCRIPTS_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.append(os.path.abspath(SCRIPTS_DIR))
from audio_capturer import reduce_noise  # noqa: E402


class AudioDumpTest(Node):
    def __init__(self):
        super().__init__("audio_dump_test")

        self.declare_parameter("audio_topic", "/rawAudioChunk")
        self.declare_parameter("duration_sec", 6.0)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("output_dir", "/workspace/src/hri/packages/speech/debug")
        self.declare_parameter("raw_filename", "raw_audio.wav")
        self.declare_parameter("processed_filename", "processed_audio.wav")
        self.declare_parameter("n_std_thresh", 0.5)
        self.declare_parameter("prop_decrease", 1.0)
        self.declare_parameter("n_fft", 512)
        self.declare_parameter("hop_length", 32)
        self.declare_parameter("min_gain", 0.05)

        self.audio_topic = (
            self.get_parameter("audio_topic").get_parameter_value().string_value
        )
        self.duration_sec = (
            self.get_parameter("duration_sec").get_parameter_value().double_value
        )
        self.sample_rate = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )
        self.output_dir = (
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        self.raw_filename = (
            self.get_parameter("raw_filename").get_parameter_value().string_value
        )
        self.processed_filename = (
            self.get_parameter("processed_filename").get_parameter_value().string_value
        )
        self.n_std_thresh = (
            self.get_parameter("n_std_thresh").get_parameter_value().double_value
        )
        self.prop_decrease = (
            self.get_parameter("prop_decrease").get_parameter_value().double_value
        )
        self.n_fft = self.get_parameter("n_fft").get_parameter_value().integer_value
        self.hop_length = (
            self.get_parameter("hop_length").get_parameter_value().integer_value
        )
        self.min_gain = (
            self.get_parameter("min_gain").get_parameter_value().double_value
        )

        self.raw_frames = []
        self.proc_frames = []

        self.noise_frames = []
        self.noise_clip: Optional[np.ndarray] = None
        self.noise_target = int(0.5 * self.sample_rate)

        self.started_at = None
        self.finished = False

        self.create_subscription(AudioData, self.audio_topic, self.audio_callback, 10)

        self.get_logger().info(
            f"AudioDumpTest listening on {self.audio_topic} for {self.duration_sec}s"
        )

    def audio_callback(self, msg: AudioData):
        if self.finished:
            return

        now = time.time()
        if self.started_at is None:
            self.started_at = now

        raw_bytes = bytes(msg.data)
        self.raw_frames.append(raw_bytes)

        # Build noise clip from initial frames
        audio_arr = np.frombuffer(raw_bytes, dtype=np.int16)
        if self.noise_clip is None:
            self.noise_frames.append(audio_arr)
            total = sum(f.shape[0] for f in self.noise_frames)
            if total >= self.noise_target:
                concat = np.concatenate(self.noise_frames)
                self.noise_clip = concat[: self.noise_target].astype(np.float32)
                self.noise_frames = []
                self.get_logger().info("Noise clip captured for processing.")

        # Process chunk (if noise clip ready). If not, pass-through.
        if self.noise_clip is not None:
            processed = reduce_noise(
                audio_arr.astype(np.float32),
                sr=self.sample_rate,
                noise_clip=self.noise_clip,
                n_std_thresh=self.n_std_thresh,
                prop_decrease=self.prop_decrease,
                n_fft=self.n_fft,
                hop_length=self.hop_length,
                min_gain=self.min_gain,
            )
            processed_int16 = np.clip(processed, -32768, 32767).astype(np.int16)
            self.proc_frames.append(processed_int16.tobytes())

            # Debug: log noise clip RMS occasionally
            if len(self.proc_frames) == 10:
                noise_rms = np.sqrt(np.mean(self.noise_clip**2))
                self.get_logger().info(
                    f"[DEBUG] Noise clip RMS: {noise_rms:.2f} | Processing active"
                )
        else:
            self.proc_frames.append(raw_bytes)

        if now - self.started_at >= self.duration_sec:
            self.finished = True
            self.write_files()
            self.log_stats()
            self.get_logger().info("Capture complete.")

    def write_files(self):
        os.makedirs(self.output_dir, exist_ok=True)
        raw_path = os.path.join(self.output_dir, self.raw_filename)
        proc_path = os.path.join(self.output_dir, self.processed_filename)

        self._write_wav(raw_path, self.raw_frames)
        self._write_wav(proc_path, self.proc_frames)

        self.get_logger().info(f"Saved raw wav: {raw_path}")
        self.get_logger().info(f"Saved processed wav: {proc_path}")

    def _write_wav(self, path: str, frames):
        with wave.open(path, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # int16
            wf.setframerate(self.sample_rate)
            wf.writeframes(b"".join(frames))

    def log_stats(self):
        raw = np.frombuffer(b"".join(self.raw_frames), dtype=np.int16).astype(
            np.float32
        )
        proc = np.frombuffer(b"".join(self.proc_frames), dtype=np.int16).astype(
            np.float32
        )

        if raw.size == 0 or proc.size == 0:
            self.get_logger().warn("No audio captured for stats.")
            return

        min_len = min(raw.size, proc.size)
        raw = raw[:min_len]
        proc = proc[:min_len]

        raw_rms = np.sqrt(np.mean(raw**2))
        proc_rms = np.sqrt(np.mean(proc**2))
        diff_rms = np.sqrt(np.mean((raw - proc) ** 2))

        self.get_logger().info(
            f"RMS raw: {raw_rms:.2f} | RMS processed: {proc_rms:.2f} | RMS diff: {diff_rms:.2f}"
        )


def main():
    rclpy.init()

    dump_node = AudioDumpTest()

    try:
        rclpy.spin(dump_node)
    except KeyboardInterrupt:
        pass
    finally:
        dump_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
