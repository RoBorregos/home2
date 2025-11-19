#!/usr/bin/env python3
"""External test: record raw audio from a topic, process it and play/publish result.

This script records `duration` seconds from `input_topic` (default `/rawAudioChunk`),
saves a raw wav, runs a processing pipeline (reduce_noise + regulate_gain by default),
saves the processed wav, plays it locally, and optionally republishes it to `output_topic`.

Usage example:
  python3 external_record_and_play.py --duration 5 --play --publish
  python3 src/hri/packages/speech/scripts/external_record_and_play.py --duration 5 --play --ab_test

Requires: rclpy, numpy, soundfile, sounddevice, speech.audio_processing
"""
import argparse
import os
import time
from typing import List

import sys
import numpy as np
import rclpy
from rclpy.node import Node
import soundfile as sf
import sounddevice as sd

from frida_interfaces.msg import AudioData

# Try normal import, otherwise add package parent to sys.path for direct script runs
try:
    from speech.audio_processing import load_wav, write_wav, reduce_noise, regulate_gain
except Exception:
    # script likely run directly from repo. Add packages/speech to sys.path
    from pathlib import Path

    repo_pkg_dir = Path(__file__).resolve().parents[1]
    if str(repo_pkg_dir) not in sys.path:
        sys.path.insert(0, str(repo_pkg_dir))
    from speech.audio_processing import load_wav, write_wav, reduce_noise, regulate_gain


class RecorderNode(Node):
    def __init__(self, input_topic: str):
        super().__init__("external_recorder")
        self.chunks: List[bytes] = []
        self.sub = self.create_subscription(AudioData, input_topic, self.cb, 50)

    def cb(self, msg: AudioData):
        if msg.data:
            self.chunks.append(bytes(msg.data))


def bytes_to_float_array(b: bytes) -> np.ndarray:
    if not b:
        return np.zeros(0, dtype=np.float32)
    arr = np.frombuffer(b, dtype=np.int16).astype(np.float32) / 32768.0
    return arr


def float_to_int16_bytes(y: np.ndarray) -> bytes:
    y_int = np.clip(y, -1.0, 1.0) * 32767.0
    return y_int.astype(np.int16).tobytes()


def _rms(y: np.ndarray) -> float:
    y = np.asarray(y)
    return float(np.sqrt(np.mean(y ** 2) + 1e-12))


def compute_snr(original: np.ndarray, processed: np.ndarray) -> float:
    """Estimate SNR improvement using noise = original - processed."""
    orig_rms = _rms(original)
    noise = original - processed
    noise_rms = _rms(noise)
    if noise_rms < 1e-12:
        return float("inf")
    return 20.0 * np.log10(orig_rms / noise_rms)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0, help="Seconds to record")
    parser.add_argument("--input_topic", default="/rawAudioChunk")
    parser.add_argument("--output_topic", default="/processedAudioChunk")
    parser.add_argument("--sr", type=int, default=16000)
    parser.add_argument("--out_dir", default="/tmp/speech_test")
    parser.add_argument("--play", action="store_true", help="Play processed audio locally")
    parser.add_argument("--publish", action="store_true", help="Publish processed audio to output topic")
    parser.add_argument("--pipeline", nargs="*", default=["reduce_noise", "regulate_gain"],
                        help="Processing steps to apply")
    parser.add_argument("--ab_test", action="store_true", help="Save/play both raw and processed for A/B comparison")
    parser.add_argument("--play_delay", type=float, default=1.0, help="Seconds between playing raw and processed in A/B test")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    rclpy.init()
    node = RecorderNode(args.input_topic)

    node.get_logger().info(f"Recording from {args.input_topic} for {args.duration}s...")
    start = time.time()
    try:
        while time.time() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")

    # concatenate
    raw_bytes = b"".join(node.chunks)
    if len(raw_bytes) == 0:
        node.get_logger().warn("No audio received on topic. Exiting.")
        node.destroy_node()
        rclpy.shutdown()
        return

    # convert to float array
    y = bytes_to_float_array(raw_bytes)
    sr = args.sr

    raw_path = os.path.join(args.out_dir, "raw_recording.wav")
    write_wav(raw_path, y, sr)
    node.get_logger().info(f"Saved raw recording: {raw_path}")

    # processing pipeline
    y_proc = y.copy()
    if "reduce_noise" in args.pipeline:
        try:
            y_proc = reduce_noise(y_proc, sr)
        except Exception as e:
            node.get_logger().warn(f"reduce_noise failed: {e}")
    if "regulate_gain" in args.pipeline:
        y_proc = regulate_gain(y_proc, target_rms=0.05)

    # If A/B test requested, also save an unprocessed copy (raw already saved)
    if args.ab_test:
        ab_raw_path = os.path.join(args.out_dir, "ab_raw.wav")
        write_wav(ab_raw_path, y, sr)
        node.get_logger().info(f"Saved A/B raw copy: {ab_raw_path}")

    proc_path = os.path.join(args.out_dir, "processed_recording.wav")
    write_wav(proc_path, y_proc, sr)
    node.get_logger().info(f"Saved processed recording: {proc_path}")


    # If A/B test: compute simple metrics
    if args.ab_test:
        try:
            snr = compute_snr(y, y_proc)
            node.get_logger().info(f"Estimated SNR (orig vs processed): {snr:.2f} dB")
            node.get_logger().info(f"RMS original: {_rms(y):.6f}, RMS processed: {_rms(y_proc):.6f}")
        except Exception as e:
            node.get_logger().warn(f"Failed to compute A/B metrics: {e}")

    # Play if requested. In A/B mode, play raw then processed separated by a delay
    if args.play:
        try:
            if args.ab_test:
                node.get_logger().info("Playing A/B: raw first, then processed...")
                data_raw, srate = sf.read(raw_path)
                sd.play(data_raw, srate)
                sd.wait()
                time.sleep(args.play_delay)
            data, srate = sf.read(proc_path)
            sd.play(data, srate)
            sd.wait()
            node.get_logger().info("Finished playback")
        except Exception as e:
            node.get_logger().warn(f"Playback failed: {e}")

    # Publish processed audio as AudioData (int16 bytes)
    if args.publish:
        pub = node.create_publisher(AudioData, args.output_topic, 10)
        out_bytes = float_to_int16_bytes(y_proc)
        msg = AudioData()
        msg.data = bytes(out_bytes)
        # publish a few times to ensure subscribers get it
        for _ in range(3):
            pub.publish(msg)
            time.sleep(0.05)
        node.get_logger().info(f"Published processed audio to {args.output_topic}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
