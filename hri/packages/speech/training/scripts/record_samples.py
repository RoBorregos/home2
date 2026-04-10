#!/usr/bin/env python3
"""
Interactive recorder for wakeword training data.

Subscribes to the SAME topic kws_oww uses (/hri/processedAudioChunk) so the
recorded clips see the exact same preprocessing chain (DeepFilterNet noise
cancellation, same sample rate, same chunking). This guarantees train-time
and inference-time audio distributions match.

Usage (run inside the ROS2 workspace, with the HRI audio stack launched):

    # Record 10 positive clips of "yes", 1.5 seconds each:
    ros2 run speech record_samples.py --word yes --count 10 --duration 1.5

    # Record 30 negative clips (random speech that is NOT a wakeword):
    ros2 run speech record_samples.py --word negatives --count 30 --duration 2.0

    # Record 2 minutes of continuous background/noise:
    ros2 run speech record_samples.py --word backgrounds --count 1 --duration 120.0

The script prompts you before each clip, counts down, records the window,
and writes a 16 kHz mono int16 WAV file to:

    hri/packages/speech/training/data/raw/<word>/<word>_<timestamp>.wav
"""

import argparse
import os
import queue
import sys
import time
import wave
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node

from frida_interfaces.msg import AudioData

SAMPLE_RATE = 16000
BYTES_PER_SAMPLE = 2  # int16

# Resolve the training data directory relative to this file, whether running
# from source or from the installed `share/` directory.
_THIS_FILE = os.path.abspath(__file__)
if "install" in _THIS_FILE:
    _WS_ROOT = _THIS_FILE[: _THIS_FILE.index("install")]
    DEFAULT_OUTPUT_ROOT = os.path.join(
        _WS_ROOT, "src", "hri", "packages", "speech", "training", "data", "raw"
    )
else:
    DEFAULT_OUTPUT_ROOT = os.path.abspath(
        os.path.join(os.path.dirname(_THIS_FILE), "..", "data", "raw")
    )


class SampleRecorder(Node):
    def __init__(self, topic: str):
        super().__init__("kws_sample_recorder")
        self._audio_queue: "queue.Queue[bytes]" = queue.Queue()
        self._recording = False
        self.create_subscription(AudioData, topic, self._audio_callback, 20)
        self.get_logger().info(f"Listening on {topic}")

    def _audio_callback(self, msg: AudioData) -> None:
        if self._recording:
            self._audio_queue.put(bytes(msg.data))

    def record_window(self, duration_s: float) -> np.ndarray:
        """Record `duration_s` seconds of audio from the subscribed topic."""
        # Drain any pending chunks so the window starts clean.
        while not self._audio_queue.empty():
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                break

        target_bytes = int(duration_s * SAMPLE_RATE) * BYTES_PER_SAMPLE
        collected = bytearray()
        self._recording = True
        t0 = time.monotonic()
        # Spin the executor manually while collecting chunks so callbacks fire.
        while len(collected) < target_bytes:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                while True:
                    collected.extend(self._audio_queue.get_nowait())
            except queue.Empty:
                pass
            if time.monotonic() - t0 > duration_s + 5.0:
                self.get_logger().warn(
                    "Timed out waiting for audio — is the audio stack running?"
                )
                break
        self._recording = False

        audio = np.frombuffer(bytes(collected[:target_bytes]), dtype=np.int16)
        return audio


def save_wav(path: str, audio: np.ndarray) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(audio.tobytes())


def peak_dbfs(audio: np.ndarray) -> float:
    if audio.size == 0:
        return -120.0
    peak = float(np.max(np.abs(audio.astype(np.float32)))) / 32768.0
    if peak <= 0:
        return -120.0
    return 20.0 * np.log10(peak)


def prompt_countdown(word: str, index: int, total: int, duration_s: float) -> None:
    print(
        f"\n[{index + 1}/{total}] Get ready to say '{word}' "
        f"(recording window: {duration_s:.1f}s)"
    )
    print("Press ENTER to start recording (Ctrl+C to quit)...", end="", flush=True)
    try:
        input()
    except EOFError:
        sys.exit(0)
    for n in (3, 2, 1):
        print(f"  {n}...", end=" ", flush=True)
        time.sleep(0.6)
    print("RECORD")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--word",
        required=True,
        help="Label for the recorded samples (e.g. yes, no, stop, frida, "
        "negatives, backgrounds). Creates a subfolder with this name.",
    )
    parser.add_argument(
        "--count", type=int, default=10, help="Number of clips to record."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.5,
        help="Window length per clip in seconds (1.5 is good for single words).",
    )
    parser.add_argument(
        "--topic",
        default="/hri/processedAudioChunk",
        help="ROS topic to subscribe to.",
    )
    parser.add_argument(
        "--output-root",
        default=DEFAULT_OUTPUT_ROOT,
        help="Where to save the raw clips.",
    )
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="Don't wait for ENTER between clips (useful for one long background clip).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()
    node = SampleRecorder(topic=args.topic)
    out_dir = os.path.join(args.output_root, args.word)
    os.makedirs(out_dir, exist_ok=True)

    print(f"\nSaving clips to: {out_dir}")
    print("Tip: record from ~30 cm away, at normal volume, varying intonation.\n")

    try:
        for i in range(args.count):
            if not args.no_prompt:
                prompt_countdown(args.word, i, args.count, args.duration)
            else:
                print(
                    f"[{i + 1}/{args.count}] Recording {args.duration:.1f}s of "
                    f"'{args.word}'..."
                )
            audio = node.record_window(args.duration)
            if audio.size == 0:
                node.get_logger().error("Empty recording — skipping.")
                continue

            dbfs = peak_dbfs(audio)
            if dbfs < -45:
                print(
                    f"  warning: clip is very quiet (peak {dbfs:.1f} dBFS). "
                    "Speak louder or move closer."
                )
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = os.path.join(out_dir, f"{args.word}_{ts}.wav")
            save_wav(path, audio)
            print(f"  saved {path}  (peak {dbfs:.1f} dBFS)")
    except KeyboardInterrupt:
        print("\nInterrupted — exiting.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
