#!/usr/bin/env python3
"""Runtime AEC test using the integrated AEC pipeline.

This script:
- Calls the TTS service `/hri/speech/speak` with the provided text.
- Subscribes to `/rawAudioChunk`, `/robot_audio_output`, `/aec_audio_output` and `/saying`.
- Collects audio frames published on those topics during the test and writes them to WAV files.
- Optionally plays back the recorded files (if paplay/aplay available).

Usage:
    python3 test_aec_runtime.py --text "Hello from test" --timeout 10 --sr 16000
    python3 hri/packages/speech/scripts/test_aec_runtime.py --text "Hello, this is an echo cancellation test" --timeout 10 --out /tmp/aec_test --play
    python3 hri/packages/speech/scripts/aec_monitor.py
    ros2 node list | grep aec
    ros2 topic info /aec_audio_output
    ros2 topic echo /aec_status

Run this inside the HRI environment (container) where ROS2 and the AEC nodes are running.
"""

import argparse
import os
import time
import wave
import subprocess

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from frida_interfaces.msg import AudioData
from frida_interfaces.srv import Speak


class AECRuntimeTester(Node):
    def __init__(self, text: str, timeout: float, sample_rate: int, out_dir: str):
        super().__init__("aec_runtime_tester")
        self.text = text
        self.timeout = timeout
        self.sample_rate = sample_rate
        self.out_dir = out_dir

        # Buffers for raw bytes
        self.raw_chunks = []
        self.robot_chunks = []
        self.aec_chunks = []

        # Saying state
        self.robot_speaking = False
        self.saying_seen = False

        # Subscribers
        self.create_subscription(AudioData, "/rawAudioChunk", self.raw_cb, 50)
        self.create_subscription(AudioData, "/robot_audio_output", self.robot_cb, 50)
        self.create_subscription(AudioData, "/aec_audio_output", self.aec_cb, 50)
        self.create_subscription(Bool, "/saying", self.saying_cb, 10)

        # Service client for TTS
        self.speak_client = self.create_client(Speak, "/hri/speech/speak")
        if not self.speak_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                "TTS service /hri/speech/speak not available (continuing, but test will not trigger robot TTS)"
            )

    def raw_cb(self, msg: AudioData):
        # AudioData.data contains raw int16 bytes
        self.raw_chunks.append(msg.data)

    def robot_cb(self, msg: AudioData):
        self.robot_chunks.append(msg.data)

    def aec_cb(self, msg: AudioData):
        self.aec_chunks.append(msg.data)

    def saying_cb(self, msg: Bool):
        self.saying_seen = True
        self.robot_speaking = bool(msg.data)

    def call_tts(self):
        if not self.speak_client.service_is_ready():
            self.get_logger().warning(
                "TTS service not ready, skipping local service call"
            )
            return False
        req = Speak.Request()
        req.text = self.text
        req.speed = 1.0
        # fut = self.speak_client.call_async(req)
        # don't block too long - the TTS node will publish /saying
        return True

    def write_wav(self, filename: str, chunks: list):
        if not chunks:
            self.get_logger().warning(f"No data for {filename}")
            return None
        path = os.path.join(self.out_dir, filename)
        data = b"".join(chunks)
        try:
            with wave.open(path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)  # int16
                wf.setframerate(self.sample_rate)
                wf.writeframes(data)
            self.get_logger().info(f"Wrote {path} ({len(data)} bytes)")
            return path
        except Exception as e:
            self.get_logger().error(f"Failed to write {path}: {e}")
            return None

    def run_test(self):
        start = time.time()

        # Make sure output dir exists
        os.makedirs(self.out_dir, exist_ok=True)

        # Trigger TTS (best-effort)
        self.get_logger().info(f'Calling TTS with text: "{self.text}"')
        self.call_tts()

        # Wait until robot stops speaking or timeout
        deadline = start + self.timeout
        # If we never see /saying, fall back to timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            # if we saw saying at least once and it's now false, finish
            if self.saying_seen and not self.robot_speaking:
                self.get_logger().info("Robot finished speaking (detected via /saying)")
                break

        # Allow a small grace period to collect final frames
        time.sleep(0.2)

        # Write WAVs
        raw_path = self.write_wav("mic_raw.wav", self.raw_chunks)
        robot_path = self.write_wav("robot_ref.wav", self.robot_chunks)
        aec_path = self.write_wav("aec_output.wav", self.aec_chunks)

        return raw_path, robot_path, aec_path


def play_file(path: str):
    if not path:
        return
    # prefer paplay, then aplay, then ffplay
    for cmd in (
        ["paplay", path],
        ["aplay", path],
        ["ffplay", "-nodisp", "-autoexit", path],
    ):
        if shutil_which(cmd[0]):
            try:
                subprocess.run(cmd, check=False)
                return
            except Exception:
                pass
    print(f"No playback available. File saved at: {path}")


def shutil_which(name: str) -> bool:
    from shutil import which

    return which(name) is not None


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--text", "-t", default="Hello from AEC test", help="Text to speak"
    )
    parser.add_argument(
        "--timeout", "-T", type=float, default=8.0, help="Max seconds to wait"
    )
    parser.add_argument(
        "--sr", type=int, default=16000, help="Sample rate for WAV files"
    )
    parser.add_argument("--out", "-o", default="/tmp/aec_test", help="Output directory")
    parser.add_argument(
        "--play", action="store_true", help="Play the AEC output after test"
    )
    args = parser.parse_args(argv)

    rclpy.init()
    tester = AECRuntimeTester(
        text=args.text, timeout=args.timeout, sample_rate=args.sr, out_dir=args.out
    )
    try:
        raw, robot, aec = tester.run_test()
        print("\nTest completed. Files:")
        print("  mic raw:    ", raw)
        print("  robot ref:  ", robot)
        print("  aec out:    ", aec)
        if args.play and aec:
            print("\nPlaying AEC output...")
            play_file(aec)
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
