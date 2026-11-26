#!/usr/bin/env python3
"""Replay 6-channel recordings (record.py) as if they came from audio_capturer,
so speaker_verification can be tested without the microphone.

    python3 replay.py a.wav [b.wav ...] [--gap 2] [--speed 1]
"""

import argparse
import time
import wave

import numpy as np
import rclpy
from frida_constants.hri_constants import MULTICHANNEL_AUDIO_TOPIC, RAW_AUDIO_TOPIC
from frida_interfaces.msg import AudioData

CHUNK = 1024


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wavs", nargs="+")
    ap.add_argument(
        "--gap", type=float, default=2.0, help="quiet seconds between files"
    )
    ap.add_argument("--speed", type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("speaker_id_replay")
    mc_pub = node.create_publisher(AudioData, MULTICHANNEL_AUDIO_TOPIC, 20)
    mono_pub = node.create_publisher(AudioData, RAW_AUDIO_TOPIC, 20)
    time.sleep(1.0)  # let subscribers match

    period = CHUNK / 16000 / args.speed
    for path in args.wavs:
        with wave.open(path) as w:
            assert w.getnchannels() == 6 and w.getframerate() == 16000, path
            x = np.frombuffer(w.readframes(w.getnframes()), np.int16).reshape(-1, 6)
        # low-level noise between files keeps the noise models alive
        tail = np.random.default_rng(0).standard_normal((int(args.gap * 16000), 6))
        x = np.concatenate([x, (tail * 20).astype(np.int16)])
        node.get_logger().info(f"replaying {path} ({x.shape[0] / 16000:.1f} s)")
        t_next = time.monotonic()
        for s in range(0, x.shape[0] - CHUNK + 1, CHUNK):
            block = np.ascontiguousarray(x[s : s + CHUNK])
            mc_pub.publish(AudioData(data=block.tobytes()))
            mono_pub.publish(
                AudioData(data=np.ascontiguousarray(block[:, 0]).tobytes())
            )
            t_next += period
            time.sleep(max(0.0, t_next - time.monotonic()))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
