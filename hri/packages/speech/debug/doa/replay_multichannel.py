#!/usr/bin/env python3
"""Replay 6-channel ReSpeaker WAVs as if they came from audio_capturer.

Publishes the interleaved 6 channels on /hri/rawAudioMultichannel and ch0 on
/hri/rawAudioChunk in real time (1024-sample chunks), so speaker_verification
and the spatial nodes can be tested without the microphone.

    python3 replay_multichannel.py a.wav [b.wav ...] [--gap 2] [--speed 1]
"""

import argparse
import time
import wave

import numpy as np
import rclpy
from frida_interfaces.msg import AudioData

CHUNK = 1024


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wavs", nargs="+")
    ap.add_argument("--gap", type=float, default=2.0, help="silence between files")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--mc-topic", default="/hri/rawAudioMultichannel")
    ap.add_argument("--mono-topic", default="/hri/rawAudioChunk")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("replay_multichannel")
    mc_pub = node.create_publisher(AudioData, args.mc_topic, 20)
    mono_pub = node.create_publisher(AudioData, args.mono_topic, 20)
    time.sleep(1.0)  # let subscribers match

    period = CHUNK / 16000 / args.speed
    for path in args.wavs:
        with wave.open(path) as w:
            assert w.getnchannels() == 6 and w.getframerate() == 16000, path
            x = np.frombuffer(w.readframes(w.getnframes()), np.int16).reshape(-1, 6)
        # quiet tail between files: keeps the noise model alive (low-level noise)
        tail = np.random.default_rng(0).standard_normal((int(args.gap * 16000), 6)) * 20
        x = np.concatenate([x, tail.astype(np.int16)])
        node.get_logger().info(f"replaying {path} ({x.shape[0] / 16000:.1f}s)")
        t_next = time.monotonic()
        for s in range(0, x.shape[0] - CHUNK + 1, CHUNK):
            block = np.ascontiguousarray(x[s : s + CHUNK])
            mc_pub.publish(AudioData(data=block.tobytes()))
            mono_pub.publish(
                AudioData(data=np.ascontiguousarray(block[:, 0]).tobytes())
            )
            t_next += period
            time.sleep(max(0.0, t_next - time.monotonic()))
    node.get_logger().info("replay done")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
