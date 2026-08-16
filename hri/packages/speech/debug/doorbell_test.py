#!/usr/bin/env python3
"""
doorbell_test.py — live-mic test for the Edge Impulse doorbell detector.

Mirrors ``scripts/ei_audio_node.py`` to validate the doorbell model (and filtro
1, the dB gate) from a laptop mic. Needs a reachable EI inference server, e.g.
http://localhost:1337.

    python3 doorbell_test.py --ei-url http://localhost:1337
    python3 doorbell_test.py --list-devices | --device 2
    python3 doorbell_test.py --min-db -45 --threshold 0.8

Requires: numpy, pyaudio, requests.
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np
import pyaudio
import requests

INT16_MAX = 32768.0


def list_devices() -> None:
    p = pyaudio.PyAudio()
    print("Available input devices:")
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info["maxInputChannels"] > 0:
            print(
                f"  [{i}] {info['name']}  "
                f"(in={info['maxInputChannels']}, default_sr={int(info['defaultSampleRate'])})"
            )
    p.terminate()


def dbfs(window: np.ndarray) -> float:
    rms = float(np.sqrt(np.mean(window.astype(np.float64) ** 2)) + 1e-9)
    return 20.0 * math.log10(max(rms, 1e-9) / INT16_MAX)


def wait_for_server(ei_url: str, retries: int = 12, interval: float = 2.0) -> bool:
    for attempt in range(1, retries + 1):
        try:
            if requests.get(f"{ei_url}/api/info", timeout=3.0).ok:
                print(f"[INFO] EI server ready at {ei_url} (attempt {attempt}).")
                return True
        except requests.RequestException:
            pass
        print(f"[INFO] Waiting for EI server at {ei_url} ... ({attempt}/{retries})")
        time.sleep(interval)
    return False


def classify(ei_url: str, window: np.ndarray, noise_label: str):
    """Return (best_label, best_score, full_classification) or (None, 0.0, {})."""
    features = window.astype(float).tolist()
    try:
        resp = requests.post(
            f"{ei_url}/api/features", json={"features": features}, timeout=5.0
        )
        resp.raise_for_status()
    except requests.RequestException as exc:
        print(f"[WARN] EI inference failed: {exc}")
        return None, 0.0, {}

    classification = resp.json().get("result", {}).get("classification", {})
    best_label, best_score = None, 0.0
    for label, score in classification.items():
        if label.lower() == noise_label.lower():
            continue
        if score > best_score:
            best_label, best_score = label, score
    return best_label, best_score, classification


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live-mic test for the Edge Impulse doorbell detector",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--list-devices", action="store_true", help="list input devices and exit"
    )
    parser.add_argument("--device", type=int, default=None, help="input device index")
    parser.add_argument(
        "--ei-url", default="http://localhost:1337", help="EI inference server URL"
    )
    parser.add_argument("--rate", type=int, default=16000, help="sample rate (Hz)")
    parser.add_argument(
        "--window", type=float, default=1.0, help="inference window (s)"
    )
    parser.add_argument(
        "--hop", type=float, default=1.0, help="hop as a fraction of the window"
    )
    parser.add_argument("--chunk", type=int, default=1024, help="frames per mic read")
    parser.add_argument(
        "--min-db", type=float, default=-40.0, help="energy gate (filtro 1), dBFS"
    )
    parser.add_argument(
        "--threshold", type=float, default=0.8, help="detection sensitivity"
    )
    parser.add_argument(
        "--noise-label", default="noise", help="label treated as background"
    )
    args = parser.parse_args()

    if args.list_devices:
        list_devices()
        return

    if not wait_for_server(args.ei_url):
        print(f"[ERROR] EI server not reachable at {args.ei_url}. Start it and retry.")
        return

    window_samples = int(args.rate * args.window)
    hop_samples = max(1, int(window_samples * args.hop))

    p = pyaudio.PyAudio()
    try:
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=args.rate,
            input=True,
            input_device_index=args.device,
            frames_per_buffer=args.chunk,
        )
    except Exception as exc:
        print(f"[ERROR] Could not open input stream: {exc}")
        print("        Try '--list-devices' and pass '--device <index>'.")
        p.terminate()
        return

    print("=" * 64)
    print(" EDGE IMPULSE DOORBELL DETECTOR — live mic test")
    print(f"  ei_url={args.ei_url}  rate={args.rate} Hz  window={args.window}s")
    print(f"  min_db={args.min_db} dBFS (filtro 1)  threshold={args.threshold}")
    print("  Ring / play a doorbell. Ctrl-C to stop.")
    print("=" * 64)

    audio_buffer = np.array([], dtype=np.int16)
    gated = 0
    detections = 0
    try:
        while True:
            data = stream.read(args.chunk, exception_on_overflow=False)
            audio_buffer = np.concatenate(
                [audio_buffer, np.frombuffer(data, dtype=np.int16)]
            )

            while len(audio_buffer) >= window_samples:
                window = audio_buffer[:window_samples]
                audio_buffer = audio_buffer[hop_samples:]

                level = dbfs(window)
                # filtro 1: skip inference on quiet audio.
                if level < args.min_db:
                    gated += 1
                    print(
                        f"\r[gated] {level:6.1f} dBFS < {args.min_db}  "
                        f"(skipped EI inferences: {gated})   ",
                        end="",
                        flush=True,
                    )
                    continue

                label, score, full = classify(args.ei_url, window, args.noise_label)
                if label and score >= args.threshold:
                    detections += 1
                    print(
                        f"\n>>> DOORBELL #{detections}: {label} "
                        f"({score:.2f}) @ {level:.1f} dBFS | {full}"
                    )
                else:
                    print(
                        f"\r[listening] {level:6.1f} dBFS  "
                        f"best={label}:{score:.2f}  detections:{detections}   ",
                        end="",
                        flush=True,
                    )
    except KeyboardInterrupt:
        print(f"\n\nStopped. Doorbell detections: {detections}, gated windows: {gated}")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()


if __name__ == "__main__":
    main()
