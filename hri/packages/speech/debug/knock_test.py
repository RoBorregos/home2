#!/usr/bin/env python3
"""
knock_test.py — live-mic test for the DSP knock detector.

Fully standalone (no ROS, no EI server). Run it, then knock on a desk/door; each
confirmed burst prints a line and a live dBFS meter helps tune ``--min-db``
(filtro 1). Speech and steady noise should stay quiet.

    python3 knock_test.py
    python3 knock_test.py --list-devices | --device 2
    python3 knock_test.py --min-db -45 --onsets 1 --no-bandpass --no-meter

Requires: numpy, pyaudio (scipy optional, enables the band-pass stage).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
import pyaudio

# Import the speech package without a built ROS workspace: add the package root.
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from speech.knock_detection_utils import (  # noqa: E402
    KnockDetector,
    KnockDetectorConfig,
)

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


def dbfs(chunk: np.ndarray) -> float:
    rms = float(np.sqrt(np.mean(chunk.astype(np.float64) ** 2)) + 1e-9)
    return 20.0 * math.log10(max(rms, 1e-9) / INT16_MAX)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live-mic test for the DSP knock detector",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--list-devices", action="store_true", help="list input devices and exit"
    )
    parser.add_argument("--device", type=int, default=None, help="input device index")
    parser.add_argument("--rate", type=int, default=16000, help="sample rate (Hz)")
    parser.add_argument("--chunk", type=int, default=1024, help="frames per mic read")
    parser.add_argument(
        "--min-db", type=float, default=-40.0, help="energy gate (filtro 1), dBFS"
    )
    parser.add_argument(
        "--onsets", type=int, default=2, help="onsets needed to confirm a knock"
    )
    parser.add_argument(
        "--onset-ratio", type=float, default=3.0, help="onset rise over baseline"
    )
    parser.add_argument(
        "--window", type=float, default=1.5, help="onset pattern window (s)"
    )
    parser.add_argument(
        "--no-bandpass", action="store_true", help="disable band-pass stage"
    )
    parser.add_argument(
        "--no-meter", action="store_true", help="hide the live level meter"
    )
    args = parser.parse_args()

    if args.list_devices:
        list_devices()
        return

    cfg = KnockDetectorConfig(
        sample_rate=args.rate,
        min_db=args.min_db,
        min_onsets=args.onsets,
        onset_ratio=args.onset_ratio,
        pattern_window_s=args.window,
        use_bandpass=not args.no_bandpass,
    )
    detector = KnockDetector(cfg)

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
    print(" DSP KNOCK DETECTOR — live mic test")
    print(
        f"  rate={args.rate} Hz  min_db={args.min_db} dBFS (filtro 1)  "
        f"onsets={args.onsets}  bandpass={'off' if args.no_bandpass else 'on'}"
    )
    print("  Knock on a desk/door. Ctrl-C to stop.")
    print("=" * 64)

    knock_count = 0
    meter_max = -120.0
    last_meter = time.time()
    try:
        while True:
            data = stream.read(args.chunk, exception_on_overflow=False)
            chunk = np.frombuffer(data, dtype=np.int16)

            meter_max = max(meter_max, dbfs(chunk))

            for ev in detector.process(chunk):
                knock_count += 1
                print(
                    f"\n>>> KNOCK #{knock_count} detected  "
                    f"score={ev.score:.2f}  onsets={ev.num_onsets}  "
                    f"peak={ev.peak_db:.1f} dBFS  (t={ev.time_s:.1f}s)"
                )

            now = time.time()
            if not args.no_meter and (now - last_meter) >= 0.25:
                gate = "OPEN" if meter_max >= args.min_db else "shut"
                bar_len = int(np.clip((meter_max + 60) / 60 * 30, 0, 30))
                bar = "#" * bar_len + "-" * (30 - bar_len)
                print(
                    f"\rlevel [{bar}] {meter_max:6.1f} dBFS  gate:{gate}  "
                    f"knocks:{knock_count}   ",
                    end="",
                    flush=True,
                )
                meter_max = -120.0
                last_meter = now
    except KeyboardInterrupt:
        print(f"\n\nStopped. Total knocks detected: {knock_count}")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()


if __name__ == "__main__":
    main()
