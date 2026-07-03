#!/usr/bin/env python3
"""
ding_dong_test.py — live-mic / WAV test for the DSP "ding-dong" detector.

Fully standalone (no ROS, no EI server) and self-calibrating: there is nothing
to tune per doorbell. It learns the ambient level, finds pitched notes by
autocorrelation clarity (scale-free), and confirms a "ding-dong" when it sees
two stable notes with the second lower than the first.

Just run it and ring a doorbell; each confirmed ding-dong prints the notes it
matched. The live meter shows the adaptive floor, current level, dominant pitch
and clarity, so you can *see* why something did or didn't fire — but you should
not need to change any threshold.

    python3 ding_dong_test.py                     # live mic, fully automatic
    python3 ding_dong_test.py --list-devices | --device 2
    python3 ding_dong_test.py --wav doorbell.wav  # replay a recording offline
    python3 ding_dong_test.py --any-order --tones 1   # single-note bells

Advanced (rarely needed) knobs mirror the detector config; defaults auto-adapt:
    --margin (dB above floor)  --clarity (0..1)  --window / --max-gap (s)

Requires: numpy, pyaudio (pyaudio only for live mic; --wav needs just numpy).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
import wave

import numpy as np

# Import the speech package without a built ROS workspace: add the package root.
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from speech.ding_dong_detection_utils import (  # noqa: E402
    DingDongDetector,
    DingDongDetectorConfig,
)

INT16_MAX = 32768.0


def list_devices() -> None:
    import pyaudio

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


def build_config(args) -> DingDongDetectorConfig:
    return DingDongDetectorConfig(
        sample_rate=args.rate,
        active_margin_db=args.margin,
        clarity_min=args.clarity,
        min_tones=args.tones,
        max_gap_s=args.max_gap,
        pattern_window_s=args.window,
        require_descending=args.descending,
    )


def report(count: int, ev) -> None:
    notes = "  ".join(f"{t.freq_hz:6.1f}Hz/{t.peak_db:5.1f}dB" for t in ev.tones)
    print(
        f"\n>>> DING-DONG #{count}  score={ev.score:.2f}  "
        f"span={ev.span_s:.2f}s  (t={ev.time_s:.1f}s)\n"
        f"    notes: {notes}"
    )


def run_wav(args) -> None:
    with wave.open(args.wav, "rb") as wf:
        rate = wf.getframerate()
        ch = wf.getnchannels()
        raw = wf.readframes(wf.getnframes())
    samples = np.frombuffer(raw, dtype=np.int16)
    if ch > 1:
        samples = samples.reshape(-1, ch)[:, 0]  # first channel
    args.rate = rate

    detector = DingDongDetector(build_config(args))
    print("=" * 68)
    print(f" DING-DONG DETECTOR — offline WAV replay ({args.wav})")
    print(f"  rate={rate} Hz  samples={len(samples)}  ({len(samples)/rate:.1f}s)")
    print("=" * 68)

    count = 0
    for i in range(0, len(samples), args.chunk):
        for ev in detector.process(samples[i : i + args.chunk]):
            count += 1
            report(count, ev)
    for ev in detector.flush():
        count += 1
        report(count, ev)
    print(f"\nDone. Ding-dongs detected in {args.wav}: {count}")


def run_mic(args) -> None:
    import pyaudio

    detector = DingDongDetector(build_config(args))
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

    print("=" * 68)
    print(" DSP DING-DONG DOORBELL DETECTOR — live mic (self-calibrating)")
    print(
        f"  rate={args.rate} Hz  tones={args.tones}  window={args.window}s  "
        f"order={'descending' if args.descending else 'any'}"
    )
    print("  Nothing to tune — just ring a doorbell. Ctrl-C to stop.")
    print("=" * 68)

    count = 0
    meter_max = -120.0
    last_meter = time.time()
    try:
        while True:
            data = stream.read(args.chunk, exception_on_overflow=False)
            chunk = np.frombuffer(data, dtype=np.int16)
            meter_max = max(meter_max, dbfs(chunk))

            for ev in detector.process(chunk):
                count += 1
                report(count, ev)

            now = time.time()
            if not args.no_meter and (now - last_meter) >= 0.25:
                floor = detector._floor_db  # noqa: SLF001 (debug view of the gate)
                floor_s = f"{floor:6.1f}" if floor is not None else "  --  "
                gate = (
                    "OPEN"
                    if (floor is None or meter_max > floor + args.margin)
                    else "shut"
                )
                bar_len = int(np.clip((meter_max + 60) / 60 * 30, 0, 30))
                bar = "#" * bar_len + "-" * (30 - bar_len)
                print(
                    f"\rlevel [{bar}] {meter_max:6.1f} dBFS  floor:{floor_s}  "
                    f"gate:{gate}  ding-dongs:{count}   ",
                    end="",
                    flush=True,
                )
                meter_max = -120.0
                last_meter = now
    except KeyboardInterrupt:
        print(f"\n\nStopped. Total ding-dongs detected: {count}")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Self-calibrating live-mic / WAV test for the ding-dong detector",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--list-devices", action="store_true", help="list input devices and exit"
    )
    parser.add_argument("--device", type=int, default=None, help="input device index")
    parser.add_argument(
        "--wav", default=None, help="replay a WAV file instead of the mic"
    )
    parser.add_argument("--rate", type=int, default=16000, help="sample rate (Hz)")
    parser.add_argument("--chunk", type=int, default=1024, help="frames per read/step")

    # Pattern shape (what counts as a ding-dong) — safe, coarse choices.
    parser.add_argument(
        "--tones", type=int, default=2, help="notes to confirm (2 == ding-dong)"
    )
    parser.add_argument(
        "--max-gap", type=float, default=1.5, help="max note spacing (s)"
    )
    parser.add_argument("--window", type=float, default=3.5, help="pattern window (s)")
    order = parser.add_mutually_exclusive_group()
    order.add_argument(
        "--descending",
        dest="descending",
        action="store_true",
        help="require each note lower than the previous (ding>dong) [default]",
    )
    order.add_argument(
        "--any-order",
        dest="descending",
        action="store_false",
        help="accept the notes in any pitch order",
    )
    parser.set_defaults(descending=True)

    # Adaptive gate / tonality — rarely touched; both are self-scaling.
    parser.add_argument(
        "--margin", type=float, default=8.0, help="dB above the adaptive noise floor"
    )
    parser.add_argument(
        "--clarity",
        type=float,
        default=0.6,
        help="autocorrelation clarity 0..1 for a tone",
    )
    parser.add_argument("--no-meter", action="store_true", help="hide the live meter")
    args = parser.parse_args()

    if args.list_devices:
        list_devices()
        return
    if args.wav:
        run_wav(args)
        return
    run_mic(args)


if __name__ == "__main__":
    main()
