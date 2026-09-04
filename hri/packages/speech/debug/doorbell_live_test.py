#!/usr/bin/env python3
"""
doorbell_live_test.py — live-mic / WAV test for the robust DSP doorbell detector.

Standalone (no ROS, no EI server) and self-calibrating. It confirms any loud
sound event (any doorbell type or knock) that stands clearly above the learned
ambient floor. It will also fire on loud talking near the mic — that is by
design; in the robot the detector only listens while armed and is muted while
speaking. Test by ringing doorbells of different lengths: all should register.

    python3 doorbell_live_test.py                      # live mic
    python3 doorbell_live_test.py --list-devices | --device 2
    python3 doorbell_live_test.py --wav doorbell.wav   # replay a recording offline
    python3 doorbell_live_test.py --debug              # per-event diagnostics

Knobs (defaults auto-adapt): --confirm-margin (dB over floor) --min-event-ms

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

from speech.doorbell_detection_utils import (  # noqa: E402
    DoorbellDetector,
    DoorbellDetectorConfig,
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


def build_config(args) -> DoorbellDetectorConfig:
    return DoorbellDetectorConfig(
        sample_rate=args.rate,
        active_margin_db=args.margin,
        confirm_margin_db=args.confirm_margin,
        min_event_ms=args.min_event_ms,
        clarity_min=args.clarity,
    )


def report(count: int, ev) -> None:
    print(
        f"\n>>> DOORBELL #{count}  score={ev.score:.2f}  sim={ev.template_similarity:.2f}"
        f"  (t={ev.time_s:.1f}s)\n"
        f"    loud={ev.margin_db:.1f}dB over floor  dur={ev.dur_ms:.0f}ms  "
        f"peak={ev.peak_db:.1f}dBFS  dominant={ev.dominant_hz:.0f}Hz"
    )


def handle_events(detector, events, count, enroll, debug=False) -> int:
    """Print events and auto-enroll the first confident ring (as the ROS node does)."""
    if debug:
        for d in detector.drain_debug():
            mark = "ACCEPT" if d["reason"] == "ok" else f"reject:{d['reason']}"
            print(
                f"\n    [event] loud={d['margin_db']:5.1f}dB  dur={d['dur_ms']:5.0f}ms  "
                f"peak={d['peak_db']:5.1f}dB  dominant={d['dominant_hz']:6.0f}Hz  "
                f"clarity={d['clarity']:.2f}  -> {mark}"
            )
    for ev in events:
        count += 1
        report(count, ev)
        if enroll and not detector.has_template and ev.spectrum is not None:
            detector.enroll_template(ev.spectrum)
            print("    (enrolled as this session's reference doorbell)")
    return count


def run_wav(args) -> None:
    with wave.open(args.wav, "rb") as wf:
        rate = wf.getframerate()
        ch = wf.getnchannels()
        raw = wf.readframes(wf.getnframes())
    samples = np.frombuffer(raw, dtype=np.int16)
    if ch > 1:
        samples = samples.reshape(-1, ch)[:, 0]  # first channel
    args.rate = rate

    detector = DoorbellDetector(build_config(args))
    print("=" * 68)
    print(f" ROBUST DOORBELL DETECTOR — offline WAV replay ({args.wav})")
    print(f"  rate={rate} Hz  samples={len(samples)}  ({len(samples) / rate:.1f}s)")
    print("=" * 68)

    count = 0
    for i in range(0, len(samples), args.chunk):
        count = handle_events(
            detector, detector.process(samples[i : i + args.chunk]), count, args.enroll
        )
    count = handle_events(detector, detector.flush(), count, args.enroll, args.debug)
    print(f"\nDone. Doorbells detected in {args.wav}: {count}")


def run_mic(args) -> None:
    import pyaudio

    detector = DoorbellDetector(build_config(args))
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
    print(" ROBUST DSP DOORBELL DETECTOR — live mic (self-calibrating)")
    print(
        f"  rate={args.rate} Hz  confirm>={args.confirm_margin:.0f}dB over floor  "
        f">={args.min_event_ms:.0f}ms  enroll={'on' if args.enroll else 'off'}"
    )
    print("  Detects any LOUD sound event (any doorbell type / knock). It will")
    print("  also fire on loud talking near the mic — that is expected; in the")
    print("  robot it only listens while armed and is muted while speaking.")
    print("  Ring doorbells of different lengths. Ctrl-C to stop.")
    print("=" * 68)

    count = 0
    meter_max = -120.0
    last_meter = time.time()
    try:
        while True:
            data = stream.read(args.chunk, exception_on_overflow=False)
            chunk = np.frombuffer(data, dtype=np.int16)
            meter_max = max(meter_max, dbfs(chunk))

            count = handle_events(
                detector, detector.process(chunk), count, args.enroll, args.debug
            )

            now = time.time()
            if not args.no_meter and (now - last_meter) >= 0.25:
                floor = detector._floor_db  # noqa: SLF001 (debug view of the gate)
                floor_s = f"{floor:6.1f}" if floor is not None else "  --  "
                gate = (
                    "OPEN"
                    if (floor is None or meter_max > floor + args.margin)
                    else "shut"
                )
                tpl = "yes" if detector.has_template else "no "
                bar_len = int(np.clip((meter_max + 60) / 60 * 30, 0, 30))
                bar = "#" * bar_len + "-" * (30 - bar_len)
                print(
                    f"\rlevel [{bar}] {meter_max:6.1f} dBFS  floor:{floor_s}  "
                    f"gate:{gate}  enrolled:{tpl}  doorbells:{count}   ",
                    end="",
                    flush=True,
                )
                meter_max = -120.0
                last_meter = now
    except KeyboardInterrupt:
        print(f"\n\nStopped. Total doorbells detected: {count}")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Self-calibrating live-mic / WAV test for the robust doorbell detector",
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

    # Event confirmation — loudness, not tonal shape.
    parser.add_argument(
        "--confirm-margin",
        type=float,
        default=12.0,
        help="dB the event peak must stand above the ambient floor to confirm",
    )
    parser.add_argument(
        "--min-event-ms",
        type=float,
        default=120.0,
        help="total loud time (ms) needed to confirm an event",
    )
    enroll = parser.add_mutually_exclusive_group()
    enroll.add_argument(
        "--enroll",
        dest="enroll",
        action="store_true",
        help="auto-enroll the first ring and match later ones to it [default]",
    )
    enroll.add_argument(
        "--no-enroll", dest="enroll", action="store_false", help="disable enrollment"
    )
    parser.set_defaults(enroll=True)

    # Adaptive gate / tonality — rarely touched; both are self-scaling.
    parser.add_argument(
        "--margin",
        type=float,
        default=8.0,
        help="dB above the adaptive floor for a frame to join an event",
    )
    parser.add_argument(
        "--clarity",
        type=float,
        default=0.60,
        help="clarity 0..1 marking a frame as tonal (scoring/fingerprint only)",
    )
    parser.add_argument("--no-meter", action="store_true", help="hide the live meter")
    parser.add_argument(
        "--debug",
        action="store_true",
        help="print every loud event (loudness/duration/dominant pitch) and "
        "whether it confirmed — lower --confirm-margin / --min-event-ms if a real "
        "doorbell is rejected",
    )
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
