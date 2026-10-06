#!/usr/bin/env python3
"""
door_event_eval.py — offline evaluation / tuning for door_event_detection.

Replays audio files through ``DoorEventTagger`` exactly as the ROS node streams
them (16 kHz int16, 1024-sample chunks), with silence around each clip, and
reports what fired. No ROS needed.

Lay clips out by expected outcome; the folder name is the expected keyword and
anything else (e.g. ``negative/``) must not fire:

    door_eval/
      doorbell/*.wav|ogg|flac
      knock/*.wav
      negative/*.wav            # speech, TTS, doors, footsteps, party noise

    python3 door_event_eval.py door_eval/ --weights ../assets/downloads/efficientat/mn10_as_mAP_471.pt
    python3 door_event_eval.py door_eval/ --noise party.wav --snr-db 5   # add background
    python3 door_event_eval.py clip.wav --verbose                        # per-window scores

Requires: numpy, scipy, soundfile, torch.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from collections import defaultdict

import numpy as np
import soundfile as sf
from scipy.signal import resample_poly

# Import the speech package without a built ROS workspace: add the package root.
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from speech.door_event_tagger import (  # noqa: E402
    DoorEventTagger,
    DoorEventTaggerConfig,
)
from speech.efficientat import WEIGHTS_FILE  # noqa: E402

SR = 16000
CHUNK = 1024
AUDIO_EXT = (".wav", ".ogg", ".flac", ".mp3")


def load_16k(path: str) -> np.ndarray:
    audio, sr = sf.read(path, dtype="float32", always_2d=True)
    audio = audio.mean(axis=1)
    if sr != SR:
        g = np.gcd(SR, sr)
        audio = resample_poly(audio, SR // g, sr // g).astype(np.float32)
    return audio


def mix_noise(clip: np.ndarray, noise: np.ndarray, snr_db: float) -> np.ndarray:
    reps = int(np.ceil(len(clip) / len(noise)))
    noise = np.tile(noise, reps)[: len(clip)]
    p_clip = np.mean(clip**2) + 1e-12
    p_noise = np.mean(noise**2) + 1e-12
    return clip + noise * np.sqrt(p_clip / (p_noise * 10 ** (snr_db / 10)))


def pink_noise(n: int, dbfs: float, rng: np.random.Generator) -> np.ndarray:
    """Room-tone stand-in: 1/f noise at ``dbfs`` RMS (a real mic is never digital zero)."""
    spec = np.fft.rfft(rng.standard_normal(n))
    spec /= np.sqrt(np.maximum(np.arange(len(spec)), 1))
    x = np.fft.irfft(spec, n)
    return (x / (np.sqrt(np.mean(x**2)) + 1e-12) * 10 ** (dbfs / 20)).astype(np.float32)


def run_clip(
    det: DoorEventTagger, audio: np.ndarray, pad_s: float, room_db: float, verbose: bool
):
    det.reset()
    pad = np.zeros(int(pad_s * SR), dtype=np.float32)
    stream = np.concatenate([pad, audio, pad])
    if room_db > -120:
        stream = stream + pink_noise(len(stream), room_db, np.random.default_rng(0))
    pcm = (np.clip(stream, -1.0, 1.0) * 32767).astype(np.int16)
    events, best = [], 0.0
    for i in range(0, len(pcm), CHUNK):
        events += det.process(pcm[i : i + CHUNK])
        if det.last_scores is not None:
            keyword, label, score, speech = det.last_scores
            det.last_scores = None
            best = max(best, score)
            if verbose:
                print(
                    f"    t={i / SR - pad_s:5.2f}s  best {label:<10} p={score:.2f}"
                    f"  speech p={speech:.2f}"
                )
    return events, best


def collect(paths):
    """Return [(path, expected_keyword_or_None)] from files or a labelled folder tree."""
    items = []
    for p in paths:
        if os.path.isdir(p):
            for f in sorted(glob.glob(os.path.join(p, "**", "*"), recursive=True)):
                if f.lower().endswith(AUDIO_EXT):
                    items.append((f, os.path.basename(os.path.dirname(f))))
        else:
            items.append((p, None))
    return items


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("paths", nargs="+", help="audio files or a labelled folder tree")
    ap.add_argument(
        "--weights",
        default=os.path.join(
            _PKG_ROOT, "assets", "downloads", "efficientat", WEIGHTS_FILE
        ),
    )
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--threshold", type=float, default=DoorEventTaggerConfig.threshold)
    ap.add_argument(
        "--strong-threshold", type=float, default=DoorEventTaggerConfig.strong_threshold
    )
    ap.add_argument(
        "--min-consecutive", type=int, default=DoorEventTaggerConfig.min_consecutive
    )
    ap.add_argument("--window-s", type=float, default=DoorEventTaggerConfig.window_s)
    ap.add_argument("--hop-s", type=float, default=DoorEventTaggerConfig.hop_s)
    ap.add_argument(
        "--pad-s", type=float, default=5.0, help="room tone before/after each clip"
    )
    ap.add_argument(
        "--room-db",
        type=float,
        default=-50.0,
        help="room-tone level in dBFS (-120 = digital silence)",
    )
    ap.add_argument("--noise", help="background audio mixed into every clip")
    ap.add_argument("--snr-db", type=float, default=10.0)
    ap.add_argument(
        "--gain-db",
        type=float,
        default=0.0,
        help="attenuate/boost clips (e.g. -20 = far away)",
    )
    ap.add_argument(
        "--verbose", "-v", action="store_true", help="print per-window scores"
    )
    args = ap.parse_args()

    cfg = DoorEventTaggerConfig(
        threshold=args.threshold,
        strong_threshold=args.strong_threshold,
        min_consecutive=args.min_consecutive,
        window_s=args.window_s,
        hop_s=args.hop_s,
        weights_path=args.weights,
        device=args.device,
    )
    det = DoorEventTagger(cfg)
    noise = load_16k(args.noise) if args.noise else None

    stats = defaultdict(lambda: [0, 0])  # expected -> [correct, total]
    for path, expected in collect(args.paths):
        try:
            audio = load_16k(path) * (10 ** (args.gain_db / 20))
        except Exception as e:  # unsupported container/codec
            print(f"SKIP {os.path.basename(path)}: {e}")
            continue
        if noise is not None:
            audio = mix_noise(audio, noise, args.snr_db)
        events, best = run_clip(det, audio, args.pad_s, args.room_db, args.verbose)
        fired = sorted({e.keyword for e in events})
        detail = (
            ", ".join(f"{e.keyword}:{e.label}@{e.score:.2f}" for e in events) or "-"
        )

        if expected in (cfg.doorbell_keyword, cfg.knock_keyword):
            ok = expected in fired
        elif expected is not None:
            ok = not fired
        else:
            ok = None
        if expected is not None:
            stats[expected][0] += int(ok)
            stats[expected][1] += 1
        mark = {
            True: "OK  ",
            False: "MISS"
            if expected in (cfg.doorbell_keyword, cfg.knock_keyword)
            else "FP  ",
            None: "    ",
        }[ok]
        print(
            f"{mark} [{expected or '?':<8}] {os.path.basename(path):<45} max p={best:.2f}  fired: {detail}"
        )

    if stats:
        print("\nSummary:")
        for expected, (correct, total) in sorted(stats.items()):
            what = (
                "detected"
                if expected in (cfg.doorbell_keyword, cfg.knock_keyword)
                else "silent (no false trigger)"
            )
            print(f"  {expected:<10} {correct}/{total} {what}")


if __name__ == "__main__":
    main()
