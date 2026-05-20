#!/usr/bin/env python3
"""
Throwaway: sweep audio gain values against an EI inference server to find the
setting that lifts true-keyword detections the most without also lifting noise.

Usage:
    python kws_gain_sweep.py \
        --audio-dir /workspace/src/hri/packages/speech/assets/kws_detections \
        --ei-url http://localhost:1338 \
        --threshold 0.5

Filename convention from kws_eim.py's _save_audio_window is:
    {timestamp}_{label}_{score}.wav
The middle token is used as ground-truth label when available.
"""

import argparse
import glob
import os
import re
import wave
from collections import defaultdict

import numpy as np
import requests


def load_wav_int16(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as w:
        if w.getsampwidth() != 2 or w.getnchannels() != 1:
            raise ValueError(f"{path}: expected mono 16-bit PCM")
        rate = w.getframerate()
        frames = w.readframes(w.getnframes())
    return np.frombuffer(frames, dtype=np.int16), rate


def apply_gain(window: np.ndarray, gain: float) -> np.ndarray:
    if gain == 1.0:
        return window
    boosted = window.astype(np.float32) * gain
    return np.clip(boosted, np.iinfo(np.int16).min, np.iinfo(np.int16).max).astype(
        np.int16
    )


def classify(ei_url: str, window: np.ndarray) -> dict:
    resp = requests.post(
        f"{ei_url}/api/features",
        json={"features": window.astype(float).tolist()},
        timeout=5.0,
    )
    resp.raise_for_status()
    return resp.json().get("result", {}).get("classification", {})


def best_scores_over_file(
    ei_url: str,
    audio: np.ndarray,
    gain: float,
    window_samples: int,
    hop_samples: int,
) -> dict:
    """Slide a window across the file, return per-label max score."""
    audio = apply_gain(audio, gain)
    maxes: dict[str, float] = defaultdict(float)
    if len(audio) < window_samples:
        audio = np.pad(audio, (0, window_samples - len(audio)))
    for start in range(0, len(audio) - window_samples + 1, hop_samples):
        cls = classify(ei_url, audio[start : start + window_samples])
        for label, score in cls.items():
            if score > maxes[label]:
                maxes[label] = score
    return dict(maxes)


def label_from_filename(path: str) -> str | None:
    name = os.path.splitext(os.path.basename(path))[0]
    m = re.match(r"\d{8}_\d{6}_\d+_(.+)_\d+\.\d+$", name)
    return m.group(1) if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--audio-dir", required=True)
    ap.add_argument("--ei-url", default="http://localhost:1338")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--noise-label", default="noise")
    ap.add_argument("--sample-rate", type=int, default=16000)
    ap.add_argument("--window-s", type=float, default=1.0)
    ap.add_argument("--hop-ratio", type=float, default=0.25)
    ap.add_argument(
        "--gains",
        type=float,
        nargs="+",
        default=[1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0],
    )
    args = ap.parse_args()

    wavs = sorted(glob.glob(os.path.join(args.audio_dir, "*.wav")))
    if not wavs:
        raise SystemExit(f"No WAV files in {args.audio_dir}")
    print(f"Found {len(wavs)} WAV files. Sweeping gains: {args.gains}\n")

    window_samples = int(args.sample_rate * args.window_s)
    hop_samples = max(1, int(window_samples * args.hop_ratio))

    files = []
    for path in wavs:
        audio, rate = load_wav_int16(path)
        if rate != args.sample_rate:
            print(f"  skip {os.path.basename(path)}: {rate} Hz != {args.sample_rate}")
            continue
        files.append((path, audio, label_from_filename(path)))

    header = (
        f"{'gain':>5} | {'mean best-kw':>13} | {'% above thr':>11} | "
        f"{'mean noise':>10} | {'max noise':>9} | {'kw beats noise':>14}"
    )
    print(header)
    print("-" * len(header))

    for gain in args.gains:
        best_kw_scores = []
        noise_scores = []
        above_thr = 0
        kw_beats_noise = 0

        for path, audio, truth in files:
            scores = best_scores_over_file(
                args.ei_url, audio, gain, window_samples, hop_samples
            )
            noise = scores.get(args.noise_label, 0.0)
            kw_items = {k: v for k, v in scores.items() if k != args.noise_label}
            if truth and truth in kw_items:
                best_kw_score = kw_items[truth]
            else:
                best_kw_score = max(kw_items.values()) if kw_items else 0.0

            best_kw_scores.append(best_kw_score)
            noise_scores.append(noise)
            if best_kw_score >= args.threshold:
                above_thr += 1
            if best_kw_score > noise:
                kw_beats_noise += 1

        n = len(files)
        print(
            f"{gain:>5.2f} | {np.mean(best_kw_scores):>13.3f} | "
            f"{100 * above_thr / n:>10.1f}% | "
            f"{np.mean(noise_scores):>10.3f} | {max(noise_scores):>9.3f} | "
            f"{kw_beats_noise:>7}/{n:<6}"
        )

    print(
        "\nPick the gain with the highest 'mean best-kw' and '% above thr' "
        "without 'max noise' creeping past the threshold."
    )


if __name__ == "__main__":
    main()
