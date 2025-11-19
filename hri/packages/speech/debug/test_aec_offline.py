#!/usr/bin/env python3
"""
Standalone STFT AEC test script.

This script can be used to test the AEC implementation offline with audio files.
Similar to the original test script you provided, but integrated with the project structure.

Usage:
    python test_aec_offline.py --robot robot.wav --mic mic.wav [options]
"""

import argparse
import os
import sys

import numpy as np
import soundfile as sf
from scipy.signal import resample_poly

# Add speech module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from speech.stft_aec import STFTEchoCanceller, normalize_audio, rms_normalize


def to_mono(sig: np.ndarray) -> np.ndarray:
    """Convert stereo to mono."""
    if sig.ndim == 1:
        return sig
    return np.mean(sig, axis=1)


def resample_to(sig: np.ndarray, sr_in: int, sr_out: int) -> np.ndarray:
    """Resample audio to target sample rate."""
    if sr_in == sr_out:
        return sig
    from math import gcd

    g = gcd(sr_in, sr_out)
    up = sr_out // g
    down = sr_in // g
    return resample_poly(sig, up, down)


def main():
    parser = argparse.ArgumentParser(
        description="STFT Echo Canceller test: produce robot estimate and echo-cancelled outputs."
    )
    parser.add_argument(
        "--robot",
        required=True,
        help="Path to reference robot voice WAV (far-end)",
    )
    parser.add_argument(
        "--mic",
        required=True,
        help="Path to mic capture WAV (near-end + echo + noise)",
    )
    parser.add_argument(
        "--sr",
        type=int,
        default=16000,
        help="Processing sample rate (default 16000)",
    )
    parser.add_argument(
        "--frame",
        type=int,
        default=1024,
        help="STFT frame size (default 1024)",
    )
    parser.add_argument(
        "--hop",
        type=int,
        default=256,
        help="STFT hop size (default 256)",
    )
    parser.add_argument(
        "--beta",
        type=float,
        default=0.9,
        help="PSD smoothing factor [0..1] (default 0.9)",
    )
    parser.add_argument(
        "--max_delay_ms",
        type=float,
        default=200.0,
        help="Max delay to search for alignment (ms, default 200)",
    )
    parser.add_argument(
        "--prefix",
        default="stft_aec",
        help="Output filename prefix (default 'stft_aec')",
    )
    parser.add_argument(
        "--output_dir",
        default=".",
        help="Output directory (default current directory)",
    )
    args = parser.parse_args()

    # Create output directory if it doesn't exist
    os.makedirs(args.output_dir, exist_ok=True)

    print("Loading audio files...")
    # --- Load audio ---
    robot, sr_r = sf.read(args.robot, always_2d=False)
    mic, sr_m = sf.read(args.mic, always_2d=False)

    robot = to_mono(robot).astype(np.float64)
    mic = to_mono(mic).astype(np.float64)

    print(f"Robot audio: {len(robot)} samples @ {sr_r} Hz")
    print(f"Mic audio: {len(mic)} samples @ {sr_m} Hz")

    # --- Resample to common SR for processing ---
    sr = args.sr
    print(f"Resampling to {sr} Hz...")
    robot = resample_to(robot, sr_r, sr)
    mic = resample_to(mic, sr_m, sr)

    # --- Level normalize (RMS) ---
    print("Normalizing audio levels...")
    robot = rms_normalize(robot, target_rms=0.1)
    mic = rms_normalize(mic, target_rms=0.1)

    # --- Initialize AEC ---
    print("Initializing AEC...")
    aec = STFTEchoCanceller(
        frame_size=args.frame,
        hop_size=args.hop,
        beta=args.beta,
        sample_rate=sr,
    )

    # --- Rough delay alignment (critical for AEC to lock) ---
    print("Estimating delay...")
    robot_al, mic_al, delay = aec.align_signals(robot, mic, args.max_delay_ms)
    print(f"Estimated delay: {delay} samples (~{1000.0 * delay / sr:.1f} ms)")

    # --- Run STFT AEC ---
    print("Running AEC processing...")
    y_est, e_out = aec.process_block(robot_al, mic_al)

    # Trim small pad from overlap-add tails (optional)
    tail = args.frame
    if len(y_est) > tail:
        y_est = y_est[:-tail]
        e_out = e_out[:-tail]

    # Normalize outputs to prevent clipping
    print("Normalizing outputs...")
    y_est = normalize_audio(y_est, target_level=0.8)
    e_out = normalize_audio(e_out, target_level=0.8)

    # --- Save outputs ---
    print("Saving output files...")
    output_files = {
        "robot_estimate": y_est,
        "echo_cancelled": e_out,
        "mic_aligned": mic_al[: len(e_out)],
        "robot_aligned": robot_al[: len(e_out)],
    }

    for name, data in output_files.items():
        filepath = os.path.join(args.output_dir, f"{args.prefix}_{name}.wav")
        sf.write(filepath, data, sr)
        print(f"  - {filepath}")

    print("\nDone!")
    print(f"\nOutput files in '{args.output_dir}':")
    print(f"- {args.prefix}_robot_estimate.wav   (estimated robot/echo)")
    print(f"- {args.prefix}_echo_cancelled.wav   (echo-cancelled mix)")
    print(f"- {args.prefix}_mic_aligned.wav      (aligned mic input)")
    print(f"- {args.prefix}_robot_aligned.wav    (aligned robot reference)")

    # Calculate some metrics
    print("\n=== Metrics ===")

    # Echo reduction estimate (simple energy ratio)
    mic_energy = np.sum(mic_al[: len(e_out)] ** 2)
    output_energy = np.sum(e_out**2)
    if mic_energy > 1e-10:
        echo_reduction_db = 10 * np.log10(mic_energy / (output_energy + 1e-10))
        print(f"Echo reduction: {echo_reduction_db:.1f} dB")

    print(f"Output RMS: {np.sqrt(np.mean(e_out**2)):.4f}")


if __name__ == "__main__":
    main()
