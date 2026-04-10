#!/usr/bin/env python3
"""
Data augmentation for wakeword training clips.

Takes the raw recordings from
    hri/packages/speech/training/data/raw/<word>/*.wav
and expands each clip into many augmented variants, written to
    hri/packages/speech/training/data/augmented/<word>/*.wav

Augmentations applied (randomly combined):
  - Gain variation (±6 dB)
  - Pitch shift (±3 semitones) via resampling
  - Time stretch (0.9× to 1.1×) via resampling
  - Random leading/trailing silence (so the word isn't always centred)
  - Additive Gaussian noise (SNR 20–35 dB)
  - Background mixing with user-provided ambient clips
    (data/backgrounds/*.wav, optional)

Output clips are always 1.28 seconds long (20480 samples at 16 kHz) so they
map to exactly 16 embedding frames, the same window length kws_oww sees.

Usage:
    python3 augment_clips.py --word yes --variants 200
    python3 augment_clips.py --word negatives --variants 50
    python3 augment_clips.py --all --variants 200
"""

import argparse
import glob
import os
import random
import sys
import wave
from typing import List, Tuple

import numpy as np

SAMPLE_RATE = 16000
TARGET_SAMPLES = 20480  # 1.28 s -> 16 embedding frames

_THIS_FILE = os.path.abspath(__file__)
DEFAULT_DATA_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(_THIS_FILE), "..", "data")
)


def read_wav(path: str) -> np.ndarray:
    with wave.open(path, "rb") as wf:
        if wf.getnchannels() != 1:
            raise ValueError(f"{path}: expected mono, got {wf.getnchannels()} channels")
        if wf.getsampwidth() != 2:
            raise ValueError(f"{path}: expected 16-bit PCM")
        if wf.getframerate() != SAMPLE_RATE:
            raise ValueError(
                f"{path}: expected {SAMPLE_RATE} Hz, got {wf.getframerate()}"
            )
        data = wf.readframes(wf.getnframes())
    return np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0


def write_wav(path: str, audio_f32: np.ndarray) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    clipped = np.clip(audio_f32, -1.0, 1.0)
    pcm = (clipped * 32767.0).astype(np.int16)
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(pcm.tobytes())


def trim_silence(audio: np.ndarray, threshold: float = 0.005) -> np.ndarray:
    """Trim leading/trailing silence based on absolute amplitude."""
    if audio.size == 0:
        return audio
    mask = np.abs(audio) > threshold
    if not mask.any():
        return audio
    start = int(np.argmax(mask))
    end = int(len(audio) - np.argmax(mask[::-1]))
    return audio[start:end]


def resample_linear(audio: np.ndarray, ratio: float) -> np.ndarray:
    """Simple linear resampling — good enough for augmentation."""
    if ratio == 1.0:
        return audio
    new_len = max(1, int(round(len(audio) / ratio)))
    idx = np.linspace(0, len(audio) - 1, new_len)
    return np.interp(idx, np.arange(len(audio)), audio).astype(np.float32)


def pitch_shift(audio: np.ndarray, semitones: float) -> np.ndarray:
    """Crude pitch shift via resampling (changes duration and pitch).

    We compensate by resampling back to the original length afterwards, so
    only the pitch changes. This is lower quality than a phase vocoder but
    good enough for data augmentation.
    """
    factor = 2 ** (semitones / 12.0)
    shifted = resample_linear(audio, 1.0 / factor)
    return resample_linear(shifted, len(shifted) / len(audio))


def time_stretch(audio: np.ndarray, rate: float) -> np.ndarray:
    """Stretch (<1) or compress (>1) duration without pitch correction."""
    return resample_linear(audio, rate)


def fit_to_length(audio: np.ndarray, target_len: int, rng: random.Random) -> np.ndarray:
    """Pad with random silence (or crop) to match target_len samples."""
    n = len(audio)
    if n >= target_len:
        start = rng.randint(0, n - target_len)
        return audio[start : start + target_len]
    pad_total = target_len - n
    left = rng.randint(0, pad_total)
    right = pad_total - left
    return np.concatenate(
        [np.zeros(left, dtype=np.float32), audio, np.zeros(right, dtype=np.float32)]
    )


def apply_gain(audio: np.ndarray, db: float) -> np.ndarray:
    return audio * (10 ** (db / 20.0))


def add_gaussian_noise(
    audio: np.ndarray, snr_db: float, rng: random.Random
) -> np.ndarray:
    sig_power = float(np.mean(audio**2)) + 1e-12
    noise_power = sig_power / (10 ** (snr_db / 10.0))
    noise = np.random.default_rng(rng.randint(0, 2**31 - 1)).normal(
        0.0, np.sqrt(noise_power), size=audio.shape
    )
    return audio + noise.astype(np.float32)


def mix_background(
    audio: np.ndarray,
    backgrounds: List[np.ndarray],
    snr_db: float,
    rng: random.Random,
) -> np.ndarray:
    if not backgrounds:
        return audio
    bg = backgrounds[rng.randint(0, len(backgrounds) - 1)]
    if len(bg) < len(audio):
        reps = int(np.ceil(len(audio) / len(bg)))
        bg = np.tile(bg, reps)
    start = rng.randint(0, max(0, len(bg) - len(audio)))
    bg_slice = bg[start : start + len(audio)]
    sig_power = float(np.mean(audio**2)) + 1e-12
    bg_power = float(np.mean(bg_slice**2)) + 1e-12
    scale = np.sqrt(sig_power / (bg_power * 10 ** (snr_db / 10.0)))
    return audio + bg_slice * scale


def load_backgrounds(bg_dir: str) -> List[np.ndarray]:
    if not os.path.isdir(bg_dir):
        return []
    clips = []
    for path in sorted(glob.glob(os.path.join(bg_dir, "*.wav"))):
        try:
            clips.append(read_wav(path))
        except Exception as e:
            print(f"  skipping background {path}: {e}")
    return clips


def augment_one(
    clean: np.ndarray,
    backgrounds: List[np.ndarray],
    rng: random.Random,
    is_long_clip: bool,
) -> np.ndarray:
    """Apply a random combination of augmentations to `clean`."""
    audio = clean.copy()

    # Trim silence on short (word) clips only; for long negatives/backgrounds
    # we want to keep the full duration so we can crop random windows.
    if not is_long_clip:
        audio = trim_silence(audio)
        if len(audio) < 800:  # < 50 ms of speech — probably a bad clip
            audio = clean.copy()

    # Pitch shift (50% of the time)
    if rng.random() < 0.5:
        audio = pitch_shift(audio, rng.uniform(-3.0, 3.0))

    # Time stretch (50% of the time)
    if rng.random() < 0.5:
        audio = time_stretch(audio, rng.uniform(0.9, 1.1))

    # Gain (always)
    audio = apply_gain(audio, rng.uniform(-6.0, 6.0))

    # Fit to training window length (random padding / cropping)
    audio = fit_to_length(audio, TARGET_SAMPLES, rng)

    # Background mix (70% of the time if we have backgrounds)
    if backgrounds and rng.random() < 0.7:
        audio = mix_background(audio, backgrounds, rng.uniform(5.0, 20.0), rng)

    # Gaussian noise (30% of the time)
    if rng.random() < 0.3:
        audio = add_gaussian_noise(audio, rng.uniform(20.0, 35.0), rng)

    # Final safety: avoid clipping
    peak = float(np.max(np.abs(audio)))
    if peak > 0.98:
        audio = audio * (0.98 / peak)

    return audio


def augment_word(
    word: str,
    raw_root: str,
    aug_root: str,
    backgrounds: List[np.ndarray],
    variants: int,
    seed: int,
) -> Tuple[int, int]:
    in_dir = os.path.join(raw_root, word)
    out_dir = os.path.join(aug_root, word)
    if not os.path.isdir(in_dir):
        print(f"  {word}: no raw clips at {in_dir}, skipping")
        return 0, 0

    rng = random.Random(seed)
    clips = sorted(glob.glob(os.path.join(in_dir, "*.wav")))
    if not clips:
        print(f"  {word}: no .wav files found in {in_dir}")
        return 0, 0

    # Clear previous augmented output for this word so re-runs are clean.
    if os.path.isdir(out_dir):
        for f in glob.glob(os.path.join(out_dir, "*.wav")):
            os.remove(f)

    is_long = word in ("negatives", "backgrounds")
    written = 0
    for clip_path in clips:
        try:
            clean = read_wav(clip_path)
        except Exception as e:
            print(f"    skip {clip_path}: {e}")
            continue

        # For long clips (multi-second negatives) we additionally slice random
        # 1.28 s windows out of each recording before augmenting, so a 2-minute
        # background recording produces many independent training windows.
        if is_long and len(clean) > TARGET_SAMPLES * 2:
            n_windows = max(1, len(clean) // TARGET_SAMPLES)
            windows = []
            for _ in range(n_windows):
                start = rng.randint(0, len(clean) - TARGET_SAMPLES)
                windows.append(clean[start : start + TARGET_SAMPLES])
        else:
            windows = [clean]

        per_window = max(1, variants // max(1, len(windows)))
        for w_idx, window in enumerate(windows):
            for v in range(per_window):
                aug = augment_one(window, backgrounds, rng, is_long)
                name = os.path.splitext(os.path.basename(clip_path))[0]
                out_path = os.path.join(out_dir, f"{name}_w{w_idx:02d}_v{v:03d}.wav")
                write_wav(out_path, aug)
                written += 1

    print(f"  {word}: {len(clips)} raw -> {written} augmented")
    return len(clips), written


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--word", help="Word to augment (folder name under data/raw).")
    parser.add_argument(
        "--all", action="store_true", help="Augment all subfolders under data/raw."
    )
    parser.add_argument(
        "--variants",
        type=int,
        default=200,
        help="Variants per raw clip (default: 200).",
    )
    parser.add_argument(
        "--data-root",
        default=DEFAULT_DATA_ROOT,
        help="Path to training/data (default: repo path).",
    )
    parser.add_argument(
        "--seed", type=int, default=0xC0FFEE, help="Random seed for reproducibility."
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.word and not args.all:
        print("error: pass --word <name> or --all", file=sys.stderr)
        sys.exit(1)

    raw_root = os.path.join(args.data_root, "raw")
    aug_root = os.path.join(args.data_root, "augmented")
    bg_dir = os.path.join(args.data_root, "backgrounds")

    backgrounds = load_backgrounds(bg_dir)
    print(f"Loaded {len(backgrounds)} background clips from {bg_dir}")

    if args.all:
        words: List[str] = []
        if os.path.isdir(raw_root):
            for name in sorted(os.listdir(raw_root)):
                if os.path.isdir(os.path.join(raw_root, name)):
                    words.append(name)
    else:
        words = [args.word]

    if not words:
        print(f"error: no word folders found under {raw_root}")
        sys.exit(1)

    print(f"Augmenting {len(words)} word(s): {', '.join(words)}")
    total_raw = total_aug = 0
    for w in words:
        r, a = augment_word(
            w, raw_root, aug_root, backgrounds, args.variants, args.seed
        )
        total_raw += r
        total_aug += a
    print(f"\nDone. {total_raw} raw clips -> {total_aug} augmented clips.")


if __name__ == "__main__":
    main()
