#!/usr/bin/env python3
"""Idempotent dataset downloader for the openWakeWord training pipeline.

Mirrors the recipe in openWakeWord's ``automatic_model_training.ipynb``.
Fetches everything train.py needs into ``.data/`` next to this file:

    mit_rirs/                       MIT room impulse responses (WAV, 16 kHz)
    audioset_16k/                   AudioSet bal_train09 shard (WAV, 16 kHz)
    negative_features_large.npy     ACAV100M pre-computed OWW features
    validation_set_features.npy     False-positive validation features
    oww_features/                   OWW feature extractor ONNX files

Any target that already exists on disk is skipped, so re-running ``run.sh``
after a crash is cheap. Prints a size summary before starting so the user
can abort on a metered link.
"""

from __future__ import annotations

import os
import shutil
import sys
import tarfile
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA_DIR = HERE / ".data"

# Approximate disk footprint for the summary (printed before anything runs).
APPROX_SIZES_GB = {
    "mit_rirs": 0.05,
    "audioset_16k": 3.0,
    "negative_features_large.npy": 2.0,
    "validation_set_features.npy": 0.3,
    "oww_features": 0.05,
}


def _nonempty_dir(p: Path) -> bool:
    if not p.is_dir():
        return False
    for _ in p.iterdir():
        return True
    return False


def _download_file(url: str, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = out_path.with_suffix(out_path.suffix + ".part")
    print(f"  fetching {url}")
    with urllib.request.urlopen(url) as response, open(tmp_path, "wb") as fh:
        shutil.copyfileobj(response, fh)
    tmp_path.rename(out_path)


# --- Individual fetchers -----------------------------------------------------


def fetch_mit_rirs() -> None:
    """Stream the MIT RIR dataset and write 16 kHz WAVs to .data/mit_rirs/."""
    out_dir = DATA_DIR / "mit_rirs"
    if _nonempty_dir(out_dir):
        print(f"[skip] mit_rirs (already present at {out_dir})")
        return

    print("[get ] MIT Room Impulse Responses")
    import datasets
    import numpy as np
    import scipy.io.wavfile
    from tqdm import tqdm

    out_dir.mkdir(parents=True, exist_ok=True)
    rir_dataset = datasets.load_dataset(
        "davidscripka/MIT_environmental_impulse_responses",
        split="train",
        streaming=True,
    )
    for row in tqdm(rir_dataset, desc="mit_rirs"):
        name = row["audio"]["path"].split("/")[-1]
        if not name.lower().endswith(".wav"):
            name = os.path.splitext(name)[0] + ".wav"
        scipy.io.wavfile.write(
            out_dir / name,
            16000,
            (row["audio"]["array"] * 32767).astype(np.int16),
        )


def fetch_audioset() -> None:
    """Download one AudioSet shard and rewrite its FLACs as 16 kHz WAVs."""
    out_dir = DATA_DIR / "audioset_16k"
    if _nonempty_dir(out_dir):
        print(f"[skip] audioset_16k (already present at {out_dir})")
        return

    print("[get ] AudioSet (bal_train09 shard)")
    import datasets
    import numpy as np
    import scipy.io.wavfile
    from tqdm import tqdm

    tar_dir = DATA_DIR / "audioset"
    tar_dir.mkdir(parents=True, exist_ok=True)
    tar_path = tar_dir / "bal_train09.tar"
    url = (
        "https://huggingface.co/datasets/agkphysics/AudioSet/resolve/main/data/"
        "bal_train09.tar"
    )
    if not tar_path.exists():
        _download_file(url, tar_path)

    print(f"  extracting {tar_path.name}")
    with tarfile.open(tar_path) as tf:
        tf.extractall(tar_dir)

    out_dir.mkdir(parents=True, exist_ok=True)
    flac_paths = [str(p) for p in (tar_dir / "audio").glob("**/*.flac")]
    if not flac_paths:
        print(f"  WARNING: no FLAC files found under {tar_dir / 'audio'}")
        return
    audioset_dataset = datasets.Dataset.from_dict({"audio": flac_paths})
    audioset_dataset = audioset_dataset.cast_column(
        "audio", datasets.Audio(sampling_rate=16000)
    )
    for row in tqdm(audioset_dataset, desc="audioset_16k"):
        name = row["audio"]["path"].split("/")[-1].replace(".flac", ".wav")
        scipy.io.wavfile.write(
            out_dir / name,
            16000,
            (row["audio"]["array"] * 32767).astype(np.int16),
        )


def fetch_acav100m_features() -> None:
    out_path = DATA_DIR / "negative_features_large.npy"
    if out_path.exists() and out_path.stat().st_size > 0:
        print(f"[skip] ACAV100M features (already present at {out_path})")
        return
    print("[get ] ACAV100M adversarial negative features")
    _download_file(
        "https://huggingface.co/datasets/davidscripka/openwakeword_features/"
        "resolve/main/openwakeword_features_ACAV100M_2000_hrs_16bit.npy",
        out_path,
    )


def fetch_validation_features() -> None:
    out_path = DATA_DIR / "validation_set_features.npy"
    if out_path.exists() and out_path.stat().st_size > 0:
        print(f"[skip] validation features (already present at {out_path})")
        return
    print("[get ] False-positive validation features")
    _download_file(
        "https://huggingface.co/datasets/davidscripka/openwakeword_features/"
        "resolve/main/validation_set_features.npy",
        out_path,
    )


def fetch_oww_feature_extractors() -> None:
    """Ensure openwakeword's melspectrogram/embedding ONNX files are cached.

    openwakeword ships a helper that downloads these on first use; calling it
    here keeps training runs self-contained and avoids a surprise download
    the first time train.py imports the package.
    """
    marker = DATA_DIR / "oww_features" / ".downloaded"
    if marker.exists():
        print("[skip] openwakeword feature extractors (already cached)")
        return
    print("[get ] openwakeword feature extractors (mel + embedding)")
    try:
        import openwakeword.utils as utils

        utils.download_models()
    except Exception as exc:  # noqa: BLE001
        print(f"  WARNING: openwakeword.download_models() failed: {exc}")
        print("  training will attempt to fetch them itself on first run.")
        return
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.touch()


# --- Driver ------------------------------------------------------------------


FETCHERS = [
    ("mit_rirs", fetch_mit_rirs),
    ("audioset_16k", fetch_audioset),
    ("negative_features_large.npy", fetch_acav100m_features),
    ("validation_set_features.npy", fetch_validation_features),
    ("oww_features", fetch_oww_feature_extractors),
]


def _pending() -> list[str]:
    pending = []
    for name, _ in FETCHERS:
        p = DATA_DIR / name
        if name.endswith(".npy"):
            if not (p.exists() and p.stat().st_size > 0):
                pending.append(name)
        elif name == "oww_features":
            if not (DATA_DIR / "oww_features" / ".downloaded").exists():
                pending.append(name)
        else:
            if not _nonempty_dir(p):
                pending.append(name)
    return pending


def _print_summary() -> None:
    pending = _pending()
    if not pending:
        print("All datasets already present. Nothing to download.")
        return
    total = sum(APPROX_SIZES_GB.get(n, 0.0) for n in pending)
    print("The following assets will be downloaded:")
    for n in pending:
        print(f"  - {n:<40} ~{APPROX_SIZES_GB.get(n, 0.0):>5.2f} GB")
    print(f"Estimated total: ~{total:.1f} GB")
    print(f"Target directory: {DATA_DIR}")
    if os.environ.get("OWW_DOWNLOAD_YES") != "1":
        reply = input("Proceed? [y/N] ").strip().lower()
        if reply not in ("y", "yes"):
            print("Aborted.")
            sys.exit(1)


def main() -> int:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    _print_summary()
    for name, fn in FETCHERS:
        try:
            fn()
        except Exception as exc:  # noqa: BLE001 - surface any download error
            print(f"ERROR while fetching {name}: {exc}", file=sys.stderr)
            return 1
    print("All datasets ready under", DATA_DIR)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
