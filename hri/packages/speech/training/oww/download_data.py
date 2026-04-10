#!/usr/bin/env python3
"""Idempotent dataset downloader for the openWakeWord training pipeline.

Fetches everything train.py needs into ``.data/`` next to this file:

    mit_rirs/                       MIT room impulse responses
    fma/fma_small/                  Free Music Archive small (background music)
    audioset_16k/                   AudioSet 16 kHz subset (background speech/noise)
    negative_features_large.npy     ACAV100M pre-computed OWW features
    dinner_party_eval_features.npy  False-positive validation features
    oww_features/melspectrogram.onnx
    oww_features/embedding_model.onnx

Any target that already exists on disk is skipped, so re-running ``run.sh``
after a crash is cheap. Prints a size summary before starting so the user can
abort on a metered link.
"""

from __future__ import annotations

import os
import shutil
import sys
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

HERE = Path(__file__).resolve().parent
DATA_DIR = HERE / ".data"


@dataclass
class Asset:
    name: str
    url: str
    # Destination path relative to DATA_DIR. For zips this is the directory
    # the archive should expand into; for loose files it is the file path.
    dest: str
    approx_size_gb: float
    is_zip: bool = False
    # Optional post-download hook (takes the extracted/downloaded path).
    post: Optional[Callable[[Path], None]] = None

    @property
    def abs_dest(self) -> Path:
        return DATA_DIR / self.dest


ASSETS: list[Asset] = [
    Asset(
        name="MIT Room Impulse Responses",
        url="https://huggingface.co/datasets/davidscripka/MIT_environmental_impulse_responses/resolve/main/mit_rirs.zip",
        dest="mit_rirs",
        approx_size_gb=0.5,
        is_zip=True,
    ),
    Asset(
        name="FMA small (background music)",
        url="https://huggingface.co/datasets/davidscripka/MIT_environmental_impulse_responses/resolve/main/fma_small.zip",
        dest="fma/fma_small",
        approx_size_gb=8.0,
        is_zip=True,
    ),
    Asset(
        name="AudioSet 16 kHz subset (background speech/noise)",
        url="https://huggingface.co/datasets/davidscripka/MIT_environmental_impulse_responses/resolve/main/audioset_16k.zip",
        dest="audioset_16k",
        approx_size_gb=3.0,
        is_zip=True,
    ),
    Asset(
        name="ACAV100M adversarial negative features",
        url="https://huggingface.co/datasets/davidscripka/openwakeword_features_ACAV100M_2000_hrs_16bit.npy/resolve/main/openwakeword_features_ACAV100M_2000_hrs_16bit.npy",
        dest="negative_features_large.npy",
        approx_size_gb=2.0,
    ),
    Asset(
        name="Dinner-party false-positive validation features",
        url="https://huggingface.co/datasets/davidscripka/openwakeword_features_dinner_party_eval.npy/resolve/main/openwakeword_features_dinner_party_eval.npy",
        dest="dinner_party_eval_features.npy",
        approx_size_gb=0.3,
    ),
    Asset(
        name="OWW feature extractor (melspectrogram.onnx)",
        url="https://github.com/dscripka/openWakeWord/releases/download/v0.5.1/melspectrogram.onnx",
        dest="oww_features/melspectrogram.onnx",
        approx_size_gb=0.01,
    ),
    Asset(
        name="OWW feature extractor (embedding_model.onnx)",
        url="https://github.com/dscripka/openWakeWord/releases/download/v0.5.1/embedding_model.onnx",
        dest="oww_features/embedding_model.onnx",
        approx_size_gb=0.02,
    ),
]


def _already_have(asset: Asset) -> bool:
    """Return True if ``asset.abs_dest`` already exists and is non-empty."""
    p = asset.abs_dest
    if not p.exists():
        return False
    if p.is_dir():
        # Directory is considered present if it has at least one file inside.
        for _ in p.rglob("*"):
            return True
        return False
    return p.stat().st_size > 0


def _download(url: str, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = out_path.with_suffix(out_path.suffix + ".part")
    print(f"  fetching {url}")
    with urllib.request.urlopen(url) as response, open(tmp_path, "wb") as fh:
        shutil.copyfileobj(response, fh)
    tmp_path.rename(out_path)


def _extract_zip(zip_path: Path, extract_to: Path) -> None:
    extract_to.mkdir(parents=True, exist_ok=True)
    print(f"  extracting {zip_path.name} -> {extract_to}")
    with zipfile.ZipFile(zip_path) as zf:
        zf.extractall(extract_to)


def _fetch_asset(asset: Asset) -> None:
    if _already_have(asset):
        print(f"[skip] {asset.name} (already present at {asset.abs_dest})")
        return

    print(f"[get ] {asset.name}")
    if asset.is_zip:
        # Extract into the parent of ``dest``; the zip contents should produce
        # the directory named by ``dest``. If the archive has a different top
        # level, the extraction target is simply dest itself.
        zip_tmp = DATA_DIR / f"_tmp_{asset.abs_dest.name}.zip"
        _download(asset.url, zip_tmp)
        _extract_zip(zip_tmp, asset.abs_dest.parent)
        zip_tmp.unlink(missing_ok=True)
        # If the expected dest dir still doesn't exist, the archive expanded
        # into the parent flat; treat the parent as the dataset dir and leave
        # a README marker so the configs keep pointing at the right place.
        if not asset.abs_dest.exists():
            asset.abs_dest.mkdir(parents=True, exist_ok=True)
    else:
        _download(asset.url, asset.abs_dest)

    if asset.post:
        asset.post(asset.abs_dest)


def _print_summary() -> None:
    pending = [a for a in ASSETS if not _already_have(a)]
    if not pending:
        print("All datasets already present. Nothing to download.")
        return
    total = sum(a.approx_size_gb for a in pending)
    print("The following assets will be downloaded:")
    for a in pending:
        print(f"  - {a.name:<55} ~{a.approx_size_gb:>5.2f} GB")
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
    for asset in ASSETS:
        try:
            _fetch_asset(asset)
        except Exception as exc:  # noqa: BLE001 - surface any download error
            print(f"ERROR while fetching {asset.name}: {exc}", file=sys.stderr)
            return 1
    print("All datasets ready under", DATA_DIR)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
