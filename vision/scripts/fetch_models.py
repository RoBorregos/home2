#!/usr/bin/env python3
"""Fetch every vision model weight up front; optionally pre-build TRT engines.

Run INSIDE the vision container (needs ultralytics; insightface for --warmup):

    python3 /workspace/src/vision/scripts/fetch_models.py            # fetch only
    python3 /workspace/src/vision/scripts/fetch_models.py --warmup   # + TRT export
    ./run.sh vision --warmup                                         # from the host

Why: `.pt` weights are gitignored and download lazily from the internet on each
node's first run, followed by minutes of TensorRT export — on competition day,
with no internet, a fresh container simply breaks. This script makes the stack
offline-safe: standard weights land in TENSORRT_CACHE_DIR (a persistent mount
that `load_yolo_trt` already checks), detector weights land next to
`detectors/registry.py`, and --warmup pre-builds every TRT engine for THIS
device (engines are device- and TRT-version-specific — never copy them between
the laptop and the Orin). A MANIFEST.json with sha256 hashes is kept alongside
the weights for integrity checks.
"""

import argparse
import hashlib
import json
import os
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# Standard ultralytics-hosted weights: name -> YOLO task (None = fetch only)
STANDARD_MODELS = {
    "yolo11m-pose.pt": "pose",  # hric_commands (wrists), tracker/gpsr/customer pose
    "yolov8n.pt": "detect",  # tracker, moondream person crop
    "yolo26n.pt": "detect",  # object_detector yolo_generic
    "yoloe-11l-seg.pt": None,  # zero_shot (loads via its own YOLOE path)
}

# Custom weights that cannot be downloaded — verify presence, warn if missing.
CUSTOM_MODELS = [
    "robocup2026_v1.pt",
    "tmr2025.pt",
    "dishwasher_layout.pt",
    "dishwasher_rack.pt",
    "dishwasher_tablet.pt",
]

# Weights the object_detector registry expects beside detectors/registry.py.
DETECTOR_MODELS = ["yolo26n.pt", "yoloe-11l-seg.pt", "robocup2026_v1.pt"]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def weights_dir() -> Path:
    d = Path(os.environ.get("TENSORRT_CACHE_DIR", "/workspace/trt_cache"))
    d.mkdir(parents=True, exist_ok=True)
    return d


def detector_dirs() -> list[Path]:
    """Every detectors/ dir holding a registry.py (source + installed copies)."""
    roots = [REPO_ROOT, Path("/workspace/install"), Path("/workspace/src")]
    found = set()
    for root in roots:
        if root.is_dir():
            for reg in root.glob("**/detectors/registry.py"):
                if "build" not in reg.parts:
                    found.add(reg.parent)
    return sorted(found)


def fetch_standard(dest: Path, manifest: dict) -> list[str]:
    from ultralytics.utils.downloads import attempt_download_asset

    failures = []
    for name in STANDARD_MODELS:
        target = dest / name
        if target.exists():
            print(f"[fetch] ok       {target}")
        else:
            print(f"[fetch] getting  {name} ...")
            try:
                got = Path(attempt_download_asset(str(target)))
                if got != target and got.exists():
                    shutil.move(str(got), target)
            except Exception as e:
                print(f"[fetch] FAILED   {name}: {e}")
                failures.append(name)
                continue
        digest = sha256(target)
        known = manifest.get(name)
        if known and known != digest:
            print(
                f"[fetch] WARNING  {name} sha256 changed ({digest[:12]} != {known[:12]})"
            )
        manifest[name] = digest
    return failures


def check_customs(manifest: dict) -> list[str]:
    missing = []
    search = [weights_dir(), *detector_dirs(), REPO_ROOT / "vision"]
    for name in CUSTOM_MODELS:
        hits = [d / name for d in search if (d / name).exists()]
        hits += list((REPO_ROOT / "vision").glob(f"**/{name}"))
        hits = [h for h in hits if "build" not in h.parts]
        if hits:
            print(f"[custom] ok      {hits[0]}")
            manifest[name] = sha256(hits[0])
        else:
            print(f"[custom] MISSING {name} (custom weight — copy it in manually)")
            missing.append(name)
    return missing


def sync_detector_models(dest: Path):
    """Copy detector weights beside every detectors/registry.py found."""
    for ddir in detector_dirs():
        for name in DETECTOR_MODELS:
            src = dest / name
            target = ddir / name
            if src.exists() and not target.exists():
                shutil.copy2(src, target)
                print(f"[sync]  {name} -> {ddir}")


def warmup(dest: Path):
    """Pre-build TRT engines + insightface cache for THIS device."""
    sys.path.insert(0, str(REPO_ROOT / "vision" / "packages" / "vision_general"))
    from vision_general.utils.trt_utils import load_yolo_trt

    for name, task in STANDARD_MODELS.items():
        if task is None:
            continue
        print(f"[warmup] building engine for {name} (task={task}) ...")
        load_yolo_trt(str(dest / name), task=task)

    try:
        from insightface.app import FaceAnalysis

        print("[warmup] preparing insightface buffalo_sc ...")
        app = FaceAnalysis(name="buffalo_sc")
        app.prepare(ctx_id=0, det_size=(640, 640))
    except Exception as e:
        print(f"[warmup] insightface skipped: {e}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--warmup", action="store_true", help="also pre-build TRT engines"
    )
    args = parser.parse_args()

    dest = weights_dir()
    manifest_path = dest / "MANIFEST.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}

    failures = fetch_standard(dest, manifest)
    missing = check_customs(manifest)
    sync_detector_models(dest)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"[fetch] manifest -> {manifest_path}")

    if args.warmup:
        warmup(dest)

    if failures or missing:
        print(f"\nIncomplete: failed={failures} missing_custom={missing}")
        sys.exit(1)
    print("\nAll models present." + (" Engines warm." if args.warmup else ""))


if __name__ == "__main__":
    main()
