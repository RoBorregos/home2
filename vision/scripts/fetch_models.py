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

Also fetches the embedding-gallery detector's dependencies: the DINOv2
backbone (HF_MODELS, cached under TENSORRT_CACHE_DIR/hf_cache since it isn't
a single ultralytics asset) and the few-shot object gallery itself
(sync_gallery() — build it once with gallery_build.py into
TENSORRT_CACHE_DIR/gallery, this script copies it beside every
detectors/registry.py found, same as the .pt weights).
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
    "yoloe-11l-seg-pf.pt": None,  # embedding_box_proposer (prompt-free checkpoint)
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
DETECTOR_MODELS = [
    "yolo26n.pt",
    "yoloe-11l-seg.pt",
    "yoloe-11l-seg-pf.pt",
    "robocup2026_v1.pt",
]

# HF-hub-hosted models (not a single ultralytics asset) — e.g. the DINOv2
# backbone for the embedding-gallery detector. Verified by presence (a
# successful load), not a single-file sha256: HF Hub artifacts are
# multi-file (config.json, model.safetensors, ...).
HF_MODELS = ["vit_base_patch14_dinov2.lvd142m"]


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


def fetch_hf_models(dest: Path) -> list[str]:
    """Pre-download timm/HF-hub-hosted models into a persistent cache beside
    the TensorRT cache, so they survive container rebuilds and don't need
    internet on competition day (same offline-safety goal as STANDARD_MODELS,
    different download mechanism)."""
    hf_cache = dest / "hf_cache"
    hf_cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("HF_HOME", str(hf_cache))

    try:
        import timm
    except ImportError:
        print("[fetch] timm not installed, skipping HF models:", HF_MODELS)
        return list(HF_MODELS)

    failures = []
    for name in HF_MODELS:
        try:
            print(f"[fetch] getting  {name} (HF hub) ...")
            timm.create_model(name, pretrained=True, num_classes=0)
            print(f"[fetch] ok       {name}")
        except Exception as e:
            print(f"[fetch] FAILED   {name}: {e}")
            failures.append(name)
    return failures


def sync_gallery(dest: Path):
    """Copy the embedding gallery (built by gallery_build.py into
    weights_dir()/gallery) beside every detectors/registry.py found.
    Unlike sync_detector_models()'s copy-if-missing, this overwrites files
    that are newer at the source: the gallery is expected to change during
    setup day as objects get re-shot or added, unlike the static competition
    YOLO weights."""
    src = dest / "gallery"
    if not src.is_dir():
        return
    for ddir in detector_dirs():
        target_root = ddir / "gallery"
        for item in src.rglob("*"):
            if item.is_dir():
                continue
            rel = item.relative_to(src)
            target = target_root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists() or item.stat().st_mtime > target.stat().st_mtime:
                shutil.copy2(item, target)
                print(f"[sync]  gallery/{rel} -> {ddir}")


def warmup(dest: Path):
    """Pre-build TRT engines + insightface cache for THIS device."""
    sys.path.insert(0, str(REPO_ROOT / "vision" / "packages" / "vision_general"))
    from utils.trt_utils import load_yolo_trt

    for name, task in STANDARD_MODELS.items():
        if task is None:
            continue
        print(f"[warmup] building engine for {name} (task={task}) ...")
        load_yolo_trt(str(dest / name), task=task)

    try:
        import numpy as np

        sys.path.insert(
            0,
            str(
                REPO_ROOT
                / "vision"
                / "packages"
                / "object_detector_2d"
                / "scripts"
                / "detectors"
            ),
        )
        from backbone import EmbeddingBackbone

        for name in HF_MODELS:
            print(f"[warmup] forcing kernel compilation for {name} ...")
            backbone = EmbeddingBackbone(name).load()
            from PIL import Image

            dummy = Image.fromarray(np.zeros((224, 224, 3), dtype=np.uint8))
            backbone.embed_batch([dummy])
        print("[warmup] embedding backbone(s) ready")
    except Exception as e:
        print(f"[warmup] embedding backbone warmup skipped: {e}")

    try:
        import numpy as np
        from insightface.app import FaceAnalysis

        print("[warmup] preparing insightface buffalo_sc ...")
        app = FaceAnalysis(name="buffalo_sc")
        app.prepare(ctx_id=0, det_size=(640, 640))
        # prepare() only creates the sessions — the onnxruntime-TRT engines
        # compile on FIRST INFERENCE (~8 min stall on the Orin, during which
        # face_recognition_node never becomes Ready). Force the builds now.
        print("[warmup] building face TRT engines (first time can take ~10 min) ...")
        app.get(np.zeros((640, 640, 3), dtype=np.uint8))
        rec = app.models.get("recognition")
        if rec is not None:
            rec.get_feat(np.zeros((112, 112, 3), dtype=np.uint8))
        print("[warmup] insightface engines ready")
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
    hf_failures = fetch_hf_models(dest)
    missing = check_customs(manifest)
    sync_detector_models(dest)
    sync_gallery(dest)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"[fetch] manifest -> {manifest_path}")

    if args.warmup:
        warmup(dest)

    failures = failures + hf_failures
    if failures or missing:
        print(f"\nIncomplete: failed={failures} missing_custom={missing}")
        sys.exit(1)
    print("\nAll models present." + (" Engines warm." if args.warmup else ""))


if __name__ == "__main__":
    main()
