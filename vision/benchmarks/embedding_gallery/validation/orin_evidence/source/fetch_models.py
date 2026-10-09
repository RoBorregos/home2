#!/usr/bin/env python3
"""Fetch every vision weight up front so the stack works offline; --warmup also builds TRT engines.

Run inside the vision container (./run.sh vision --warmup); engines are device-specific, never copy them.
Also fetches the DINOv2 weights and syncs the embedding gallery beside every detectors/registry.py.
"""

import argparse
import hashlib
import json
import os
import shutil
import sys
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

sys.path.insert(
    0, str(REPO_ROOT / "vision" / "packages" / "object_detector_2d" / "scripts")
)
from embedding_gallery.core.constants import GALLERY_DIRNAME  # noqa: E402

# Standard ultralytics-hosted weights: name -> YOLO task (None = fetch only)
STANDARD_MODELS = {
    "yolo11m-pose.pt": "pose",  # hric_commands (wrists), tracker/gpsr/customer pose
    "yolov8n.pt": "detect",  # tracker, moondream person crop
    "yolo26n.pt": "detect",  # object_detector yolo_generic
    "yoloe-11l-seg.pt": None,  # zero_shot (loads via its own YOLOE path)
    "yoloe-11l-seg-pf.pt": None,  # embedding_box_proposer (prompt-free checkpoint)
}

URL_MODELS = {
    "face_landmarker.task": "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task",
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

# HF-hub models (the DINOv2 embedder). Multi-file artifacts, so they are checked
# by a successful load rather than a sha256.
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


def fetch_urls(dest: Path, manifest: dict) -> list[str]:
    failures = []
    for name, url in URL_MODELS.items():
        target = dest / name
        if target.exists():
            print(f"[fetch] ok       {target}")
        else:
            print(f"[fetch] getting  {name} ...")
            try:
                tmp = target.with_suffix(target.suffix + ".part")
                urllib.request.urlretrieve(url, tmp)
                tmp.rename(target)
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
    """Pre-download HF-hub models into hf_cache beside the TensorRT cache.

    The cache is a persistent mount, so the models work offline on competition day."""
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
    """Copy the built gallery (weights_dir()/gallery) beside every detectors/registry.py.

    Unlike the weights, it overwrites files that are newer at the source: objects
    change during setup day."""
    src = dest / GALLERY_DIRNAME
    if not src.is_dir():
        return
    for ddir in detector_dirs():
        target_root = ddir / GALLERY_DIRNAME
        for item in src.rglob("*"):
            if item.is_dir():
                continue
            rel = item.relative_to(src)
            target = target_root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists() or item.stat().st_mtime > target.stat().st_mtime:
                shutil.copy2(item, target)
                print(f"[sync]  {GALLERY_DIRNAME}/{rel} -> {ddir}")


def warmup(dest: Path):
    """Pre-build TRT engines + insightface cache for THIS device."""
    sys.path.insert(0, str(REPO_ROOT / "vision" / "packages" / "vision_general" / "scripts"))
    from utils.trt_utils import load_yolo_trt

    for name, task in STANDARD_MODELS.items():
        if task is None:
            continue
        print(f"[warmup] building engine for {name} (task={task}) ...")
        load_yolo_trt(str(dest / name), task=task)

    try:
        import numpy as np

        from detectors.registry import MODEL_CONFIGS
        from utils.models.image_embedder import ImageEmbedder

        # Use production's config (registry.py), not the class default: otherwise
        # this warms up PyTorch while production runs TensorRT, and the engine is
        # built on the node's first frame anyway.
        use_trt = MODEL_CONFIGS.get("embedding_gallery", {}).get("use_trt", True)
        for name in HF_MODELS:
            print(
                f"[warmup] forcing kernel compilation for {name} (use_trt={use_trt}) ..."
            )
            embedder = ImageEmbedder(name, use_trt=use_trt).load()
            from PIL import Image

            dummy = Image.fromarray(np.zeros((224, 224, 3), dtype=np.uint8))
            embedder.embed_batch([dummy])
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
    failures += fetch_urls(dest, manifest)
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
