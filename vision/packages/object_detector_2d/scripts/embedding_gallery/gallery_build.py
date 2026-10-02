#!/usr/bin/env python3
"""Build or update one gallery entry (name -> N embeddings) from a folder of
photos, cropped by the same box proposer + oversized-box filter the runtime detector uses (--no-crop to skip). Every object in one gallery must share the embedding dimension, or Gallery.load() crashes at the next node restart — --backbone defaults to registry.py's MODEL_CONFIGS, don't override it."""

import argparse
import glob
import json
from pathlib import Path

import numpy as np
from detectors.registry import MODEL_CONFIGS, ModelRegistry
from embedding_gallery.backbone import EmbeddingBackbone
from embedding_gallery.gallery_matcher import (
    DEFAULT_MARGIN_MIN,
    DEFAULT_MAX_BOX_AREA_FRAC,
    DEFAULT_MIN_SIMILARITY,
    l2_normalize,
)

RECOMMENDED_MIN_PHOTOS = 10
RECOMMENDED_MAX_PHOTOS = 30

# Single source of truth for "what backbone does production use" — keeps
# this default in sync with MODEL_CONFIGS["embedding_gallery"]["backbone"].
DEFAULT_BACKBONE = MODEL_CONFIGS["embedding_gallery"]["backbone"]


def load_box_proposer() -> tuple:
    """Return (proposer, max_box_area_frac) as configured for production.

    Importing `detectors.registry` runs detectors/__init__.py, which registers
    the model types (yolo, yolo_e, embedding)."""
    config = MODEL_CONFIGS["embedding_gallery"]
    proposer = ModelRegistry.get(config["box_model"])
    return proposer, config.get("max_box_area_frac", DEFAULT_MAX_BOX_AREA_FRAC)


def pick_main_box(detections: list, w: int, h: int, max_area_frac: float):
    """Pixel box (x1, y1, x2, y2) of the most likely main object, or None —
    drops oversized boxes (same rule as the runtime detector), then prefers high-confidence boxes near the image centre."""
    best, best_score = None, 0.0
    for det in detections:
        x1 = max(0, int(det.bbox_.x1 * w))
        y1 = max(0, int(det.bbox_.y1 * h))
        x2 = min(w, int(det.bbox_.x2 * w))
        y2 = min(h, int(det.bbox_.y2 * h))
        if x2 <= x1 or y2 <= y1:
            continue
        if (x2 - x1) * (y2 - y1) > max_area_frac * w * h:
            continue
        cx, cy = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
        score = det.confidence_ * max(0.0, 1.0 - abs(cx - 0.5) - abs(cy - 0.5))
        if score > best_score:
            best, best_score = (x1, y1, x2, y2), score
    return best


def crop_photos(paths: list[str]) -> tuple[list, list[str]]:
    """Cut each photo to its main object. Returns (crops, kept_paths);
    photos with no usable box are skipped with a warning."""
    from PIL import Image

    proposer, max_area_frac = load_box_proposer()
    crops_dir = Path(paths[0]).parent / "_crops"
    crops_dir.mkdir(exist_ok=True)

    crops, kept = [], []
    for path in paths:
        img = Image.open(path).convert("RGB")
        w, h = img.size
        box = pick_main_box(
            proposer.detect(np.asarray(img)[:, :, ::-1]), w, h, max_area_frac
        )
        if box is None:
            print(f"[gallery_build] WARNING: no usable box in {path}, skipped")
            continue
        crop = img.crop(box)
        crop.save(crops_dir / f"{Path(path).stem}.jpg")
        crops.append(crop)
        kept.append(path)
    print(
        f"[gallery_build] cropped {len(kept)}/{len(paths)} photos "
        f"(review them in {crops_dir})"
    )
    return crops, kept


def build_gallery(
    object_name: str,
    photo_glob: str,
    backbone_id: str,
    gallery_dir: Path,
    crop: bool = True,
) -> int:
    from PIL import Image

    paths = sorted(glob.glob(photo_glob))
    if not paths:
        raise SystemExit(f"No photos matched {photo_glob!r}")

    if crop:
        crops, paths = crop_photos(paths)
        if not crops:
            raise SystemExit(
                f"[gallery_build] no photo of {object_name!r} produced a usable box; "
                f"retake them with the object front and centre, or pass --no-crop "
                f"if they are already tight crops"
            )
    else:
        crops = [Image.open(p).convert("RGB") for p in paths]

    if not (RECOMMENDED_MIN_PHOTOS <= len(paths) <= RECOMMENDED_MAX_PHOTOS):
        print(
            f"[gallery_build] WARNING: {len(paths)} usable photos for {object_name!r} "
            f"(recommended {RECOMMENDED_MIN_PHOTOS}-{RECOMMENDED_MAX_PHOTOS})"
        )

    gallery_dir.mkdir(parents=True, exist_ok=True)
    backbone = EmbeddingBackbone(backbone_id).load()
    vectors = l2_normalize(backbone.embed_batch(crops))

    # Check dimension against another object BEFORE writing anything — a
    # wrong --backbone used to write a bad .npy silently and crash later.
    manifest_path = gallery_dir / "manifest.json"
    existing_manifest = (
        json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    )
    for other_name, other_cfg in (existing_manifest.get("objects") or {}).items():
        if other_name == object_name:
            continue
        other_npy = gallery_dir / other_cfg["npy"]
        if not other_npy.exists():
            continue
        other_dim = np.load(other_npy, mmap_mode="r").shape[-1]
        if other_dim != vectors.shape[-1]:
            raise SystemExit(
                f"[gallery_build] REFUSING: {object_name!r} embedded at {vectors.shape[-1]} dims "
                f"(backbone={backbone_id!r}), but {other_name!r} in this gallery is {other_dim} dims "
                f"(backbone={other_cfg.get('backbone')!r}). Every object in one gallery/ must use the "
                f"SAME --backbone, or the whole gallery crashes at the next node restart, not now."
            )

    npy_name = f"{object_name}.npy"
    np.save(gallery_dir / npy_name, vectors)

    manifest = existing_manifest
    manifest.setdefault("objects", {})
    existing = manifest["objects"].get(object_name, {})
    manifest["objects"][object_name] = {
        "npy": npy_name,
        "num_photos": len(paths),
        "backbone": backbone_id,
        # Preserve hand-tuned thresholds across rebuilds; seed from Phase 1's
        # calibrated defaults (or the module defaults) only the first time.
        "min_similarity": existing.get("min_similarity", DEFAULT_MIN_SIMILARITY),
        "margin_min": existing.get("margin_min", DEFAULT_MARGIN_MIN),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(
        f"[gallery_build] {object_name}: {len(paths)} photos -> {gallery_dir / npy_name}"
    )
    return len(paths)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--object", required=True, help="canonical object name (gallery key)"
    )
    parser.add_argument(
        "--photos", required=True, help='glob pattern, e.g. "gallery_photos/coke/*.jpg"'
    )
    parser.add_argument(
        "--backbone",
        default=DEFAULT_BACKBONE,
        help="timm model id, or clip:<name> (e.g. clip:ViT-B/32) — defaults to "
        "production's embedding_gallery backbone; see module docstring before overriding",
    )
    parser.add_argument("--gallery-dir", default="gallery")
    parser.add_argument(
        "--no-crop",
        action="store_true",
        help="embed photos as-is (use only if they are already tight object crops)",
    )
    args = parser.parse_args()
    build_gallery(
        args.object,
        args.photos,
        args.backbone,
        Path(args.gallery_dir),
        crop=not args.no_crop,
    )


if __name__ == "__main__":
    main()
