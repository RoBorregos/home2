#!/usr/bin/env python3
"""Build gallery entries from photos: one folder per object under gallery_photos/.

Each photo is cropped to its main object with the production box proposer, the
crops are embedded with the production backbone and saved as <object>.npy, and
manifest.json is updated. Run it through add_object.sh.

    gallery_build.py <object> [<object> ...] [--no-crop]
    gallery_build.py --all [--no-crop]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from detectors.registry import MODEL_CONFIGS, ModelRegistry
from PIL import Image

from embedding_gallery.backbone import EmbeddingBackbone
from embedding_gallery.constants import (
    CROPS_DIRNAME,
    GALLERY_DIRNAME,
    MANIFEST_NAME,
    PHOTO_EXTENSIONS,
    PHOTOS_DIRNAME,
    tensorrt_cache_dir,
)
from embedding_gallery.gallery_matcher import (
    DEFAULT_MARGIN_MIN,
    DEFAULT_MAX_BOX_AREA_FRAC,
    DEFAULT_MIN_SIMILARITY,
    l2_normalize,
)

RECOMMENDED_MIN_PHOTOS = 10
RECOMMENDED_MAX_PHOTOS = 30

PHOTOS_DIR = Path(__file__).resolve().parent / PHOTOS_DIRNAME

# Every object in one gallery must share the embedding dimension, so the
# backbone is always production's (registry.py), never a CLI option.
BACKBONE_ID = MODEL_CONFIGS["embedding_gallery"]["backbone"]


class BuildError(Exception):
    """One object could not be enrolled; the message says why."""


def gallery_dir() -> Path:
    """Persistent gallery location; fetch_models.py copies it beside every detectors/registry.py."""
    return tensorrt_cache_dir() / GALLERY_DIRNAME


def find_photos(object_name: str) -> list[Path]:
    folder = PHOTOS_DIR / object_name
    if not folder.is_dir():
        return []
    return sorted(
        p
        for p in folder.iterdir()
        if p.is_file() and p.suffix.lower() in PHOTO_EXTENSIONS
    )


def list_objects() -> list[str]:
    if not PHOTOS_DIR.is_dir():
        return []
    return sorted(
        d.name
        for d in PHOTOS_DIR.iterdir()
        if d.is_dir() and not d.name.startswith(("_", "."))
    )


def load_box_proposer() -> tuple:
    """(proposer, max_box_area_frac) as configured for production."""
    config = MODEL_CONFIGS["embedding_gallery"]
    proposer = ModelRegistry.get(config["box_model"])
    return proposer, config.get("max_box_area_frac", DEFAULT_MAX_BOX_AREA_FRAC)


def pick_main_box(detections: list, w: int, h: int, max_area_frac: float):
    """Pixel box (x1, y1, x2, y2) of the most likely main object, or None.

    Drops oversized boxes (same rule as the runtime detector), then prefers
    high-confidence boxes near the image centre."""
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


def crop_photos(
    paths: list[Path], proposer, max_area_frac: float
) -> tuple[list, list[Path]]:
    """Cut each photo to its main object. Returns (crops, kept_paths).

    Photos with no usable box are skipped with a warning. The crops are also
    saved next to the photos (CROPS_DIRNAME) so they can be reviewed."""
    crops_dir = paths[0].parent / CROPS_DIRNAME
    crops_dir.mkdir(exist_ok=True)
    for stale in crops_dir.glob("*.jpg"):
        stale.unlink()

    crops, kept = [], []
    for path in paths:
        img = Image.open(path).convert("RGB")
        w, h = img.size
        box = pick_main_box(
            proposer.detect(np.asarray(img)[:, :, ::-1]), w, h, max_area_frac
        )
        if box is None:
            print(f"[gallery_build] WARNING: no usable box in {path.name}, skipped")
            continue
        crop = img.crop(box)
        crop.save(crops_dir / f"{path.stem}.jpg")
        crops.append(crop)
        kept.append(path)
    print(
        f"[gallery_build] cropped {len(kept)}/{len(paths)} photos "
        f"(review them in {crops_dir})"
    )
    return crops, kept


def build_object(
    object_name: str,
    backbone: EmbeddingBackbone,
    proposer,
    max_area_frac: float,
    out_dir: Path,
    crop: bool = True,
) -> int:
    """Embed one object's photos and write its entry into out_dir. Returns the photo count."""
    paths = find_photos(object_name)
    if not paths:
        raise BuildError(
            f"no {'/'.join(PHOTO_EXTENSIONS)} photos in {PHOTOS_DIR / object_name}"
        )
    if not (RECOMMENDED_MIN_PHOTOS <= len(paths) <= RECOMMENDED_MAX_PHOTOS):
        print(
            f"[gallery_build] WARNING: {len(paths)} photos for {object_name!r} "
            f"(recommended {RECOMMENDED_MIN_PHOTOS}-{RECOMMENDED_MAX_PHOTOS})"
        )

    if crop:
        crops, paths = crop_photos(paths, proposer, max_area_frac)
        if not crops:
            raise BuildError(
                f"no usable box in any photo of {object_name!r}; retake them with "
                "the object centered, or use --no-crop for tight crops"
            )
    else:
        crops = [Image.open(p).convert("RGB") for p in paths]

    vectors = l2_normalize(backbone.embed_batch(crops))

    # Check the dimension against the other objects BEFORE writing anything.
    manifest_path = out_dir / MANIFEST_NAME
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    for other_name, other_cfg in (manifest.get("objects") or {}).items():
        other_npy = out_dir / other_cfg["npy"]
        if other_name == object_name or not other_npy.exists():
            continue
        other_dim = np.load(other_npy, mmap_mode="r").shape[-1]
        if other_dim != vectors.shape[-1]:
            raise BuildError(
                f"{object_name!r} has {vectors.shape[-1]} dims but {other_name!r} "
                f"has {other_dim} (backbone={other_cfg.get('backbone')!r}); every "
                f"object in {GALLERY_DIRNAME}/ must share one backbone"
            )

    out_dir.mkdir(parents=True, exist_ok=True)
    npy_name = f"{object_name}.npy"
    np.save(out_dir / npy_name, vectors)

    objects = manifest.setdefault("objects", {})
    previous = objects.get(object_name, {})
    objects[object_name] = {
        "npy": npy_name,
        "num_photos": len(paths),
        "backbone": BACKBONE_ID,
        # Keep hand-tuned thresholds across rebuilds; seed new objects with the
        # calibrated defaults.
        "min_similarity": previous.get("min_similarity", DEFAULT_MIN_SIMILARITY),
        "margin_min": previous.get("margin_min", DEFAULT_MARGIN_MIN),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"[gallery_build] {object_name}: {len(paths)} photos -> {out_dir / npy_name}")
    return len(paths)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "objects",
        nargs="*",
        help=f"object names (folders under {PHOTOS_DIRNAME}/)",
    )
    parser.add_argument(
        "--all", action="store_true", help=f"every folder under {PHOTOS_DIRNAME}/"
    )
    parser.add_argument(
        "--no-crop",
        action="store_true",
        help="embed photos as they are (only if they are already tight object crops)",
    )
    args = parser.parse_args()
    if args.all == bool(args.objects):
        parser.error("pass object names, or --all (not both, not neither)")

    names = list_objects() if args.all else args.objects
    if not names:
        print(f"[gallery_build] no object folders in {PHOTOS_DIR}")
        return 1

    crop = not args.no_crop
    proposer, max_area_frac = load_box_proposer() if crop else (None, None)
    backbone = EmbeddingBackbone(BACKBONE_ID).load()
    out_dir = gallery_dir()

    failures = {}
    for name in names:
        try:
            build_object(name, backbone, proposer, max_area_frac, out_dir, crop)
        except BuildError as e:
            print(f"[gallery_build] FAILED {name}: {e}")
            failures[name] = str(e)

    if failures:
        print(
            f"[gallery_build] {len(failures)}/{len(names)} failed: {sorted(failures)}"
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
