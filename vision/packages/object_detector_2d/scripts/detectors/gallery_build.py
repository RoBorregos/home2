#!/usr/bin/env python3
"""Build or update one gallery entry (name -> N embeddings) from a folder of photos.

Usage (inside the vision container — needs timm/torch, and clip if the
backbone id starts with "clip:"):

    python3 gallery_build.py --object coke \\
        --photos "gallery_photos/coke/*.jpg" \\
        --backbone vit_small_patch14_dinov2.lvd142m \\
        --gallery-dir gallery/

Writes gallery/<object>.npy (float32 [N, D], L2-normalized) and updates a
single gallery/manifest.json across all objects (mirrors the MANIFEST.json
convention fetch_models.py already uses). Re-running for the same object
overwrites its .npy and refreshes photo count/backbone, but preserves any
match thresholds already tuned by hand in manifest.json.
"""

import argparse
import glob
import json
from pathlib import Path

import numpy as np
from backbone import EmbeddingBackbone
from gallery_matcher import DEFAULT_MARGIN_MIN, DEFAULT_MIN_SIMILARITY, l2_normalize

RECOMMENDED_MIN_PHOTOS = 10
RECOMMENDED_MAX_PHOTOS = 30


def build_gallery(
    object_name: str, photo_glob: str, backbone_id: str, gallery_dir: Path
) -> int:
    from PIL import Image

    paths = sorted(glob.glob(photo_glob))
    if not paths:
        raise SystemExit(f"No photos matched {photo_glob!r}")
    if not (RECOMMENDED_MIN_PHOTOS <= len(paths) <= RECOMMENDED_MAX_PHOTOS):
        print(
            f"[gallery_build] WARNING: {len(paths)} photos for {object_name!r} "
            f"(recommended {RECOMMENDED_MIN_PHOTOS}-{RECOMMENDED_MAX_PHOTOS})"
        )

    gallery_dir.mkdir(parents=True, exist_ok=True)
    crops = [Image.open(p).convert("RGB") for p in paths]
    backbone = EmbeddingBackbone(backbone_id).load()
    vectors = l2_normalize(backbone.embed_batch(crops))

    npy_name = f"{object_name}.npy"
    np.save(gallery_dir / npy_name, vectors)

    manifest_path = gallery_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
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
        default="vit_small_patch14_dinov2.lvd142m",
        help="timm model id, or clip:<name> (e.g. clip:ViT-B/32)",
    )
    parser.add_argument("--gallery-dir", default="gallery")
    args = parser.parse_args()
    build_gallery(args.object, args.photos, args.backbone, Path(args.gallery_dir))


if __name__ == "__main__":
    main()
