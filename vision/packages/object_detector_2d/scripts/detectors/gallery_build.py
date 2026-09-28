#!/usr/bin/env python3
"""Build or update one gallery entry (name -> N embeddings) from a folder of photos.

Usage (inside the vision container — needs timm/torch, and clip if the
backbone id starts with "clip:"):

    python3 gallery_build.py --object coke \\
        --photos "gallery_photos/coke/*.jpg" \\
        --gallery-dir gallery/

--backbone defaults to whatever MODEL_CONFIGS["embedding_gallery"]["backbone"]
in registry.py uses — override it only if you deliberately want a mismatched
object (you don't: every object in one gallery must share the same backbone,
same embedding dimension, or Gallery.load() crashes at the NEXT node restart,
not now, which is a much worse time to find out).

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

try:
    # Single source of truth for "what backbone does production actually
    # use" — keeps this default from silently drifting out of sync with
    # MODEL_CONFIGS["embedding_gallery"]["backbone"] in registry.py.
    from registry import MODEL_CONFIGS

    DEFAULT_BACKBONE = MODEL_CONFIGS["embedding_gallery"]["backbone"]
except Exception:
    DEFAULT_BACKBONE = "vit_base_patch14_dinov2.lvd142m"


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

    # Every object in one gallery must share embedding dimension (Gallery
    # stacks them into one matrix). Check against another object BEFORE
    # writing anything — this exact mismatch (wrong --backbone) used to
    # write a bad .npy silently and crash at the NEXT node restart instead.
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
    args = parser.parse_args()
    build_gallery(args.object, args.photos, args.backbone, Path(args.gallery_dir))


if __name__ == "__main__":
    main()
