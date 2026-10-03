"""Embedding helpers: crops from data/ and from the real box proposer."""

import glob
import json
from pathlib import Path

import numpy as np
from embedding_gallery.core.image_embedder import ImageEmbedder

from core.dataset import (
    DATA_DIR,
    E2E_CACHE_PATH,
    OUT_OF_GALLERY_CLASSES,
    load_translation,
    require_dir,
)
from core.metrics import match_gt_to_boxes
from core.prepare_dataset import iter_split, load_class_names
from core.proposers import make_box_proposer


def _load_images(paths: list[Path]):
    from PIL import Image

    return [Image.open(p).convert("RGB") for p in paths]


def embed_gallery_photos(backbone) -> dict[str, np.ndarray]:
    """Embeds data/gallery_photos/<object>/*.{jpg,png}: {object: [N, D]}."""
    embeddings = {}
    for obj_dir in sorted(require_dir(DATA_DIR / "gallery_photos").iterdir()):
        if not obj_dir.is_dir():
            continue
        paths = sorted(
            Path(p)
            for p in glob.glob(str(obj_dir / "*.jpg"))
            + glob.glob(str(obj_dir / "*.png"))
        )
        if not paths:
            continue
        embeddings[obj_dir.name] = backbone.embed_batch(_load_images(paths))
    return embeddings


def embed_labeled_dir(
    backbone, data_dir: Path
) -> tuple[list[str], list[str], np.ndarray]:
    """Returns (filenames, labels, embeddings[N, D]) from annotations.json."""
    ann_path = require_dir(data_dir) / "annotations.json"
    if not ann_path.exists():
        raise SystemExit(f"No {ann_path}, see README.md for the expected format.")
    labels_by_file = json.loads(ann_path.read_text())
    filenames = list(labels_by_file.keys())
    embeddings = backbone.embed_batch(_load_images([data_dir / f for f in filenames]))
    labels = [labels_by_file[f] for f in filenames]
    return filenames, labels, embeddings


def embed_unlabeled_dir(backbone, data_dir: Path) -> tuple[list[str], np.ndarray]:
    """Returns (filenames, embeddings[N, D]) for every image in data_dir."""
    filenames = sorted(
        p.name
        for p in require_dir(data_dir).iterdir()
        if p.suffix.lower() in (".jpg", ".jpeg", ".png")
    )
    if not filenames:
        raise SystemExit(
            f"No images in {data_dir}, need out-of-gallery crops to measure rejection."
        )
    embeddings = backbone.embed_batch(_load_images([data_dir / f for f in filenames]))
    return filenames, embeddings


def sample_images(source: Path, split: str, n_images: int, seed: int) -> list:
    """Seeded sample of (image_path, [(label, bbox_px), ...]) from a YOLO-seg export split."""
    names = load_class_names(source)
    samples = list(iter_split(source, split, names, load_translation()))
    np.random.default_rng(seed).shuffle(samples)
    return samples[:n_images]


def split_ground_truth(gt_boxes: list, gallery_labels: set) -> tuple[list, list]:
    """Splits ground truth into (objects in the gallery, out-of-gallery objects)."""
    in_gallery = [(label, bbox) for label, bbox in gt_boxes if label in gallery_labels]
    out_of_gallery = [
        (label, bbox) for label, bbox in gt_boxes if label in OUT_OF_GALLERY_CLASSES
    ]
    return in_gallery, out_of_gallery


def clip_proposals(proposals: list, width: int, height: int) -> list:
    """Clips the proposer's boxes to the image and drops empty ones: [([x1, y1, x2, y2], polygon)]."""
    clipped = []
    for (x1, y1, x2, y2), poly in proposals:
        x1, y1, x2, y2 = max(0, x1), max(0, y1), min(width, x2), min(height, y2)
        if x2 > x1 and y2 > y1:
            clipped.append(([x1, y1, x2, y2], poly))
    return clipped


def collect_real_crops(
    source: Path,
    split: str,
    n_images: int,
    seed: int,
    backbone_id: str,
    cache_path: Path | None = None,
):
    """Embeds the production proposer's crops matched to ground truth.

    Returns (gallery_embeddings, held_out_labels, held_out_emb, ood_emb) and
    caches them to `cache_path` (default E2E_CACHE_PATH) for --from-cache.
    """
    cache_path = cache_path or E2E_CACHE_PATH
    import cv2
    from PIL import Image as PILImage

    print("[calib] loading gallery (data/gallery_photos/, clean enrollment crops)...")
    backbone = ImageEmbedder(backbone_id).load()
    gallery_embeddings = embed_gallery_photos(backbone)
    gallery_labels = set(gallery_embeddings)
    # Derived from the backbone's output, not hardcoded: a different
    # --backbone (e.g. ViT-S/14 is 384-dim) must not silently zero-array the
    # wrong shape.
    embed_dim = next(iter(gallery_embeddings.values())).shape[-1]
    print(f"[calib] gallery has {len(gallery_labels)} objects, embed_dim={embed_dim}")

    print("[calib] loading box proposer (YOLOE prompt-free, conf=0.10)...")
    propose = make_box_proposer()

    samples = sample_images(source, split, n_images, seed)
    print(f"[calib] extracting real-crop embeddings from {len(samples)} images...")

    held_out_labels, held_out_emb = [], []
    ood_emb = []
    missed_gallery = missed_ood = 0

    for i, (img_path, gt_boxes) in enumerate(samples):
        image = cv2.imread(str(img_path))
        if image is None:
            continue
        gt_in_gallery, gt_ood = split_ground_truth(gt_boxes, gallery_labels)
        if not gt_in_gallery and not gt_ood:
            continue

        h, w = image.shape[:2]
        kept_px = [box for box, _ in clip_proposals(propose(image), w, h)]
        crops = [
            PILImage.fromarray(image[y1:y2, x1:x2][:, :, ::-1])
            for x1, y1, x2, y2 in kept_px
        ]

        crop_emb = (
            backbone.embed_batch(crops)
            if crops
            else np.zeros((0, embed_dim), dtype=np.float32)
        )

        used = set()
        for true_label, _gt_bbox, idx in match_gt_to_boxes(
            gt_in_gallery, kept_px, used=used
        ):
            if idx is None:
                missed_gallery += 1
                continue
            held_out_labels.append(true_label)
            held_out_emb.append(crop_emb[idx])

        for _true_label, _gt_bbox, idx in match_gt_to_boxes(gt_ood, kept_px):
            if idx is None:
                missed_ood += 1
                continue
            ood_emb.append(crop_emb[idx])

        if (i + 1) % 25 == 0:
            print(f"[calib]   ...{i + 1}/{len(samples)} images processed")

    held_out_emb = (
        np.stack(held_out_emb)
        if held_out_emb
        else np.zeros((0, embed_dim), dtype=np.float32)
    )
    ood_emb = (
        np.stack(ood_emb) if ood_emb else np.zeros((0, embed_dim), dtype=np.float32)
    )
    print(
        f"\n[calib] {len(held_out_labels)} real held-out crops "
        f"({missed_gallery} missed by proposer), {len(ood_emb)} real "
        f"out-of-gallery crops ({missed_ood} missed by proposer)"
    )

    cache_path.parent.mkdir(parents=True, exist_ok=True)
    gallery_names = sorted(gallery_embeddings)
    np.savez(
        cache_path,
        held_out_emb=held_out_emb,
        held_out_labels=np.array(held_out_labels),
        ood_emb=ood_emb,
        gallery_concat=np.concatenate(
            [gallery_embeddings[n] for n in gallery_names], axis=0
        ),
        gallery_names=np.array(gallery_names),
        gallery_sizes=np.array([len(gallery_embeddings[n]) for n in gallery_names]),
    )
    print(f"[calib] cached embeddings -> {cache_path}")

    return gallery_embeddings, held_out_labels, held_out_emb, ood_emb


def load_cached_crops(cache_path: Path | None = None):
    """Loads what collect_real_crops cached (skips proposer and backbone)."""
    cache_path = cache_path or E2E_CACHE_PATH
    print(
        f"[calib] loading cached embeddings from {cache_path} "
        "(skipping box proposer + backbone)..."
    )
    data = np.load(cache_path, allow_pickle=False)
    names = data["gallery_names"].tolist()
    sizes = data["gallery_sizes"].tolist()
    concat = data["gallery_concat"]
    gallery_embeddings = {}
    offset = 0
    for name, size in zip(names, sizes):
        gallery_embeddings[name] = concat[offset : offset + size]
        offset += size
    held_out_labels = data["held_out_labels"].tolist()
    held_out_emb = data["held_out_emb"]
    ood_emb = data["ood_emb"]
    print(
        f"[calib] loaded {len(held_out_labels)} held-out crops, {len(ood_emb)} "
        f"out-of-gallery crops, {len(gallery_embeddings)} gallery objects"
    )
    return gallery_embeddings, held_out_labels, held_out_emb, ood_emb
