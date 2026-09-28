#!/usr/bin/env python3
"""End-to-end recall check: the production BOX PROPOSER's own predicted
crops -> DINOv2 -> gallery match, instead of report.py's ground-truth-polygon
crops. report.py's ~82% gated recall was measured on crops cut from
RCW2026_v2's hand-labeled polygons — perfect boxes a real detector never
gives you. A first run of this script (plain bbox crops) measured 71.6%
recall (all classes) — a real ~5-10pt drop, because loose/shifted real boxes
include background clutter the embedding matcher never saw during
calibration.

This version also tests a fix: YOLOE is a SEGMENTATION model — it already
returns a per-instance mask, never used anywhere in this pipeline before.
Blanking the background OUTSIDE the mask (filled with DINOv2's own
normalization mean, so it contributes ~zero signal post-normalization,
rather than an arbitrary black square) before embedding should remove the
clutter that a loose bbox crop drags in. Both variants (plain bbox vs.
masked) run in the SAME pass over the SAME box proposals, so the comparison
is apples-to-apples and YOLOE only runs once.

Two distinct failure modes are reported separately:
  - "missed_by_proposer": the box proposer never localized the object at
    IoU>=0.5 at all — an embedding-matching fix can't help this.
  - "wrong_label": the proposer found the object fine, but the embedding
    matcher assigned the wrong label (or unknown).

Usage:
    python3 e2e_eval.py --source ~/Downloads/RCW2026_v2 --n-images 150
"""

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[2]
        / "packages"
        / "object_detector_2d"
        / "scripts"
        / "detectors"
    ),
)

import numpy as np
from backbone import EmbeddingBackbone
from gallery_matcher import UNKNOWN, Gallery

from prepare_dataset import (
    OUT_OF_GALLERY_CLASSES,
    iter_split,
    load_class_names,
    load_translation,
)
from report import KNOWN_LIMITATION_CLASSES, embed_gallery_photos

DETECTORS_DIR = (
    Path(__file__).resolve().parents[2]
    / "packages"
    / "object_detector_2d"
    / "scripts"
    / "detectors"
)
BOX_PROPOSER_WEIGHT = DETECTORS_DIR / "yoloe-11l-seg-pf.pt"
BOX_CONF = 0.10

# DINOv2's own timm normalization mean, in 0-255 RGB — filling the background
# with this color makes it contribute ~zero signal after normalization,
# instead of an arbitrary (and out-of-distribution) black square.
NEUTRAL_FILL_RGB = (124, 116, 104)


def iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def make_box_proposer():
    from ultralytics import YOLOE

    model = YOLOE(str(BOX_PROPOSER_WEIGHT))

    def propose(image):
        """Returns [(bbox_px, mask_polygon_or_None), ...]."""
        results = model.predict(image, conf=BOX_CONF, verbose=False)
        out_list = []
        for out in results:
            if out.boxes is None:
                continue
            masks = out.masks
            for i, box in enumerate(out.boxes):
                bbox = [round(v) for v in box.xyxy[0].tolist()]
                poly = None
                if masks is not None and i < len(masks):
                    xy = masks[i].xy
                    if xy and len(xy[0]) >= 3:
                        poly = np.asarray(xy[0], dtype=np.int32)
                out_list.append((bbox, poly))
        return out_list

    return propose


def masked_crop(image, bbox, poly):
    """Background outside `poly` filled with NEUTRAL_FILL_RGB (image is BGR,
    so the fill is reversed to match)."""
    import cv2

    x1, y1, x2, y2 = bbox
    crop = image[y1:y2, x1:x2].copy()
    if poly is None or len(poly) < 3:
        return crop
    full_mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.fillPoly(full_mask, [poly], 255)
    crop_mask = full_mask[y1:y2, x1:x2]
    fill_bgr = NEUTRAL_FILL_RGB[::-1]
    crop[crop_mask == 0] = fill_bgr
    return crop


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--source",
        required=True,
        help="path to a YOLO-seg export (see prepare_dataset.py)",
    )
    parser.add_argument("--split", default="test")
    parser.add_argument("--n-images", type=int, default=150)
    parser.add_argument("--backbone", default="vit_base_patch14_dinov2.lvd142m")
    parser.add_argument("--min-similarity", type=float, default=0.4)
    parser.add_argument("--margin-min", type=float, default=0.04)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    import cv2
    from PIL import Image as PILImage

    source = Path(args.source).expanduser()
    names = load_class_names(source)
    translation = load_translation()

    print("[e2e] loading gallery (data/gallery_photos/, already built)...")
    backbone = EmbeddingBackbone(args.backbone).load()
    gallery_embeddings = embed_gallery_photos(backbone)
    manifest = {
        "objects": {
            name: {"min_similarity": args.min_similarity, "margin_min": args.margin_min}
            for name in gallery_embeddings
        }
    }
    gallery = Gallery(gallery_embeddings, manifest)
    gallery_labels = set(gallery_embeddings)
    print(f"[e2e] gallery has {len(gallery_labels)} objects")

    print("[e2e] loading box proposer (YOLOE prompt-free, conf=0.10)...")
    propose = make_box_proposer()

    rng = np.random.default_rng(args.seed)
    samples = list(iter_split(source, args.split, names, translation))
    rng.shuffle(samples)
    samples = samples[: args.n_images]
    print(
        f"[e2e] evaluating on {len(samples)} images from {source.name}/{args.split}..."
    )

    cases = []  # one dict per gallery-object instance, both variants recorded
    ood_cases = []  # one dict per out-of-gallery instance (unknown-rejection check)

    for i, (img_path, gt_boxes) in enumerate(samples):
        image = cv2.imread(str(img_path))
        if image is None:
            continue
        h, w = image.shape[:2]

        gt_in_gallery = [
            (label, bbox) for label, bbox in gt_boxes if label in gallery_labels
        ]
        gt_ood = [
            (label, bbox) for label, bbox in gt_boxes if label in OUT_OF_GALLERY_CLASSES
        ]
        if not gt_in_gallery and not gt_ood:
            continue

        proposals = propose(image)
        plain_crops, masked_crops, kept_px, kept_poly = [], [], [], []
        for bbox, poly in proposals:
            x1, y1, x2, y2 = bbox
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            plain_crops.append(PILImage.fromarray(image[y1:y2, x1:x2][:, :, ::-1]))
            masked_crops.append(
                PILImage.fromarray(
                    masked_crop(image, [x1, y1, x2, y2], poly)[:, :, ::-1]
                )
            )
            kept_px.append([x1, y1, x2, y2])
            kept_poly.append(poly)

        plain_labels = [None] * len(kept_px)
        masked_labels = [None] * len(kept_px)
        if kept_px:
            plain_emb = backbone.embed_batch(plain_crops)
            masked_emb = backbone.embed_batch(masked_crops)
            plain_labels = [m[0] for m in gallery.match_batch(plain_emb)]
            masked_labels = [m[0] for m in gallery.match_batch(masked_emb)]

        used = set()
        for true_label, gt_bbox in gt_in_gallery:
            best_iou, best_idx = 0.0, -1
            for idx, pred_bbox in enumerate(kept_px):
                if idx in used:
                    continue
                score = iou(gt_bbox, pred_bbox)
                if score > best_iou:
                    best_iou, best_idx = score, idx
            if best_iou < 0.5:
                cases.append({"expected": true_label, "missed_by_proposer": True})
                continue
            used.add(best_idx)
            cases.append(
                {
                    "expected": true_label,
                    "missed_by_proposer": False,
                    "plain_pred": plain_labels[best_idx],
                    "masked_pred": masked_labels[best_idx],
                    "had_mask": kept_poly[best_idx] is not None,
                }
            )

        for true_label, gt_bbox in gt_ood:
            best_iou, best_idx = 0.0, -1
            for idx, pred_bbox in enumerate(kept_px):
                score = iou(gt_bbox, pred_bbox)
                if score > best_iou:
                    best_iou, best_idx = score, idx
            if best_iou < 0.5:
                ood_cases.append({"expected": true_label, "missed_by_proposer": True})
                continue
            ood_cases.append(
                {
                    "expected": true_label,
                    "missed_by_proposer": False,
                    "plain_pred": plain_labels[best_idx],
                    "masked_pred": masked_labels[best_idx],
                }
            )

        if (i + 1) % 25 == 0:
            print(f"[e2e]   ...{i + 1}/{len(samples)} images processed")

    def summarize(pred_key: str, gated: bool) -> dict:
        localized = [c for c in cases if not c["missed_by_proposer"]]
        if gated:
            localized = [
                c for c in localized if c["expected"] not in KNOWN_LIMITATION_CLASSES
            ]
        total = len(localized)
        correct = sum(1 for c in localized if c[pred_key] == c["expected"])
        return {
            "total": total,
            "correct": correct,
            "recall": correct / total if total else 0.0,
        }

    total_instances = len(cases)
    missed = sum(1 for c in cases if c["missed_by_proposer"])
    with_mask = sum(
        1 for c in cases if not c["missed_by_proposer"] and c.get("had_mask")
    )

    print("\n=== End-to-end results (real box proposer, not ground-truth crops) ===")
    print(f"Total gallery-object instances: {total_instances}")
    print(
        f"Missed by box proposer entirely: {missed} ({missed / total_instances * 100:.1f}%)"
    )
    print(
        f"Localized objects that had a usable mask: {with_mask}/{total_instances - missed}"
    )

    for label, key in [
        ("plain bbox crop", "plain_pred"),
        ("masked crop (background blanked)", "masked_pred"),
    ]:
        all_r = summarize(key, gated=False)
        gated_r = summarize(key, gated=True)
        print(f"\n-- {label} --")
        print(
            f"  recall@1 (all classes):   {all_r['recall'] * 100:.1f}%  ({all_r['correct']}/{all_r['total']})"
        )
        print(
            f"  recall@1 (gated, excl. cutlery/kitchenware/cans): "
            f"{gated_r['recall'] * 100:.1f}%  ({gated_r['correct']}/{gated_r['total']})"
        )

    print(
        f"\n=== Unknown-rejection (out-of-gallery objects: {sorted(OUT_OF_GALLERY_CLASSES)}) ==="
    )
    ood_total = len(ood_cases)
    ood_missed = sum(1 for c in ood_cases if c["missed_by_proposer"])
    print(f"Total out-of-gallery instances: {ood_total}")
    print(
        f"Missed by proposer (never boxed -> nothing published, not a false positive): "
        f"{ood_missed} ({ood_missed / ood_total * 100:.1f}%)"
        if ood_total
        else "Total out-of-gallery instances: 0"
    )
    if ood_total:
        localized_ood = [c for c in ood_cases if not c["missed_by_proposer"]]
        for label, key in [
            ("plain bbox crop", "plain_pred"),
            ("masked crop", "masked_pred"),
        ]:
            rejected = sum(1 for c in localized_ood if c[key] == UNKNOWN)
            n = len(localized_ood)
            rate = rejected / n if n else 0.0
            print(
                f"  {label}: unknown-rejection = {rate * 100:.1f}% ({rejected}/{n} localized instances)"
            )

    out_path = (
        Path(__file__).parent
        / "results"
        / f"e2e_eval_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        json.dumps(
            {"n_images": len(samples), "cases": cases, "ood_cases": ood_cases}, indent=2
        )
        + "\n"
    )
    print(f"\n[e2e] wrote {out_path}")


if __name__ == "__main__":
    main()
