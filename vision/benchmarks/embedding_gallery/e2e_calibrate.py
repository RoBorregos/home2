#!/usr/bin/env python3
"""Calibrates (min_similarity, margin_min) per-class against REAL box-proposer
crops (not report.py's oracle crops) via greedy coordinate descent — see results/e2e_calibrate_*.json for why a single global pair under-performs."""

import argparse
import itertools
import json
from datetime import datetime
from pathlib import Path

import _paths  # noqa: F401

import numpy as np
from backbone import EmbeddingBackbone
from gallery_matcher import (
    DEFAULT_MARGIN_MIN,
    DEFAULT_MIN_SIMILARITY,
    UNKNOWN,
    Gallery,
)

from e2e_eval import make_box_proposer, match_gt_to_boxes
from prepare_dataset import (
    OUT_OF_GALLERY_CLASSES,
    iter_split,
    load_class_names,
    load_translation,
)
from report import (
    KNOWN_LIMITATION_CLASSES,
    MARGIN_GRID,
    SIM_GRID,
    embed_gallery_photos,
)

RECALL_TARGET = 0.80  # matches report.py's already-adjusted target
REJECTION_TARGET = 0.80
CACHE_PATH = Path(__file__).parent / "results" / "e2e_crops_cache.npz"
# Read from gallery_matcher.py's own constants (single source of truth) so
# this comparison baseline can't drift out of sync with what's actually live.
GLOBAL_DEFAULT = (DEFAULT_MIN_SIMILARITY, DEFAULT_MARGIN_MIN)

# First per-class run picked degenerate floors (min_similarity=0.1,
# margin_min=0.0) — overfit to the calibration sample; floor the search.
MIN_SIMILARITY_FLOOR = 0.3
MARGIN_MIN_FLOOR = 0.02
PER_CLASS_SIM_GRID = [v for v in SIM_GRID if v >= MIN_SIMILARITY_FLOOR]
PER_CLASS_MARGIN_GRID = [v for v in MARGIN_GRID if v >= MARGIN_MIN_FLOOR]


def collect_real_crops(
    source: Path,
    split: str,
    n_images: int,
    seed: int,
    backbone_id: str,
    cache_path: Path | None = None,
):
    cache_path = cache_path or CACHE_PATH
    import cv2
    from PIL import Image as PILImage

    names = load_class_names(source)
    translation = load_translation()

    print("[calib] loading gallery (data/gallery_photos/, clean enrollment crops)...")
    backbone = EmbeddingBackbone(backbone_id).load()
    gallery_embeddings = embed_gallery_photos(backbone)
    gallery_labels = set(gallery_embeddings)
    # Derived from the backbone's output, not hardcoded — a different
    # --backbone (e.g. ViT-S/14 is 384-dim) must not silently zero-array the wrong shape.
    embed_dim = next(iter(gallery_embeddings.values())).shape[-1]
    print(f"[calib] gallery has {len(gallery_labels)} objects, embed_dim={embed_dim}")

    print("[calib] loading box proposer (YOLOE prompt-free, conf=0.10)...")
    propose = make_box_proposer()

    rng = np.random.default_rng(seed)
    samples = list(iter_split(source, split, names, translation))
    rng.shuffle(samples)
    samples = samples[:n_images]
    print(f"[calib] extracting real-crop embeddings from {len(samples)} images...")

    held_out_labels, held_out_emb = [], []
    ood_emb = []
    missed_gallery = missed_ood = 0

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
        crops, kept_px = [], []
        for bbox, _poly in proposals:
            x1, y1, x2, y2 = bbox
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            crops.append(PILImage.fromarray(image[y1:y2, x1:x2][:, :, ::-1]))
            kept_px.append([x1, y1, x2, y2])

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
        f"({missed_gallery} missed by proposer), {len(ood_emb)} real out-of-gallery crops "
        f"({missed_ood} missed by proposer)"
    )

    cache_path.parent.mkdir(parents=True, exist_ok=True)
    gallery_names = sorted(gallery_embeddings)
    gallery_concat = np.concatenate(
        [gallery_embeddings[n] for n in gallery_names], axis=0
    )
    gallery_sizes = np.array([len(gallery_embeddings[n]) for n in gallery_names])
    np.savez(
        cache_path,
        held_out_emb=held_out_emb,
        held_out_labels=np.array(held_out_labels),
        ood_emb=ood_emb,
        gallery_concat=gallery_concat,
        gallery_names=np.array(gallery_names),
        gallery_sizes=gallery_sizes,
    )
    print(f"[calib] cached embeddings -> {cache_path}")

    return gallery_embeddings, held_out_labels, held_out_emb, ood_emb


def load_cached_crops(cache_path: Path | None = None):
    cache_path = cache_path or CACHE_PATH
    print(
        f"[calib] loading cached embeddings from {cache_path} (skipping box proposer + backbone)..."
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
        f"[calib] loaded {len(held_out_labels)} held-out crops, {len(ood_emb)} out-of-gallery crops, "
        f"{len(gallery_embeddings)} gallery objects"
    )
    return gallery_embeddings, held_out_labels, held_out_emb, ood_emb


def gated_recall(held_out_labels, preds) -> float:
    kept = [
        (label, p)
        for label, (p, _, _) in zip(held_out_labels, preds)
        if label not in KNOWN_LIMITATION_CLASSES
    ]
    if not kept:
        return 1.0
    return sum(1 for label, p in kept if p == label) / len(kept)


def rejection_rate(preds) -> float:
    if not preds:
        return 1.0
    return sum(1 for (p, _, _) in preds if p == UNKNOWN) / len(preds)


def build_gallery(gallery_embeddings, thresholds: dict) -> Gallery:
    manifest = {
        "objects": {
            name: {"min_similarity": s, "margin_min": m}
            for name, (s, m) in thresholds.items()
        }
    }
    return Gallery(gallery_embeddings, manifest)


def score_thresholds(
    gallery_embeddings, held_out_labels, held_out_emb, ood_emb, thresholds
) -> tuple:
    gallery = build_gallery(gallery_embeddings, thresholds)
    recall = gated_recall(held_out_labels, gallery.match_batch(held_out_emb))
    rejection = rejection_rate(gallery.match_batch(ood_emb))
    both_met = recall >= RECALL_TARGET and rejection >= REJECTION_TARGET
    return both_met, min(recall, rejection), recall, rejection


def optimize_global(gallery_embeddings, held_out_labels, held_out_emb, ood_emb):
    print(
        f"[calib] sweeping {len(SIM_GRID) * len(MARGIN_GRID)} GLOBAL threshold combinations..."
    )
    best = None
    for min_sim, margin_min in itertools.product(SIM_GRID, MARGIN_GRID):
        thresholds = {name: (min_sim, margin_min) for name in gallery_embeddings}
        both_met, score, recall, rejection = score_thresholds(
            gallery_embeddings, held_out_labels, held_out_emb, ood_emb, thresholds
        )
        candidate = {
            "min_similarity": min_sim,
            "margin_min": margin_min,
            "recall_gated": round(recall, 3),
            "rejection": round(rejection, 3),
            "both_met": both_met,
            "score": score,
        }
        if best is None or (candidate["both_met"], candidate["score"]) > (
            best["both_met"],
            best["score"],
        ):
            best = candidate
    return best


def optimize_per_class(
    gallery_embeddings, held_out_labels, held_out_emb, ood_emb, init, rounds=3
):
    classes = sorted(gallery_embeddings)
    thresholds = {c: init for c in classes}
    _, score, recall, rejection = score_thresholds(
        gallery_embeddings, held_out_labels, held_out_emb, ood_emb, thresholds
    )
    print(
        f"[calib] per-class start (all @ {init}): score={score:.3f} recall={recall:.3f} rejection={rejection:.3f}"
    )

    for rnd in range(rounds):
        improved = False
        for c in classes:
            best_local = (score, thresholds[c])
            for s, m in itertools.product(PER_CLASS_SIM_GRID, PER_CLASS_MARGIN_GRID):
                trial = dict(thresholds)
                trial[c] = (s, m)
                _, sc, _, _ = score_thresholds(
                    gallery_embeddings, held_out_labels, held_out_emb, ood_emb, trial
                )
                if sc > best_local[0]:
                    best_local = (sc, (s, m))
            if best_local[1] != thresholds[c]:
                thresholds[c] = best_local[1]
                improved = True
        _, score, recall, rejection = score_thresholds(
            gallery_embeddings, held_out_labels, held_out_emb, ood_emb, thresholds
        )
        print(
            f"[calib] round {rnd}: score={score:.3f} recall={recall:.3f} rejection={rejection:.3f}"
        )
        if not improved:
            print("[calib] converged (no class improved this round)")
            break

    both_met, score, recall, rejection = score_thresholds(
        gallery_embeddings, held_out_labels, held_out_emb, ood_emb, thresholds
    )
    return thresholds, {
        "both_met": both_met,
        "score": score,
        "recall_gated": round(recall, 3),
        "rejection": round(rejection, 3),
    }


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--source", help="path to a YOLO-seg export (not needed with --from-cache)"
    )
    parser.add_argument("--split", default="test")
    parser.add_argument("--n-images", type=int, default=149)
    parser.add_argument("--backbone", default="vit_base_patch14_dinov2.lvd142m")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument(
        "--from-cache",
        action="store_true",
        help="skip box proposer + backbone, reuse results/e2e_crops_cache.npz",
    )
    parser.add_argument(
        "--rounds", type=int, default=3, help="per-class coordinate-descent rounds"
    )
    args = parser.parse_args()

    if args.from_cache:
        gallery_embeddings, held_out_labels, held_out_emb, ood_emb = load_cached_crops()
    else:
        if not args.source:
            raise SystemExit("--source is required unless --from-cache is set")
        gallery_embeddings, held_out_labels, held_out_emb, ood_emb = collect_real_crops(
            Path(args.source).expanduser(),
            args.split,
            args.n_images,
            args.seed,
            args.backbone,
        )

    global_best = optimize_global(
        gallery_embeddings, held_out_labels, held_out_emb, ood_emb
    )
    print(f"\n[calib] best GLOBAL threshold: {global_best}")

    per_class_thresholds, per_class_result = optimize_per_class(
        gallery_embeddings,
        held_out_labels,
        held_out_emb,
        ood_emb,
        init=(global_best["min_similarity"], global_best["margin_min"]),
        rounds=args.rounds,
    )

    print("\n=== Comparison on the SAME real-crop data ===")
    print(f"{'':45s} {'Recall (gated)':>16s} {'Rejection':>12s}")
    print(
        f"{'global (' + str(GLOBAL_DEFAULT[0]) + '/' + str(GLOBAL_DEFAULT[1]) + ', already in production)':45s} "
    )
    _, _, prod_recall, prod_rejection = score_thresholds(
        gallery_embeddings,
        held_out_labels,
        held_out_emb,
        ood_emb,
        {c: GLOBAL_DEFAULT for c in gallery_embeddings},
    )
    print(
        f"{'  production global threshold':45s} {prod_recall * 100:15.1f}% {prod_rejection * 100:11.1f}%"
    )
    print(
        f"{'  best global (this run)':45s} "
        f"{global_best['recall_gated'] * 100:15.1f}% {global_best['rejection'] * 100:11.1f}%"
    )
    print(
        f"{'  per-class (greedy)':45s} "
        f"{per_class_result['recall_gated'] * 100:15.1f}% {per_class_result['rejection'] * 100:11.1f}%"
    )

    results_dir = Path(__file__).parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    out_path = (
        results_dir
        / f"e2e_calibrate_perclass_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    out_path.write_text(
        json.dumps(
            {
                "n_held_out": len(held_out_labels),
                "n_ood": len(ood_emb),
                "production_global": {
                    "min_similarity": GLOBAL_DEFAULT[0],
                    "margin_min": GLOBAL_DEFAULT[1],
                    "recall_gated": round(prod_recall, 3),
                    "rejection": round(prod_rejection, 3),
                },
                "best_global": global_best,
                "per_class_thresholds": {
                    c: {"min_similarity": s, "margin_min": m}
                    for c, (s, m) in per_class_thresholds.items()
                },
                "per_class_result": per_class_result,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"\n[calib] wrote {out_path}")

    if per_class_result["both_met"] and per_class_result["score"] > max(
        global_best["score"], min(prod_recall, prod_rejection)
    ):
        (results_dir / "e2e_thresholds_perclass.json").write_text(
            json.dumps(
                {
                    c: {"min_similarity": s, "margin_min": m}
                    for c, (s, m) in per_class_thresholds.items()
                },
                indent=2,
                sort_keys=True,
            )
            + "\n"
        )
        print(
            "[calib] per-class thresholds beat the global default -> results/e2e_thresholds_perclass.json"
        )
    else:
        print(
            "[calib] per-class optimization did not clearly beat the global threshold on this data."
        )


if __name__ == "__main__":
    main()
