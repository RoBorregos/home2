#!/usr/bin/env python3
"""Benchmark tasks (boxes, embeddings, e2e_eval, e2e_calibrate) and their TASK_REGISTRY.

Each task's `run(**kwargs)` returns a plain dict; report.py prints and saves it.
run.sh calls `python3 tasks.py <task> [options]`; unset options use each task's defaults.
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
from embedding_gallery.image_embedder import ImageEmbedder
from embedding_gallery.gallery_matcher import (
    DEFAULT_MARGIN_MIN,
    DEFAULT_MIN_SIMILARITY,
    UNKNOWN,
)

from lib.dataset import (
    DATA_DIR,
    E2E_CACHE_PATH,
    MODELS_PATH,
    OUT_OF_GALLERY_CLASSES,
    KNOWN_LIMITATION_CLASSES,
    RECALL_TARGET,
    REJECTION_TARGET,
    RESULTS_DIR,
    load_translation,
    require_dir,
)
from lib.embed import (
    collect_real_crops,
    embed_gallery_photos,
    embed_labeled_dir,
    embed_unlabeled_dir,
    load_cached_crops,
)
from lib.metrics import (
    build_gallery,
    gated_recall,
    iou,
    match_gt_to_boxes,
    optimize_global,
    optimize_per_class,
    score_thresholds,
)
from lib.prepare_dataset import iter_split, load_class_names
from lib.proposers import (
    make_box_proposer,
    make_yolo_agnostic_proposer,
    make_yoloe_proposer,
    masked_crop,
)
from report import report

DEFAULT_BACKBONE = "vit_base_patch14_dinov2.lvd142m"


class BoxesTask:
    """Phase 0: box recall of the candidate class-agnostic proposers.

    If none clears the recall bar, stop and escalate before building on it.
    """

    name = "boxes"

    @classmethod
    def run(cls, data=DATA_DIR / "box_recall", iou_threshold=0.5, **_) -> dict:
        data_dir = require_dir(Path(data))
        ann_path = data_dir / "annotations.json"
        if not ann_path.exists():
            raise SystemExit(
                f"No {ann_path}: box recall needs the labeled images that "
                "`./run.sh prepare --source <export>` builds."
            )
        annotations = json.loads(ann_path.read_text())
        candidates = json.loads(MODELS_PATH.read_text())["box_proposers"]

        proposers = {}
        for cfg in candidates:
            print(f"[box_recall] evaluating {cfg['name']} ...")
            try:
                if cfg["type"] == "yolo":
                    propose = make_yolo_agnostic_proposer(cfg["filename"], cfg["conf"])
                elif cfg["type"] == "yolo_e":
                    propose = make_yoloe_proposer(
                        cfg["filename"], cfg["conf"], cfg.get("prompt_classes")
                    )
                else:
                    print(
                        f"[box_recall] unknown proposer type {cfg['type']!r}, skipping"
                    )
                    continue
            except Exception as e:
                print(f"[box_recall] SKIP {cfg['name']}: {e}")
                continue

            metrics = cls._evaluate(propose, data_dir, annotations, iou_threshold)
            proposers[cfg["name"]] = metrics
            print(f"[box_recall] {cfg['name']}: {metrics}")
        return {"iou": iou_threshold, "proposers": proposers}

    @staticmethod
    def _evaluate(
        propose, data_dir: Path, annotations: dict, iou_thresh: float
    ) -> dict:
        import cv2

        total_gt = matched_gt = total_pred = total_matched_pred = 0
        latencies_ms = []

        for image_name, gt_boxes in annotations.items():
            image = cv2.imread(str(data_dir / image_name))
            if image is None:
                print(f"[box_recall] WARNING: could not read {image_name}, skipping")
                continue

            t0 = time.perf_counter()
            pred_boxes = propose(image)  # [x1, y1, x2, y2] pixel boxes
            latencies_ms.append((time.perf_counter() - t0) * 1000)

            total_gt += len(gt_boxes)
            total_pred += len(pred_boxes)
            used_pred = set()
            for gt in gt_boxes:
                best_iou, best_idx = 0.0, -1
                for idx, pred in enumerate(pred_boxes):
                    if idx in used_pred:
                        continue
                    score = iou(gt["bbox"], pred)
                    if score > best_iou:
                        best_iou, best_idx = score, idx
                if best_iou >= iou_thresh:
                    matched_gt += 1
                    used_pred.add(best_idx)
                    total_matched_pred += 1

        recall = matched_gt / total_gt if total_gt else 0.0
        # False positives per image, not per box: the volume a downstream
        # embedding matcher would actually have to reject as "unknown".
        fp_per_image = (total_pred - total_matched_pred) / max(1, len(annotations))
        avg_latency_ms = sum(latencies_ms) / len(latencies_ms) if latencies_ms else 0.0
        return {
            "recall_at_iou": round(recall, 3),
            "gt_boxes": total_gt,
            "matched_gt": matched_gt,
            "fp_per_image": round(fp_per_image, 2),
            "avg_latency_ms": round(avg_latency_ms, 1),
        }


class EmbeddingsTask:
    """Phase 1: embed every crop set once per backbone, sweep thresholds.

    gallery_photos/ must not overlap held_out/, or recall@1 measures
    memorization, not matching.
    """

    name = "embeddings"

    @classmethod
    def run(cls, backbones: list[str] | None = None, **_) -> dict:
        all_backbones = json.loads(MODELS_PATH.read_text())["backbones"]
        selected = (
            all_backbones
            if not backbones
            else [b for b in all_backbones if b["name"] in backbones]
        )
        if not selected:
            raise SystemExit(f"No matching backbones in {MODELS_PATH} for {backbones}")
        return {
            "results": [
                cls._run_backbone(b["name"], b["id"], b.get("img_size"))
                for b in selected
            ]
        }

    @staticmethod
    def _cases(filenames, expected_labels, predictions, pass_fn) -> list[dict]:
        return [
            {
                "input": filename,
                "expected": expected,
                "got": pred_label,
                "similarity": round(sim, 3),
                "margin": round(margin, 3),
                "passed": pass_fn(expected, pred_label),
            }
            for filename, expected, (pred_label, sim, margin) in zip(
                filenames, expected_labels, predictions
            )
        ]

    @classmethod
    def _run_backbone(
        cls, backbone_name: str, backbone_id: str, img_size: int | None = None
    ) -> dict:
        print(
            f"\n[report] backbone={backbone_name} ({backbone_id}"
            f"{f', img_size={img_size}' if img_size else ''}) "
            "- embedding all crops once..."
        )
        backbone = ImageEmbedder(backbone_id, img_size=img_size).load()

        gallery_embeddings = embed_gallery_photos(backbone)
        if not gallery_embeddings:
            raise SystemExit(
                f"No enrollment photos under {DATA_DIR / 'gallery_photos'}/<object>/*.jpg"
            )
        held_out_files, held_out_labels, held_out_emb = embed_labeled_dir(
            backbone, DATA_DIR / "held_out"
        )
        hard_neg_files, hard_neg_labels, hard_neg_emb = embed_labeled_dir(
            backbone, DATA_DIR / "hard_negatives"
        )
        ood_files, ood_emb = embed_unlabeled_dir(backbone, DATA_DIR / "out_of_gallery")

        print(
            f"[report] embedded: {sum(len(v) for v in gallery_embeddings.values())} "
            f"gallery, {len(held_out_files)} held_out, {len(hard_neg_files)} "
            f"hard_negatives, {len(ood_files)} out_of_gallery"
        )

        best = optimize_global(
            gallery_embeddings, held_out_labels, held_out_emb, ood_emb, verbose=False
        )
        print(f"[report] best threshold: {best}")

        gallery = build_gallery(
            gallery_embeddings,
            {
                n: (best["min_similarity"], best["margin_min"])
                for n in gallery_embeddings
            },
        )
        recall_preds = gallery.match_batch(held_out_emb)
        hard_neg_preds = gallery.match_batch(hard_neg_emb)
        ood_preds = gallery.match_batch(ood_emb)

        recall_cases = cls._cases(
            held_out_files, held_out_labels, recall_preds, lambda e, p: p == e
        )
        hard_neg_cases = cls._cases(
            hard_neg_files,
            hard_neg_labels,
            hard_neg_preds,
            lambda e, p: p == UNKNOWN or p == e,
        )
        ood_cases = cls._cases(
            ood_files, [UNKNOWN] * len(ood_files), ood_preds, lambda e, p: p == UNKNOWN
        )

        def pass_rate(cases: list[dict]) -> float:
            return sum(c["passed"] for c in cases) / len(cases) if cases else 0.0

        recall_at_1 = pass_rate(recall_cases)
        recall_at_1_gated = gated_recall(held_out_labels, recall_preds)
        hard_neg_precision = pass_rate(hard_neg_cases)
        unknown_rejection = pass_rate(ood_cases)

        return {
            "backbone": backbone_name,
            "backbone_id": backbone_id,
            "threshold": {
                "min_similarity": best["min_similarity"],
                "margin_min": best["margin_min"],
            },
            "recall_at_1": {
                "recall_at_1": round(recall_at_1, 3),
                "recall_at_1_gated": round(recall_at_1_gated, 3),
                "excluded_known_limitation_classes": sorted(KNOWN_LIMITATION_CLASSES),
                "total": len(recall_cases),
                "cases": recall_cases,
            },
            "hard_negative_precision": {
                "precision": round(hard_neg_precision, 3),
                "total": len(hard_neg_cases),
                "cases": hard_neg_cases,
            },
            "unknown_rejection_rate": {
                "unknown_rejection_rate": round(unknown_rejection, 3),
                "total": len(ood_cases),
                "cases": ood_cases,
            },
            "targets_met": recall_at_1_gated >= RECALL_TARGET
            and unknown_rejection >= REJECTION_TARGET,
        }


class E2EEvalTask:
    """Recall with the production proposer's real crops instead of ground-truth ones.

    Also compares plain-bbox crops against mask-blanked-background crops.
    """

    name = "e2e_eval"

    @classmethod
    def run(
        cls,
        source=None,
        split="test",
        n_images=150,
        backbone=DEFAULT_BACKBONE,
        min_similarity=0.4,
        margin_min=0.04,
        seed=1,
        **_,
    ) -> dict:
        if not source:
            raise SystemExit("e2e_eval needs --source (a YOLO-seg export)")
        import cv2
        from PIL import Image as PILImage

        source = Path(source).expanduser()
        names = load_class_names(source)
        translation = load_translation()

        print("[e2e] loading gallery (data/gallery_photos/, already built)...")
        bb = ImageEmbedder(backbone).load()
        gallery_embeddings = embed_gallery_photos(bb)
        gallery = build_gallery(
            gallery_embeddings,
            {n: (min_similarity, margin_min) for n in gallery_embeddings},
        )
        gallery_labels = set(gallery_embeddings)
        print(f"[e2e] gallery has {len(gallery_labels)} objects")

        print("[e2e] loading box proposer (YOLOE prompt-free, conf=0.10)...")
        propose = make_box_proposer()

        rng = np.random.default_rng(seed)
        samples = list(iter_split(source, split, names, translation))
        rng.shuffle(samples)
        samples = samples[:n_images]
        print(
            f"[e2e] evaluating on {len(samples)} images from {source.name}/{split}..."
        )

        cases = []  # one dict per gallery-object instance, both variants recorded
        ood_cases = []  # one dict per out-of-gallery instance

        for i, (img_path, gt_boxes) in enumerate(samples):
            image = cv2.imread(str(img_path))
            if image is None:
                continue
            h, w = image.shape[:2]

            gt_in_gallery = [
                (label, bbox) for label, bbox in gt_boxes if label in gallery_labels
            ]
            gt_ood = [
                (label, bbox)
                for label, bbox in gt_boxes
                if label in OUT_OF_GALLERY_CLASSES
            ]
            if not gt_in_gallery and not gt_ood:
                continue

            plain_crops, masked_crops, kept_px, kept_poly = [], [], [], []
            for bbox, poly in propose(image):
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
                plain_labels = [
                    m[0] for m in gallery.match_batch(bb.embed_batch(plain_crops))
                ]
                masked_labels = [
                    m[0] for m in gallery.match_batch(bb.embed_batch(masked_crops))
                ]

            used = set()
            for true_label, _gt_bbox, idx in match_gt_to_boxes(
                gt_in_gallery, kept_px, used=used
            ):
                if idx is None:
                    cases.append({"expected": true_label, "missed_by_proposer": True})
                    continue
                cases.append(
                    {
                        "expected": true_label,
                        "missed_by_proposer": False,
                        "plain_pred": plain_labels[idx],
                        "masked_pred": masked_labels[idx],
                        "had_mask": kept_poly[idx] is not None,
                    }
                )

            for true_label, _gt_bbox, idx in match_gt_to_boxes(gt_ood, kept_px):
                if idx is None:
                    ood_cases.append(
                        {"expected": true_label, "missed_by_proposer": True}
                    )
                    continue
                ood_cases.append(
                    {
                        "expected": true_label,
                        "missed_by_proposer": False,
                        "plain_pred": plain_labels[idx],
                        "masked_pred": masked_labels[idx],
                    }
                )

            if (i + 1) % 25 == 0:
                print(f"[e2e]   ...{i + 1}/{len(samples)} images processed")

        return {"n_images": len(samples), "cases": cases, "ood_cases": ood_cases}


class E2ECalibrateTask:
    """Calibrates (min_similarity, margin_min) per class against REAL
    box-proposer crops via greedy coordinate descent. A single global pair
    under-performs, see results/e2e_calibrate_*.json.
    """

    name = "e2e_calibrate"

    @classmethod
    def run(
        cls,
        source=None,
        split="test",
        n_images=149,
        backbone=DEFAULT_BACKBONE,
        seed=1,
        from_cache=False,
        rounds=3,
        results_dir=RESULTS_DIR,
        **_,
    ) -> dict:
        cache_path = Path(results_dir) / E2E_CACHE_PATH.name
        if from_cache:
            gallery_embeddings, held_out_labels, held_out_emb, ood_emb = (
                load_cached_crops(cache_path)
            )
        else:
            if not source:
                raise SystemExit(
                    "e2e_calibrate needs --source unless --from-cache is set"
                )
            gallery_embeddings, held_out_labels, held_out_emb, ood_emb = (
                collect_real_crops(
                    Path(source).expanduser(),
                    split,
                    n_images,
                    seed,
                    backbone,
                    cache_path,
                )
            )

        global_best = optimize_global(
            gallery_embeddings, held_out_labels, held_out_emb, ood_emb
        )
        print(f"\n[calib] best GLOBAL threshold: {global_best}")

        thresholds, per_class_result = optimize_per_class(
            gallery_embeddings,
            held_out_labels,
            held_out_emb,
            ood_emb,
            init=(global_best["min_similarity"], global_best["margin_min"]),
            rounds=rounds,
        )

        production = (DEFAULT_MIN_SIMILARITY, DEFAULT_MARGIN_MIN)
        _, prod_score, prod_recall, prod_rejection = score_thresholds(
            gallery_embeddings,
            held_out_labels,
            held_out_emb,
            ood_emb,
            {c: production for c in gallery_embeddings},
        )
        beats = per_class_result["both_met"] and per_class_result["score"] > max(
            global_best["score"], prod_score
        )
        return {
            "n_held_out": len(held_out_labels),
            "n_ood": len(ood_emb),
            "production_global": {
                "min_similarity": production[0],
                "margin_min": production[1],
                "recall_gated": round(prod_recall, 3),
                "rejection": round(prod_rejection, 3),
            },
            "best_global": global_best,
            "per_class_thresholds": thresholds,
            "per_class_result": per_class_result,
            "per_class_beats_global": beats,
        }


TASK_REGISTRY = {
    BoxesTask.name: BoxesTask,
    EmbeddingsTask.name: EmbeddingsTask,
    E2EEvalTask.name: E2EEvalTask,
    E2ECalibrateTask.name: E2ECalibrateTask,
}


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("task", choices=sorted(TASK_REGISTRY))
    parser.add_argument(
        "--backbones",
        help="comma-separated backbone names from models.json (embeddings); default all",
    )
    parser.add_argument("--backbone", help="timm backbone id (e2e_*)")
    parser.add_argument("--source", help="path to a YOLO-seg export (e2e_*)")
    parser.add_argument("--split")
    parser.add_argument("--n-images", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--min-similarity", type=float)
    parser.add_argument("--margin-min", type=float)
    parser.add_argument("--rounds", type=int, help="per-class descent rounds")
    parser.add_argument("--iou", type=float, dest="iou_threshold")
    parser.add_argument("--data", help="box_recall data dir (boxes)")
    parser.add_argument("--from-cache", action="store_true", default=None)
    parser.add_argument("--results-dir", default=str(RESULTS_DIR))
    args = vars(parser.parse_args())

    task_name = args.pop("task")
    results_dir = args.pop("results_dir")
    if args.get("backbones"):
        args["backbones"] = [b for b in args["backbones"].split(",") if b]
    kwargs = {k: v for k, v in args.items() if v is not None}
    kwargs["results_dir"] = Path(results_dir)

    result = TASK_REGISTRY[task_name].run(**kwargs)
    report(task_name, result, results_dir)


if __name__ == "__main__":
    main()
