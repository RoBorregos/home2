#!/usr/bin/env python3
"""Phase 0: box recall of candidate class-agnostic proposers, BEFORE any
embedding-matching code gets built on top of them.

Why this runs first: the likely bottleneck for few-shot recognition is not the
embedding backbone, it's whether the proposer even puts a box around a novel
object in the first place. If nothing here clears the recall bar, stop and
escalate instead of building Phase 1-3 on an unvalidated assumption.

Ground truth format — one `annotations.json` per data dir, pixel-space boxes:
    {
      "image_001.jpg": [{"bbox": [x1, y1, x2, y2], "label": "coke"}, ...],
      "image_002.jpg": [...]
    }

Usage (inside the vision container — needs ultralytics):
    python3 box_recall_eval.py --data data/box_recall --iou 0.5
"""

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[2]
        / "packages"
        / "object_detector_2d"
        / "scripts"
    ),
)


def iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def load_annotations(data_dir: Path) -> dict:
    ann_path = data_dir / "annotations.json"
    if not ann_path.exists():
        raise SystemExit(
            f"No {ann_path} found. Box recall needs a hand-labeled validation set "
            "(see this script's docstring for the format) before it can measure anything — "
            "there is no synthetic substitute for real object photos here."
        )
    return json.loads(ann_path.read_text())


def evaluate_proposer(
    propose_fn, data_dir: Path, annotations: dict, iou_thresh: float
) -> dict:
    import cv2

    total_gt = 0
    matched_gt = 0
    total_pred = 0
    total_matched_pred = 0
    latencies_ms = []

    for image_name, gt_boxes in annotations.items():
        image = cv2.imread(str(data_dir / image_name))
        if image is None:
            print(f"[box_recall] WARNING: could not read {image_name}, skipping")
            continue

        t0 = time.perf_counter()
        pred_boxes = propose_fn(image)  # list of [x1, y1, x2, y2] pixel boxes
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
    # false positives per image, not per box — the volume a downstream embedding
    # matcher would actually have to reject as "unknown"
    fp_per_image = (total_pred - total_matched_pred) / max(1, len(annotations))
    avg_latency_ms = sum(latencies_ms) / len(latencies_ms) if latencies_ms else 0.0

    return {
        "recall_at_iou": round(recall, 3),
        "gt_boxes": total_gt,
        "matched_gt": matched_gt,
        "fp_per_image": round(fp_per_image, 2),
        "avg_latency_ms": round(avg_latency_ms, 1),
    }


def make_yolo_agnostic_proposer(filename: str, conf: float):
    from detectors.registry import MODELS_PATH
    from ultralytics import YOLO

    model = YOLO(MODELS_PATH + filename)

    def propose(image):
        results = model.predict(image, conf=conf, agnostic_nms=True, verbose=False)
        boxes = []
        for out in results:
            if out.boxes is None:
                continue
            for box in out.boxes:
                boxes.append([round(v) for v in box.xyxy[0].tolist()])
        return boxes

    return propose


def make_yoloe_proposer(filename: str, conf: float, prompt_classes: list[str] | None):
    from detectors.registry import MODELS_PATH
    from ultralytics import YOLOE

    model = YOLOE(MODELS_PATH + filename)
    if prompt_classes:
        model.set_classes(prompt_classes, model.get_text_pe(prompt_classes))

    def propose(image):
        results = model.predict(image, conf=conf, verbose=False)
        boxes = []
        for out in results:
            if out.boxes is None:
                continue
            for box in out.boxes:
                boxes.append([round(v) for v in box.xyxy[0].tolist()])
        return boxes

    return propose


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--data", default="data/box_recall")
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--models", default=str(Path(__file__).parent / "models.json"))
    parser.add_argument("--results", default="results/box_recall.json")
    args = parser.parse_args()

    data_dir = Path(args.data)
    annotations = load_annotations(data_dir)
    candidates = json.loads(Path(args.models).read_text())["box_proposers"]

    report = {}
    for cfg in candidates:
        print(f"[box_recall] evaluating {cfg['name']} ...")
        try:
            if cfg["type"] == "yolo":
                propose_fn = make_yolo_agnostic_proposer(cfg["filename"], cfg["conf"])
            elif cfg["type"] == "yolo_e":
                propose_fn = make_yoloe_proposer(
                    cfg["filename"], cfg["conf"], cfg.get("prompt_classes")
                )
            else:
                print(f"[box_recall] unknown proposer type {cfg['type']!r}, skipping")
                continue
        except Exception as e:
            print(f"[box_recall] SKIP {cfg['name']}: {e}")
            continue

        metrics = evaluate_proposer(propose_fn, data_dir, annotations, args.iou)
        report[cfg["name"]] = metrics
        print(f"[box_recall] {cfg['name']}: {metrics}")

    Path(args.results).parent.mkdir(parents=True, exist_ok=True)
    Path(args.results).write_text(json.dumps(report, indent=2) + "\n")
    print(f"\n[box_recall] wrote {args.results}")

    winners = [n for n, m in report.items() if m["recall_at_iou"] >= 0.9]
    if winners:
        print(f"[box_recall] GATE PASSED: {winners} clear >=90% recall@IoU{args.iou}")
    else:
        print(
            "[box_recall] GATE NOT MET: no candidate reached >=90% recall. "
            "Per the plan, do not proceed to Phase 1 — escalate box-proposer choice first."
        )


if __name__ == "__main__":
    main()
