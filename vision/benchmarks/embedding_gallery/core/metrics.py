"""Scoring and threshold search shared by the tasks and experiments.

IoU, gated recall, unknown-rejection and the global / per-class (min_similarity, margin_min) searches.
"""

import itertools
from collections.abc import Iterable

import numpy as np
from embedding_gallery.core.gallery_matcher import UNKNOWN, Gallery

from core.dataset import (
    KNOWN_LIMITATION_CLASSES,
    MARGIN_GRID,
    RECALL_TARGET,
    REJECTION_TARGET,
    SIM_GRID,
)

# First per-class run picked degenerate floors (min_similarity=0.1,
# margin_min=0.0), overfit to the calibration sample; floor the search.
MIN_SIMILARITY_FLOOR = 0.3
MARGIN_MIN_FLOOR = 0.02
PER_CLASS_SIM_GRID = [v for v in SIM_GRID if v >= MIN_SIMILARITY_FLOOR]
PER_CLASS_MARGIN_GRID = [v for v in MARGIN_GRID if v >= MARGIN_MIN_FLOOR]


def iou(a: Iterable[float], b: Iterable[float]) -> float:
    """Intersection-over-union of two [x1, y1, x2, y2] pixel boxes."""
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


def match_gt_to_boxes(gt_items, kept_px, used: set | None = None, iou_threshold=0.5):
    """Greedy best-IoU match of (label, gt_bbox) pairs to boxes; yields (label, gt_bbox, idx or None).

    `used` excludes boxes already matched in this call; pass None so unrelated calls don't compete.
    """
    for true_label, gt_bbox in gt_items:
        best_iou, best_idx = 0.0, -1
        for idx, pred_bbox in enumerate(kept_px):
            if used is not None and idx in used:
                continue
            score = iou(gt_bbox, pred_bbox)
            if score > best_iou:
                best_iou, best_idx = score, idx
        if best_iou < iou_threshold:
            yield true_label, gt_bbox, None
            continue
        if used is not None:
            used.add(best_idx)
        yield true_label, gt_bbox, best_idx


def gated_recall(labels: list[str], predictions: list[tuple], excluded=None) -> float:
    """recall@1 excluding `excluded` (default KNOWN_LIMITATION_CLASSES).

    This is the metric the gate actually checks. Returns 1.0 (vacuously) if
    every case is excluded.
    """
    excluded = KNOWN_LIMITATION_CLASSES if excluded is None else excluded
    kept = [
        (label, pred)
        for label, (pred, _, _) in zip(labels, predictions)
        if label not in excluded
    ]
    if not kept:
        return 1.0
    return sum(1 for label, pred in kept if pred == label) / len(kept)


def rejection_rate(predictions: list[tuple]) -> float:
    """Fraction of out-of-gallery predictions labeled UNKNOWN (1.0 if empty)."""
    if not predictions:
        return 1.0
    return sum(1 for (pred, _, _) in predictions if pred == UNKNOWN) / len(predictions)


def build_gallery(
    gallery_embeddings: dict[str, np.ndarray], thresholds: dict
) -> Gallery:
    """Gallery with per-class thresholds: {class: (min_similarity, margin_min)}."""
    manifest = {
        "objects": {
            name: {"min_similarity": s, "margin_min": m}
            for name, (s, m) in thresholds.items()
        }
    }
    return Gallery(gallery_embeddings, manifest)


def score_thresholds(
    gallery_embeddings,
    held_out_labels,
    held_out_emb,
    ood_emb,
    thresholds,
    *,
    excluded=None,
    recall_target: float = RECALL_TARGET,
    rejection_target: float = REJECTION_TARGET,
) -> tuple:
    """Returns (both_targets_met, min(recall, rejection), recall, rejection)."""
    gallery = build_gallery(gallery_embeddings, thresholds)
    recall = gated_recall(held_out_labels, gallery.match_batch(held_out_emb), excluded)
    rejection = rejection_rate(gallery.match_batch(ood_emb))
    both_met = recall >= recall_target and rejection >= rejection_target
    return both_met, min(recall, rejection), recall, rejection


def optimize_global(
    gallery_embeddings,
    held_out_labels,
    held_out_emb,
    ood_emb,
    *,
    excluded=None,
    recall_target: float = RECALL_TARGET,
    rejection_target: float = REJECTION_TARGET,
    verbose: bool = True,
) -> dict:
    """Sweeps one (min_similarity, margin_min) pair over SIM_GRID x MARGIN_GRID.

    Returns the best candidate dict (min_similarity, margin_min, recall_gated, rejection, both_met, score).
    """
    if verbose:
        print(
            f"[calib] sweeping {len(SIM_GRID) * len(MARGIN_GRID)} GLOBAL "
            "threshold combinations..."
        )
    best = None
    for min_sim, margin_min in itertools.product(SIM_GRID, MARGIN_GRID):
        thresholds = {name: (min_sim, margin_min) for name in gallery_embeddings}
        both_met, score, recall, rejection = score_thresholds(
            gallery_embeddings,
            held_out_labels,
            held_out_emb,
            ood_emb,
            thresholds,
            excluded=excluded,
            recall_target=recall_target,
            rejection_target=rejection_target,
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
    """Greedy coordinate descent of per-class thresholds, starting from `init`.

    Returns (thresholds {class: (min_similarity, margin_min)}, dict with both_met, score, recall_gated, rejection).
    """

    def score(thresholds):
        return score_thresholds(
            gallery_embeddings, held_out_labels, held_out_emb, ood_emb, thresholds
        )

    classes = sorted(gallery_embeddings)
    thresholds = {c: init for c in classes}
    _, best_score, recall, rejection = score(thresholds)
    print(
        f"[calib] per-class start (all @ {init}): score={best_score:.3f} "
        f"recall={recall:.3f} rejection={rejection:.3f}"
    )

    for rnd in range(rounds):
        improved = False
        for c in classes:
            best_local = (best_score, thresholds[c])
            for s, m in itertools.product(PER_CLASS_SIM_GRID, PER_CLASS_MARGIN_GRID):
                trial = dict(thresholds)
                trial[c] = (s, m)
                _, sc, _, _ = score(trial)
                if sc > best_local[0]:
                    best_local = (sc, (s, m))
            if best_local[1] != thresholds[c]:
                thresholds[c] = best_local[1]
                improved = True
        _, best_score, recall, rejection = score(thresholds)
        print(
            f"[calib] round {rnd}: score={best_score:.3f} "
            f"recall={recall:.3f} rejection={rejection:.3f}"
        )
        if not improved:
            print("[calib] converged (no class improved this round)")
            break

    both_met, best_score, recall, rejection = score(thresholds)
    return thresholds, {
        "both_met": both_met,
        "score": best_score,
        "recall_gated": round(recall, 3),
        "rejection": round(rejection, 3),
    }
