"""Phase 1 metrics: recall@1, hard-negative precision, unknown-rejection rate.

Each task takes a loaded Gallery (vision/.../detectors/gallery_matcher.py) and
a directory of pre-cropped, pre-labeled test images, and returns a metrics
dict plus the per-case detail report.py needs to print failures.

Dataset format for held_out/ and hard_negatives/ (same shape, different
curation intent — see this benchmark's README.md):
    annotations.json: {"image_001.jpg": "coke", "image_002.jpg": "mug", ...}

Dataset format for out_of_gallery/: any images NOT of a gallery object; no
per-image label needed, just "this must come back unknown".
"""

import json
from pathlib import Path

import numpy as np
from gallery_matcher import UNKNOWN, Gallery


def _load_labeled_dir(data_dir: Path) -> dict[str, str]:
    ann_path = data_dir / "annotations.json"
    if not ann_path.exists():
        raise SystemExit(
            f"No {ann_path} — see this benchmark's README.md for the expected format."
        )
    return json.loads(ann_path.read_text())


def _embed_dir(backbone, data_dir: Path, filenames: list[str]) -> np.ndarray:
    from PIL import Image

    crops = [Image.open(data_dir / name).convert("RGB") for name in filenames]
    return backbone.embed_batch(crops)


def recall_at_1(gallery: Gallery, backbone, data_dir: Path) -> dict:
    """Of crops of objects that ARE in the gallery, how often does top-1 match
    the true label (unknown counts as a miss — it's a false rejection here)."""
    labels_by_file = _load_labeled_dir(data_dir)
    filenames = list(labels_by_file.keys())
    embeddings = _embed_dir(backbone, data_dir, filenames)
    predictions = gallery.match_batch(embeddings)

    cases = []
    correct = 0
    for filename, (pred_label, sim, margin) in zip(filenames, predictions):
        expected = labels_by_file[filename]
        passed = pred_label == expected
        correct += int(passed)
        cases.append(
            {
                "input": filename,
                "expected": expected,
                "got": pred_label,
                "similarity": round(sim, 3),
                "margin": round(margin, 3),
                "passed": passed,
            }
        )

    total = len(filenames)
    return {
        "recall_at_1": round(correct / total, 3) if total else 0.0,
        "total": total,
        "cases": cases,
    }


def hard_negative_precision(gallery: Gallery, backbone, data_dir: Path) -> dict:
    """Of crops of visually-similar-but-different objects (e.g. fork vs spoon),
    how often does the matcher NOT confuse one for the other. Landing on
    "unknown" is a softer, acceptable failure here — the thing we're
    specifically guarding against is a confident WRONG label."""
    labels_by_file = _load_labeled_dir(data_dir)
    filenames = list(labels_by_file.keys())
    embeddings = _embed_dir(backbone, data_dir, filenames)
    predictions = gallery.match_batch(embeddings)

    cases = []
    not_confused = 0
    for filename, (pred_label, sim, margin) in zip(filenames, predictions):
        expected = labels_by_file[filename]
        confused = pred_label != UNKNOWN and pred_label != expected
        passed = not confused
        not_confused += int(passed)
        cases.append(
            {
                "input": filename,
                "expected": expected,
                "got": pred_label,
                "similarity": round(sim, 3),
                "margin": round(margin, 3),
                "passed": passed,
            }
        )

    total = len(filenames)
    return {
        "precision": round(not_confused / total, 3) if total else 0.0,
        "total": total,
        "cases": cases,
    }


def unknown_rejection_rate(gallery: Gallery, backbone, data_dir: Path) -> dict:
    """Of crops of objects NOT in the gallery, how often is the result unknown."""
    filenames = sorted(
        p.name
        for p in data_dir.iterdir()
        if p.suffix.lower() in (".jpg", ".jpeg", ".png")
    )
    if not filenames:
        raise SystemExit(
            f"No images in {data_dir} — need out-of-gallery crops to measure rejection."
        )
    embeddings = _embed_dir(backbone, data_dir, filenames)
    predictions = gallery.match_batch(embeddings)

    cases = []
    rejected = 0
    for filename, (pred_label, sim, margin) in zip(filenames, predictions):
        passed = pred_label == UNKNOWN
        rejected += int(passed)
        cases.append(
            {
                "input": filename,
                "expected": UNKNOWN,
                "got": pred_label,
                "similarity": round(sim, 3),
                "margin": round(margin, 3),
                "passed": passed,
            }
        )

    total = len(filenames)
    return {
        "unknown_rejection_rate": round(rejected / total, 3) if total else 0.0,
        "total": total,
        "cases": cases,
    }


TASK_REGISTRY = {
    "recall_at_1": recall_at_1,
    "hard_negative_precision": hard_negative_precision,
    "unknown_rejection_rate": unknown_rejection_rate,
}
