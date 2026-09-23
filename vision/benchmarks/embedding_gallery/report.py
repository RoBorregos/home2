#!/usr/bin/env python3
"""Phase 1 driver: embed data/gallery_photos/ + data/held_out/ +
data/hard_negatives/ + data/out_of_gallery/ ONCE per backbone, then sweep a
global (min_similarity, margin_min) threshold on the cached embeddings
(cheap: pure numpy) to jointly hit both acceptance targets. Writes
results/thresholds.json for gallery_build.py to seed from.

Usage (inside the vision container, or this benchmark's .venv — needs
timm/torch, clip for CLIP candidates, PIL):
    python3 report.py --backbones dinov2_vits14 --backbones clip_vit_b32
    python3 report.py                      # all backbones in models.json

Requires data/gallery_photos/<object>/*.jpg (enrollment set — must NOT overlap
with data/held_out/, or recall@1 measures memorization, not matching) plus
data/held_out/, data/hard_negatives/, data/out_of_gallery/ per README.md.
"""

import argparse
import glob
import itertools
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

try:
    from rich.console import Console
    from rich.table import Table
    from rich import box as rich_box

    _RICH = True
except ImportError:
    _RICH = False

HERE = Path(__file__).parent
DATA_DIR = HERE / "data"
RECALL_TARGET = 0.90
REJECTION_TARGET = 0.80

# Coarse grid on cached embeddings — cheap, widen freely.
SIM_GRID = [round(v, 2) for v in np.arange(0.10, 0.95, 0.05)]
MARGIN_GRID = [round(v, 2) for v in np.arange(0.00, 0.25, 0.02)]


def _load_images(paths: list[Path]):
    from PIL import Image

    return [Image.open(p).convert("RGB") for p in paths]


def embed_gallery_photos(backbone) -> dict[str, np.ndarray]:
    embeddings = {}
    for obj_dir in sorted((DATA_DIR / "gallery_photos").iterdir()):
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
    """Returns (filenames, labels, embeddings[N,D])."""
    ann_path = data_dir / "annotations.json"
    if not ann_path.exists():
        raise SystemExit(f"No {ann_path} — see README.md for the expected format.")
    labels_by_file = json.loads(ann_path.read_text())
    filenames = list(labels_by_file.keys())
    embeddings = backbone.embed_batch(_load_images([data_dir / f for f in filenames]))
    labels = [labels_by_file[f] for f in filenames]
    return filenames, labels, embeddings


def embed_unlabeled_dir(backbone, data_dir: Path) -> tuple[list[str], np.ndarray]:
    filenames = sorted(
        p.name
        for p in data_dir.iterdir()
        if p.suffix.lower() in (".jpg", ".jpeg", ".png")
    )
    if not filenames:
        raise SystemExit(
            f"No images in {data_dir} — need out-of-gallery crops to measure rejection."
        )
    embeddings = backbone.embed_batch(_load_images([data_dir / f for f in filenames]))
    return filenames, embeddings


def cases_from_predictions(
    filenames, expected_labels, predictions, pass_fn
) -> list[dict]:
    cases = []
    for filename, expected, (pred_label, sim, margin) in zip(
        filenames, expected_labels, predictions
    ):
        cases.append(
            {
                "input": filename,
                "expected": expected,
                "got": pred_label,
                "similarity": round(sim, 3),
                "margin": round(margin, 3),
                "passed": pass_fn(expected, pred_label),
            }
        )
    return cases


def run_backbone(backbone_name: str, backbone_id: str) -> dict:
    print(
        f"\n[report] backbone={backbone_name} ({backbone_id}) — embedding all crops once..."
    )
    backbone = EmbeddingBackbone(backbone_id).load()

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
        f"[report] embedded: {sum(len(v) for v in gallery_embeddings.values())} gallery, "
        f"{len(held_out_files)} held_out, {len(hard_neg_files)} hard_negatives, {len(ood_files)} out_of_gallery"
    )

    def build_gallery(min_sim: float, margin_min: float) -> Gallery:
        manifest = {
            "objects": {
                name: {"min_similarity": min_sim, "margin_min": margin_min}
                for name in gallery_embeddings
            }
        }
        return Gallery(gallery_embeddings, manifest)

    print(
        f"[report] sweeping {len(SIM_GRID) * len(MARGIN_GRID)} threshold combinations..."
    )
    best = None
    for min_sim, margin_min in itertools.product(SIM_GRID, MARGIN_GRID):
        gallery = build_gallery(min_sim, margin_min)
        recall = sum(
            1
            for label, (pred, _, _) in zip(
                held_out_labels, gallery.match_batch(held_out_emb)
            )
            if pred == label
        ) / len(held_out_labels)
        rejection = sum(
            1 for (pred, _, _) in gallery.match_batch(ood_emb) if pred == UNKNOWN
        ) / len(ood_files)
        both_met = recall >= RECALL_TARGET and rejection >= REJECTION_TARGET
        score = min(recall, rejection)
        candidate = {
            "min_similarity": min_sim,
            "margin_min": margin_min,
            "recall_at_1": round(recall, 3),
            "unknown_rejection_rate": round(rejection, 3),
            "both_targets_met": both_met,
            "score": score,
        }
        if best is None or (candidate["both_targets_met"], candidate["score"]) > (
            best["both_targets_met"],
            best["score"],
        ):
            best = candidate

    print(f"[report] best threshold: {best}")

    gallery = build_gallery(best["min_similarity"], best["margin_min"])
    recall_preds = gallery.match_batch(held_out_emb)
    hard_neg_preds = gallery.match_batch(hard_neg_emb)
    ood_preds = gallery.match_batch(ood_emb)

    recall_cases = cases_from_predictions(
        held_out_files, held_out_labels, recall_preds, lambda e, p: p == e
    )
    hard_neg_cases = cases_from_predictions(
        hard_neg_files,
        hard_neg_labels,
        hard_neg_preds,
        lambda e, p: p == UNKNOWN or p == e,
    )
    ood_cases = cases_from_predictions(
        ood_files, [UNKNOWN] * len(ood_files), ood_preds, lambda e, p: p == UNKNOWN
    )

    recall_at_1 = (
        sum(c["passed"] for c in recall_cases) / len(recall_cases)
        if recall_cases
        else 0.0
    )
    hard_neg_precision = (
        sum(c["passed"] for c in hard_neg_cases) / len(hard_neg_cases)
        if hard_neg_cases
        else 0.0
    )
    unknown_rejection = (
        sum(c["passed"] for c in ood_cases) / len(ood_cases) if ood_cases else 0.0
    )

    return {
        "backbone": backbone_name,
        "backbone_id": backbone_id,
        "threshold": {
            "min_similarity": best["min_similarity"],
            "margin_min": best["margin_min"],
        },
        "recall_at_1": {
            "recall_at_1": round(recall_at_1, 3),
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
        "targets_met": recall_at_1 >= RECALL_TARGET
        and unknown_rejection >= REJECTION_TARGET,
    }


def print_table(results: list[dict]) -> None:
    rows = [
        (
            r["backbone"],
            f"{r['recall_at_1']['recall_at_1'] * 100:.0f}%",
            f"{r['hard_negative_precision']['precision'] * 100:.0f}%",
            f"{r['unknown_rejection_rate']['unknown_rejection_rate'] * 100:.0f}%",
            f"{r['threshold']['min_similarity']}/{r['threshold']['margin_min']}",
            "PASS" if r["targets_met"] else "FAIL",
        )
        for r in results
    ]
    headers = [
        "Backbone",
        "Recall@1",
        "Hard-neg precision",
        "Unknown rejection",
        "min_sim/margin",
        "Gate",
    ]
    if _RICH:
        console = Console()
        t = Table(title="Phase 1 — backbone comparison", box=rich_box.ROUNDED)
        for h in headers:
            t.add_column(h)
        for row in rows:
            style = "green" if row[-1] == "PASS" else "red"
            t.add_row(*row[:-1], f"[{style}]{row[-1]}[/{style}]")
        console.print(t)
    else:
        print("\n" + " | ".join(headers))
        for row in rows:
            print(" | ".join(str(c) for c in row))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--backbones",
        action="append",
        help="backbone name(s) from models.json; default: all",
    )
    parser.add_argument("--models", default=str(HERE / "models.json"))
    parser.add_argument("--results-dir", default=str(HERE / "results"))
    args = parser.parse_args()

    all_backbones = json.loads(Path(args.models).read_text())["backbones"]
    selected = (
        all_backbones
        if not args.backbones
        else [b for b in all_backbones if b["name"] in args.backbones]
    )
    if not selected:
        raise SystemExit(f"No matching backbones in {args.models} for {args.backbones}")

    results = [run_backbone(b["name"], b["id"]) for b in selected]
    print_table(results)

    results_dir = Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = results_dir / f"benchmark_{ts}.json"
    out_path.write_text(
        json.dumps(
            {"timestamp": datetime.now().isoformat(), "results": results}, indent=2
        )
        + "\n"
    )
    print(f"\n[report] wrote {out_path}")

    passing = [r for r in results if r["targets_met"]]
    if passing:
        winner = max(passing, key=lambda r: r["recall_at_1"]["recall_at_1"])
        (results_dir / "thresholds.json").write_text(
            json.dumps(
                {
                    "chosen_backbone": winner["backbone"],
                    "chosen_backbone_id": winner["backbone_id"],
                    "min_similarity": winner["threshold"]["min_similarity"],
                    "margin_min": winner["threshold"]["margin_min"],
                    "note": "Global threshold from Phase 1's grid sweep — tune per-object in "
                    "gallery/manifest.json if a specific object needs a different floor.",
                },
                indent=2,
            )
            + "\n"
        )
        print(
            f"[report] GATE PASSED by {winner['backbone']} -> results/thresholds.json"
        )
    else:
        print(
            "\n[report] GATE NOT MET: no backbone hit recall@1>=90% and "
            "unknown-rejection>=80% simultaneously. Per the plan, do not write "
            "registry.py code yet — consider Phase 4 (fine-tune) instead."
        )


if __name__ == "__main__":
    main()
