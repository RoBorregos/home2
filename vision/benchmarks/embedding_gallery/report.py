"""Terminal output and JSON result files for the benchmark tasks.

Tasks (tasks.py) return plain dicts; this module prints them and writes them to results/.
"""

import json
from datetime import datetime
from pathlib import Path

from embedding_gallery.gallery_matcher import UNKNOWN

from lib.dataset import (
    KNOWN_LIMITATION_CLASSES,
    OUT_OF_GALLERY_CLASSES,
    RESULTS_DIR,
)

try:
    from rich import box as rich_box
    from rich.console import Console
    from rich.table import Table

    _RICH = True
except ImportError:
    _RICH = False


def _timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _write_json(path: Path, payload, **dump_kwargs) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, **dump_kwargs) + "\n")
    return path


# ---------------------------------------------------------------- boxes


def print_boxes(result: dict) -> None:
    winners = [n for n, m in result["proposers"].items() if m["recall_at_iou"] >= 0.9]
    if winners:
        print(
            f"[box_recall] GATE PASSED: {winners} clear >=90% recall@IoU{result['iou']}"
        )
    else:
        print(
            "[box_recall] GATE NOT MET: no candidate reached >=90% recall. "
            "Per the plan, do not proceed to Phase 1, escalate box-proposer "
            "choice first."
        )


def save_boxes(result: dict, results_dir: Path) -> Path:
    path = _write_json(results_dir / "box_recall.json", result["proposers"])
    print(f"\n[box_recall] wrote {path}")
    return path


# ----------------------------------------------------------- embeddings


def print_embeddings(result: dict) -> None:
    results = result["results"]
    rows = [
        (
            r["backbone"],
            f"{r['recall_at_1']['recall_at_1'] * 100:.0f}%",
            f"{r['recall_at_1']['recall_at_1_gated'] * 100:.0f}%",
            f"{r['hard_negative_precision']['precision'] * 100:.0f}%",
            f"{r['unknown_rejection_rate']['unknown_rejection_rate'] * 100:.0f}%",
            f"{r['threshold']['min_similarity']}/{r['threshold']['margin_min']}",
            "PASS" if r["targets_met"] else "FAIL",
        )
        for r in results
    ]
    headers = [
        "Backbone",
        "Recall@1 (all)",
        "Recall@1 (gated)",
        "Hard-neg precision",
        "Unknown rejection",
        "min_sim/margin",
        "Gate",
    ]
    if _RICH:
        table = Table(title="Phase 1: backbone comparison", box=rich_box.ROUNDED)
        for h in headers:
            table.add_column(h)
        for row in rows:
            style = "green" if row[-1] == "PASS" else "red"
            table.add_row(*row[:-1], f"[{style}]{row[-1]}[/{style}]")
        Console().print(table)
    else:
        print("\n" + " | ".join(headers))
        for row in rows:
            print(" | ".join(str(c) for c in row))


def save_embeddings(result: dict, results_dir: Path) -> Path:
    """Writes benchmark_<ts>.json and, if a backbone passed, thresholds.json."""
    results = result["results"]
    out_path = _write_json(
        results_dir / f"benchmark_{_timestamp()}.json",
        {"timestamp": datetime.now().isoformat(), "results": results},
    )
    print(f"\n[report] wrote {out_path}")

    passing = [r for r in results if r["targets_met"]]
    if not passing:
        print(
            "\n[report] GATE NOT MET: no backbone hit recall@1>=90% (excluding "
            f"{sorted(KNOWN_LIMITATION_CLASSES)}) and unknown-rejection>=80% "
            "simultaneously."
        )
        return out_path

    winner = max(passing, key=lambda r: r["recall_at_1"]["recall_at_1_gated"])
    _write_json(
        results_dir / "thresholds.json",
        {
            "chosen_backbone": winner["backbone"],
            "chosen_backbone_id": winner["backbone_id"],
            "min_similarity": winner["threshold"]["min_similarity"],
            "margin_min": winner["threshold"]["margin_min"],
            "excluded_known_limitation_classes": sorted(KNOWN_LIMITATION_CLASSES),
            "note": "Global threshold from Phase 1's grid sweep, tune per-object "
            "in gallery/manifest.json if a specific object needs a different "
            "floor. Gate excludes KNOWN_LIMITATION_CLASSES (already covered by "
            "yolo_finetuned): recall@1 on those stays below target across every "
            "backbone/fine-tune tried (see README.md).",
        },
    )
    print(
        f"[report] GATE PASSED by {winner['backbone']} -> results/thresholds.json "
        f"(recall@1 excludes {sorted(KNOWN_LIMITATION_CLASSES)}, see README.md)"
    )
    return out_path


# ----------------------------------------------------------- e2e_eval


def _summarize(cases: list[dict], pred_key: str, gated: bool) -> dict:
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


def print_e2e_eval(result: dict) -> None:
    cases, ood_cases = result["cases"], result["ood_cases"]
    total_instances = len(cases)
    missed = sum(1 for c in cases if c["missed_by_proposer"])
    with_mask = sum(
        1 for c in cases if not c["missed_by_proposer"] and c.get("had_mask")
    )

    print("\n=== End-to-end results (real box proposer, not ground-truth crops) ===")
    print(f"Total gallery-object instances: {total_instances}")
    if total_instances:
        print(
            f"Missed by box proposer entirely: {missed} "
            f"({missed / total_instances * 100:.1f}%)"
        )
    print(
        "Localized objects that had a usable mask: "
        f"{with_mask}/{total_instances - missed}"
    )

    for label, key in [
        ("plain bbox crop", "plain_pred"),
        ("masked crop (background blanked)", "masked_pred"),
    ]:
        all_r = _summarize(cases, key, gated=False)
        gated_r = _summarize(cases, key, gated=True)
        print(f"\n-- {label} --")
        print(
            f"  recall@1 (all classes):   {all_r['recall'] * 100:.1f}%  "
            f"({all_r['correct']}/{all_r['total']})"
        )
        print(
            "  recall@1 (gated, excl. cutlery/kitchenware/cans): "
            f"{gated_r['recall'] * 100:.1f}%  ({gated_r['correct']}/{gated_r['total']})"
        )

    print(
        "\n=== Unknown-rejection (out-of-gallery objects: "
        f"{sorted(OUT_OF_GALLERY_CLASSES)}) ==="
    )
    ood_total = len(ood_cases)
    if not ood_total:
        print("Total out-of-gallery instances: 0")
        return
    ood_missed = sum(1 for c in ood_cases if c["missed_by_proposer"])
    print(f"Total out-of-gallery instances: {ood_total}")
    print(
        "Missed by proposer (never boxed -> nothing published, not a false "
        f"positive): {ood_missed} ({ood_missed / ood_total * 100:.1f}%)"
    )
    localized_ood = [c for c in ood_cases if not c["missed_by_proposer"]]
    for label, key in [
        ("plain bbox crop", "plain_pred"),
        ("masked crop", "masked_pred"),
    ]:
        rejected = sum(1 for c in localized_ood if c[key] == UNKNOWN)
        n = len(localized_ood)
        rate = rejected / n if n else 0.0
        print(
            f"  {label}: unknown-rejection = {rate * 100:.1f}% "
            f"({rejected}/{n} localized instances)"
        )


def save_e2e_eval(result: dict, results_dir: Path) -> Path:
    path = _write_json(
        results_dir / f"e2e_eval_{_timestamp()}.json",
        {
            "n_images": result["n_images"],
            "cases": result["cases"],
            "ood_cases": result["ood_cases"],
        },
    )
    print(f"\n[e2e] wrote {path}")
    return path


# ------------------------------------------------------- e2e_calibrate


def print_e2e_calibrate(result: dict) -> None:
    prod, best = result["production_global"], result["best_global"]
    per_class = result["per_class_result"]
    print("\n=== Comparison on the SAME real-crop data ===")
    print(f"{'':45s} {'Recall (gated)':>16s} {'Rejection':>12s}")
    print(
        f"{'global (' + str(prod['min_similarity']) + '/' + str(prod['margin_min']) + ', already in production)':45s} "
    )
    print(
        f"{'  production global threshold':45s} "
        f"{prod['recall_gated'] * 100:15.1f}% {prod['rejection'] * 100:11.1f}%"
    )
    print(
        f"{'  best global (this run)':45s} "
        f"{best['recall_gated'] * 100:15.1f}% {best['rejection'] * 100:11.1f}%"
    )
    print(
        f"{'  per-class (greedy)':45s} "
        f"{per_class['recall_gated'] * 100:15.1f}% {per_class['rejection'] * 100:11.1f}%"
    )


def save_e2e_calibrate(result: dict, results_dir: Path) -> Path:
    """Writes e2e_calibrate_perclass_<ts>.json and, if it wins, e2e_thresholds_perclass.json."""
    thresholds = result["per_class_thresholds"]
    path = _write_json(
        results_dir / f"e2e_calibrate_perclass_{_timestamp()}.json",
        {
            "n_held_out": result["n_held_out"],
            "n_ood": result["n_ood"],
            "production_global": result["production_global"],
            "best_global": result["best_global"],
            "per_class_thresholds": {
                c: {"min_similarity": s, "margin_min": m}
                for c, (s, m) in thresholds.items()
            },
            "per_class_result": result["per_class_result"],
        },
    )
    print(f"\n[calib] wrote {path}")

    if result["per_class_beats_global"]:
        _write_json(
            results_dir / "e2e_thresholds_perclass.json",
            {
                c: {"min_similarity": s, "margin_min": m}
                for c, (s, m) in thresholds.items()
            },
            sort_keys=True,
        )
        print(
            "[calib] per-class thresholds beat the global default -> "
            "results/e2e_thresholds_perclass.json"
        )
    else:
        print(
            "[calib] per-class optimization did not clearly beat the global "
            "threshold on this data."
        )
    return path


PRINTERS = {
    "boxes": print_boxes,
    "embeddings": print_embeddings,
    "e2e_eval": print_e2e_eval,
    "e2e_calibrate": print_e2e_calibrate,
}
SAVERS = {
    "boxes": save_boxes,
    "embeddings": save_embeddings,
    "e2e_eval": save_e2e_eval,
    "e2e_calibrate": save_e2e_calibrate,
}


def report(task_name: str, result: dict, results_dir: Path = RESULTS_DIR) -> Path:
    """Prints a task's result and writes its JSON file(s). Returns the main path."""
    PRINTERS[task_name](result)
    return SAVERS[task_name](result, Path(results_dir))
