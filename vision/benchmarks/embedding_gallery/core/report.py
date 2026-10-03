"""Terminal output and JSON result files for the benchmark tasks.

Tasks (tasks.py) return plain dicts; this module prints them and writes them to results/.
"""

import json
from datetime import datetime
from pathlib import Path

from embedding_gallery.core.gallery_matcher import UNKNOWN

from core.dataset import (
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


def _pct(num: float, den: float) -> str:
    return f"{num / den * 100 if den else 0.0:.1f}%"


def _thresholds_json(thresholds: dict) -> dict:
    return {
        c: {"min_similarity": s, "margin_min": m} for c, (s, m) in thresholds.items()
    }


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


def print_embeddings(result: dict) -> None:
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
        for r in result["results"]
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
            print(" | ".join(row))


def save_embeddings(result: dict, results_dir: Path) -> Path:
    """Writes benchmark_<ts>.json and, if a backbone passed, thresholds.json."""
    results = result["results"]
    excluded = sorted(KNOWN_LIMITATION_CLASSES)
    out_path = _write_json(
        results_dir / f"benchmark_{_timestamp()}.json",
        {"timestamp": datetime.now().isoformat(), "results": results},
    )
    print(f"\n[report] wrote {out_path}")

    passing = [r for r in results if r["targets_met"]]
    if not passing:
        print(
            "\n[report] GATE NOT MET: no backbone hit recall@1>=90% (excluding "
            f"{excluded}) and unknown-rejection>=80% simultaneously."
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
            "excluded_known_limitation_classes": excluded,
            "note": "Global threshold from Phase 1's grid sweep; tune per object in "
            "gallery/manifest.json. The gate excludes known_limitation_classes (see README.md).",
        },
    )
    print(
        f"[report] GATE PASSED by {winner['backbone']} -> results/thresholds.json "
        f"(recall@1 excludes {excluded}, see README.md)"
    )
    return out_path


def _recall(cases: list[dict], key: str, gated: bool) -> tuple[int, int]:
    """(correct, total) over the cases the proposer localized, optionally gated."""
    localized = [
        c
        for c in cases
        if not c["missed_by_proposer"]
        and not (gated and c["expected"] in KNOWN_LIMITATION_CLASSES)
    ]
    return sum(c[key] == c["expected"] for c in localized), len(localized)


def print_e2e_eval(result: dict) -> None:
    cases, ood_cases = result["cases"], result["ood_cases"]
    total = len(cases)
    missed = sum(c["missed_by_proposer"] for c in cases)
    with_mask = sum(
        not c["missed_by_proposer"] and bool(c.get("had_mask")) for c in cases
    )

    print("\n=== End-to-end results (real box proposer, not ground-truth crops) ===")
    print(f"Total gallery-object instances: {total}")
    if total:
        print(f"Missed by box proposer entirely: {missed} ({_pct(missed, total)})")
    print(f"Localized objects that had a usable mask: {with_mask}/{total - missed}")

    for label, key in (
        ("plain bbox crop", "plain_pred"),
        ("masked crop (background blanked)", "masked_pred"),
    ):
        print(f"\n-- {label} --")
        for text, gated in (
            ("  recall@1 (all classes):  ", False),
            ("  recall@1 (gated, excl. cutlery/kitchenware/cans):", True),
        ):
            correct, n = _recall(cases, key, gated)
            print(f"{text} {_pct(correct, n)}  ({correct}/{n})")

    print(
        "\n=== Unknown-rejection (out-of-gallery objects: "
        f"{sorted(OUT_OF_GALLERY_CLASSES)}) ==="
    )
    if not ood_cases:
        print("Total out-of-gallery instances: 0")
        return
    ood_missed = sum(c["missed_by_proposer"] for c in ood_cases)
    print(f"Total out-of-gallery instances: {len(ood_cases)}")
    print(
        "Missed by proposer (never boxed -> nothing published, not a false "
        f"positive): {ood_missed} ({_pct(ood_missed, len(ood_cases))})"
    )
    localized = [c for c in ood_cases if not c["missed_by_proposer"]]
    for label, key in (
        ("plain bbox crop", "plain_pred"),
        ("masked crop", "masked_pred"),
    ):
        rejected = sum(c[key] == UNKNOWN for c in localized)
        print(
            f"  {label}: unknown-rejection = {_pct(rejected, len(localized))} "
            f"({rejected}/{len(localized)} localized instances)"
        )


def save_e2e_eval(result: dict, results_dir: Path) -> Path:
    keys = ("n_images", "cases", "ood_cases")
    path = _write_json(
        results_dir / f"e2e_eval_{_timestamp()}.json", {k: result[k] for k in keys}
    )
    print(f"\n[e2e] wrote {path}")
    return path


def print_e2e_calibrate(result: dict) -> None:
    prod = result["production_global"]
    print("\n=== Comparison on the SAME real-crop data ===")
    print(f"{'':45s} {'Recall (gated)':>16s} {'Rejection':>12s}")
    thr = f"{prod['min_similarity']}/{prod['margin_min']}"
    print(f"{f'global ({thr}, already in production)':45s} ")
    for label, r in (
        ("  production global threshold", prod),
        ("  best global (this run)", result["best_global"]),
        ("  per-class (greedy)", result["per_class_result"]),
    ):
        print(
            f"{label:45s} {r['recall_gated'] * 100:15.1f}% {r['rejection'] * 100:11.1f}%"
        )


def save_e2e_calibrate(result: dict, results_dir: Path) -> Path:
    """Writes e2e_calibrate_perclass_<ts>.json and, if it wins, e2e_thresholds_perclass.json."""
    thresholds = _thresholds_json(result["per_class_thresholds"])
    path = _write_json(
        results_dir / f"e2e_calibrate_perclass_{_timestamp()}.json",
        {
            "n_held_out": result["n_held_out"],
            "n_ood": result["n_ood"],
            "production_global": result["production_global"],
            "best_global": result["best_global"],
            "per_class_thresholds": thresholds,
            "per_class_result": result["per_class_result"],
        },
    )
    print(f"\n[calib] wrote {path}")

    if result["per_class_beats_global"]:
        _write_json(
            results_dir / "e2e_thresholds_perclass.json", thresholds, sort_keys=True
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


_REPORTERS = {
    "boxes": (print_boxes, save_boxes),
    "embeddings": (print_embeddings, save_embeddings),
    "e2e_eval": (print_e2e_eval, save_e2e_eval),
    "e2e_calibrate": (print_e2e_calibrate, save_e2e_calibrate),
}


def report(task_name: str, result: dict, results_dir: Path = RESULTS_DIR) -> Path:
    """Prints a task's result and writes its JSON file(s). Returns the main path."""
    show, save = _REPORTERS[task_name]
    show(result)
    return save(result, Path(results_dir))
