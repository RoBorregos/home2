"""Terminal output and JSON report writer for STT benchmarks."""

import json
import os
from datetime import datetime
from typing import Any

try:
    from rich.console import Console
    from rich.table import Table
    from rich import box

    _RICH = True
except ImportError:
    _RICH = False

_force_plain = os.environ.get("STT_FORCE_PLAIN", "") == "1"


def print_model_table(model: str, task_results: dict[str, dict]) -> None:
    if _RICH and not _force_plain:
        _print_rich(model, task_results)
    else:
        _print_plain(model, task_results)


def _print_rich(model: str, task_results: dict) -> None:
    console = Console()
    t = Table(title=f"Model: {model}", box=box.ROUNDED, show_lines=False)
    t.add_column("Task", style="cyan", no_wrap=True)
    t.add_column("Cases", justify="right")
    t.add_column("Accuracy", justify="right")
    t.add_column("Avg WER", justify="right")
    t.add_column("Avg RTF", justify="right")
    t.add_column("Avg Latency (s)", justify="right")

    for task_name, r in task_results.items():
        cases = r.get("cases", [])
        passed = sum(1 for c in cases if c["passed"])
        total = len(cases)
        pct = (passed / total * 100) if total else 0
        acc_color = "green" if pct >= 80 else ("yellow" if pct >= 60 else "red")

        wer = r.get("avg_wer")
        rtf = r.get("avg_rtf")
        lat = r.get("avg_latency_s")

        t.add_row(
            task_name,
            str(total),
            f"[{acc_color}]{passed}/{total} ({pct:.0f}%)[/{acc_color}]",
            f"{wer:.2%}" if wer is not None else "—",
            f"{rtf:.4f}" if rtf is not None else "—",
            f"{lat:.3f}" if lat is not None else "—",
        )

    console.print(t)
    _print_failures_rich(console, task_results)


def _print_failures_rich(console, task_results: dict) -> None:
    from rich.panel import Panel

    for task_name, r in task_results.items():
        failures = [c for c in r.get("cases", []) if not c["passed"]]
        if not failures:
            continue
        lines = []
        for f in failures[:5]:
            lines.append(
                f"  expected={f['expected']!r}  got={f['got']!r}  "
                f"WER={f.get('wer', '?')}"
            )
        if len(failures) > 5:
            lines.append(f"  ... and {len(failures) - 5} more")
        console.print(
            Panel(
                "\n".join(lines),
                title=f"[red]{task_name} failures[/red]",
                expand=False,
            )
        )


def _print_plain(model: str, task_results: dict) -> None:
    print(f"\n=== Model: {model} ===")
    header = (
        f"{'Task':<16} {'Cases':>6} {'Accuracy':>14} "
        f"{'Avg WER':>10} {'Avg RTF':>10} {'Latency':>10}"
    )
    print(header)
    print("-" * len(header))
    for task_name, r in task_results.items():
        cases = r.get("cases", [])
        passed = sum(1 for c in cases if c["passed"])
        total = len(cases)
        pct = (passed / total * 100) if total else 0
        wer = r.get("avg_wer")
        rtf = r.get("avg_rtf")
        lat = r.get("avg_latency_s")

        print(
            f"{task_name:<16} {total:>6} {passed}/{total} ({pct:.0f}%){'':<3} "
            f"{f'{wer:.2%}' if wer is not None else '—':>10} "
            f"{f'{rtf:.4f}' if rtf is not None else '—':>10} "
            f"{f'{lat:.3f}' if lat is not None else '—':>10}"
        )


def print_comparison_table(all_results: dict[str, dict[str, dict]]) -> None:
    """Side-by-side accuracy table for all models."""
    models = list(all_results.keys())
    tasks = list(next(iter(all_results.values())).keys()) if all_results else []
    if not models or not tasks:
        return

    if _RICH:
        console = Console()
        t = Table(title="Model Comparison (Accuracy %)", box=box.SIMPLE_HEAD)
        t.add_column("Task", style="cyan")
        for m in models:
            t.add_column(m, justify="right")
        for task in tasks:
            row = [task]
            for m in models:
                r = all_results[m].get(task, {})
                cases = r.get("cases", [])
                passed = sum(1 for c in cases if c["passed"])
                total = len(cases)
                pct = (passed / total * 100) if total else 0
                color = "green" if pct >= 80 else ("yellow" if pct >= 60 else "red")
                row.append(f"[{color}]{pct:.0f}%[/{color}]")
            t.add_row(*row)
        console.print(t)
    else:
        print("\n=== Comparison ===")
        header = f"{'Task':<16}" + "".join(f"{m[:14]:>16}" for m in models)
        print(header)
        print("-" * len(header))
        for task in tasks:
            row = f"{task:<16}"
            for m in models:
                r = all_results[m].get(task, {})
                cases = r.get("cases", [])
                passed = sum(1 for c in cases if c["passed"])
                total = len(cases)
                pct = (passed / total * 100) if total else 0
                row += f"{pct:.0f}%".rjust(16)
            print(row)


def save_json(all_results: dict, output_dir: str) -> str:
    os.makedirs(output_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(output_dir, f"benchmark_{ts}.json")

    report: dict[str, Any] = {"timestamp": datetime.now().isoformat(), "models": {}}
    for model, task_results in all_results.items():
        report["models"][model] = {}
        for task, r in task_results.items():
            cases = r.get("cases", [])
            passed = sum(1 for c in cases if c["passed"])
            failed = [c for c in cases if not c["passed"]]
            report["models"][model][task] = {
                "accuracy": round(passed / len(cases), 3) if cases else 0,
                "cases": len(cases),
                "passed": passed,
                "failed_cases": failed[:10],
                "avg_wer": r.get("avg_wer"),
                "avg_rtf": r.get("avg_rtf"),
                "avg_latency_s": r.get("avg_latency_s"),
            }

    with open(path, "w") as f:
        json.dump(report, f, indent=2, default=str)
    return path
