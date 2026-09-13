"""Terminal output and JSON report writer."""

import json
import os
from datetime import datetime
from typing import Any, Optional

try:
    from rich.console import Console
    from rich.table import Table
    from rich import box

    _RICH = True
except ImportError:
    _RICH = False

_USAGE_NOTE = (
    "tok/s shown as n/a means the backend did not report completion_tokens; "
    "throughput is never estimated."
)


def _fmt(value, spec: str = ".0f") -> str:
    return format(value, spec) if value is not None else "—"


def _accuracy_parts(results: dict):
    cases = results.get("cases", [])
    passed = sum(1 for c in cases if c["passed"])
    total = len(cases)
    return passed, total, (passed / total * 100) if total else 0.0


def _accuracy_cell(r: dict) -> str:
    passed, total, pct = _accuracy_parts(r)
    return f"{passed}/{total} ({pct:.0f}%)" if total else "—"


def _tps_cell(r: dict) -> str:
    if r.get("usage_missing") and r.get("p50_tokens_per_s") is None:
        return "n/a"
    return _fmt(r.get("p50_tokens_per_s"), ".1f")


def _json_cell(r: dict) -> str:
    rate = r.get("json_fail_rate")
    if rate is None:
        return "—"
    return f"{rate * 100:.0f}% ({r.get('json_checked', 0) - r.get('json_ok', 0)}/{r.get('json_checked', 0)})"


def print_model_table(model: str, task_results: dict[str, dict]) -> None:
    if _RICH:
        _print_rich(model, task_results)
    else:
        _print_plain(model, task_results)


def _print_rich(model: str, task_results: dict) -> None:
    console = Console()
    t = Table(title=f"Model: {model}", box=box.ROUNDED, show_lines=False)
    t.add_column("Task", style="cyan", no_wrap=True)
    t.add_column("Cases", justify="right")
    t.add_column("Accuracy", justify="right")
    t.add_column("TTFT p50", justify="right")
    t.add_column("TTFT p95", justify="right")
    t.add_column("tok/s p50", justify="right")
    t.add_column("JSON fail", justify="right")
    t.add_column("Schema", justify="left")

    degraded = False
    usage_missing = False
    for task_name, r in task_results.items():
        _, total, pct = _accuracy_parts(r)
        acc_color = (
            "white"
            if not total
            else ("green" if pct >= 80 else ("yellow" if pct >= 60 else "red"))
        )
        rate = r.get("json_fail_rate")
        json_color = "green" if rate == 0 else ("red" if rate else "white")
        mode = r.get("schema_mode", "—")
        if mode and "json_object" in mode:
            degraded = True
        if r.get("usage_missing"):
            usage_missing = True
        t.add_row(
            task_name,
            str(total),
            f"[{acc_color}]{_accuracy_cell(r)}[/{acc_color}]",
            _fmt(r.get("p50_ttft_ms")),
            _fmt(r.get("p95_ttft_ms")),
            _tps_cell(r),
            f"[{json_color}]{_json_cell(r)}[/{json_color}]",
            mode,
        )

    console.print(t)
    if degraded:
        console.print(
            "[yellow]Schema note:[/yellow] backend rejected the production "
            "json_schema and fell back to json_object."
        )
    if usage_missing:
        console.print(f"[yellow]Note:[/yellow] {_USAGE_NOTE}")
    _print_failures_rich(console, task_results)


def _print_failures_rich(console, task_results: dict) -> None:
    from rich.panel import Panel

    for task_name, r in task_results.items():
        failures = [c for c in r.get("cases", []) if not c["passed"]]
        if not failures:
            continue
        lines = []
        for f in failures[:5]:
            inp = f.get("input", "")
            if isinstance(inp, list):
                inp = str(inp[:2])
            lines.append(f"  expected={f['expected']!r}  got={f['got']!r}  ({inp})")
        if len(failures) > 5:
            lines.append(f"  ... and {len(failures) - 5} more")
        console.print(
            Panel(
                "\n".join(lines), title=f"[red]{task_name} failures[/red]", expand=False
            )
        )


def _print_plain(model: str, task_results: dict) -> None:
    print(f"\n=== Model: {model} ===")
    header = (
        f"{'Task':<22} {'Cases':>6} {'Accuracy':>14} {'TTFTp50':>9} {'TTFTp95':>9}"
        f" {'tok/s':>8} {'JSONfail':>14} {'Schema':<13}"
    )
    print(header)
    print("-" * len(header))
    usage_missing = False
    for task_name, r in task_results.items():
        _, total, _ = _accuracy_parts(r)
        if r.get("usage_missing"):
            usage_missing = True
        acc = _accuracy_cell(r)
        print(
            f"{task_name:<22} {total:>6} {acc:>14} {_fmt(r.get('p50_ttft_ms')):>9}"
            f" {_fmt(r.get('p95_ttft_ms')):>9} {_tps_cell(r):>8}"
            f" {_json_cell(r):>14} {r.get('schema_mode', '—'):<13}"
        )
    if usage_missing:
        print(f"Note: {_USAGE_NOTE}")


def print_comparison_table(all_results: dict[str, dict[str, dict]]) -> None:
    """Side-by-side accuracy, TTFT and JSON-conformance across backends/models."""
    labels = list(all_results.keys())
    tasks: list[str] = []
    for task_results in all_results.values():
        for name in task_results:
            if name not in tasks:
                tasks.append(name)
    if not labels or not tasks:
        return

    metrics = [
        (
            "Accuracy",
            lambda r: f"{_accuracy_parts(r)[2]:.0f}%" if r.get("cases") else "—",
        ),
        ("TTFT p50", lambda r: _fmt(r.get("p50_ttft_ms"))),
        ("tok/s p50", _tps_cell),
        ("JSON fail", _json_cell),
    ]

    if _RICH:
        console = Console()
        for title, getter in metrics:
            t = Table(title=f"Comparison — {title}", box=box.SIMPLE_HEAD)
            t.add_column("Task", style="cyan")
            for label in labels:
                t.add_column(label, justify="right")
            for task in tasks:
                row = [task]
                for label in labels:
                    r = all_results[label].get(task)
                    row.append(getter(r) if r else "—")
                t.add_row(*row)
            console.print(t)
    else:
        for title, getter in metrics:
            print(f"\n=== Comparison — {title} ===")
            header = f"{'Task':<22}" + "".join(f"{m[:14]:>16}" for m in labels)
            print(header)
            print("-" * len(header))
            for task in tasks:
                row = f"{task:<22}"
                for label in labels:
                    r = all_results[label].get(task)
                    row += (getter(r) if r else "—").rjust(16)
                print(row)


def save_json(all_results: dict, output_dir: str, config: Optional[dict] = None) -> str:
    os.makedirs(output_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(output_dir, f"benchmark_{ts}.json")

    report: dict[str, Any] = {
        "timestamp": datetime.now().isoformat(),
        "config": config or {},
        "models": {},
    }
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
                "avg_ttft_ms": r.get("avg_ttft_ms"),
                "p50_ttft_ms": r.get("p50_ttft_ms"),
                "p95_ttft_ms": r.get("p95_ttft_ms"),
                "avg_tokens_per_s": r.get("avg_tokens_per_s"),
                "p50_tokens_per_s": r.get("p50_tokens_per_s"),
                "p95_tokens_per_s": r.get("p95_tokens_per_s"),
                "avg_decode_tokens_per_s": r.get("avg_decode_tokens_per_s"),
                "p50_decode_tokens_per_s": r.get("p50_decode_tokens_per_s"),
                "usage_missing": r.get("usage_missing", False),
                "schema_mode": r.get("schema_mode"),
                "json_checked": r.get("json_checked"),
                "json_ok": r.get("json_ok"),
                "json_fail_rate": r.get("json_fail_rate"),
                "runs_ok": r.get("runs_ok"),
                "runs_requested": r.get("runs_requested"),
                "errors": r.get("errors"),
            }

    with open(path, "w") as f:
        json.dump(report, f, indent=2, default=str)
    return path


def merge_reports(paths: list[str]) -> dict:
    """Load saved benchmark JSONs into one label -> task -> metrics mapping."""
    merged: dict[str, dict] = {}
    for p in paths:
        with open(p) as f:
            data = json.load(f)
        backend = (data.get("config") or {}).get("backend") or "unknown"
        for model, task_results in (data.get("models") or {}).items():
            label = f"{backend}/{model}"
            merged[label] = {
                task: dict(
                    r,
                    cases=[{"passed": True}] * r.get("passed", 0)
                    + [{"passed": False}] * (r.get("cases", 0) - r.get("passed", 0)),
                )
                for task, r in task_results.items()
            }
    return merged


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("usage: report.py <benchmark_a.json> [benchmark_b.json ...]")
        raise SystemExit(1)
    merged = merge_reports(sys.argv[1:])
    for label, task_results in merged.items():
        print_model_table(label, task_results)
    if len(merged) > 1:
        print_comparison_table(merged)
