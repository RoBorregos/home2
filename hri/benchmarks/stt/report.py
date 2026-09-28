"""Terminal output for STT benchmarks."""

import os

try:
    from rich import box
    from rich.console import Console
    from rich.table import Table

    _RICH = True
except ImportError:
    _RICH = False

_force_plain = os.environ.get("STT_FORCE_PLAIN", "") == "1"
_DASH = "\u2014"


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
            f"{wer:.2%}" if wer is not None else _DASH,
            f"{rtf:.4f}" if rtf is not None else _DASH,
            f"{lat:.3f}" if lat is not None else _DASH,
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
            f"{f'{wer:.2%}' if wer is not None else _DASH:>10} "
            f"{f'{rtf:.4f}' if rtf is not None else _DASH:>10} "
            f"{f'{lat:.3f}' if lat is not None else _DASH:>10}"
        )
