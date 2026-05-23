#!/usr/bin/env python3
"""NLP model benchmark: accuracy + performance across Ollama models.

Usage:
  python benchmark.py
  python benchmark.py --models qwen3 gemma3:4b
  python benchmark.py --tasks extract_data is_positive
  python benchmark.py --ollama-url http://localhost:11434/v1
  python benchmark.py --runs 5
  python benchmark.py --no-perf
  python benchmark.py --thinking
"""

import argparse
import sys
from pathlib import Path

import yaml
from openai import OpenAI

import report as rpt
from tasks import TASK_REGISTRY, run_perf

CONFIG_PATH = Path(__file__).parent / "config.yaml"


def load_config(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def resolve_test_file(raw_path: str, config_file: Path) -> Path:
    p = Path(raw_path)
    if not p.is_absolute():
        p = (config_file.parent / p).resolve()
    # Also try relative to /test_data mount (Docker mode)
    if not p.exists():
        docker_path = Path("/test_data") / Path(raw_path).name
        if docker_path.exists():
            return docker_path
    return p


def check_ollama(base_url: str, api_key: str) -> bool:
    from openai import OpenAI

    client = OpenAI(base_url=base_url, api_key=api_key)
    try:
        client.models.list()
        return True
    except Exception as e:
        print(f"ERROR: Cannot reach Ollama at {base_url}: {e}")
        return False


def pull_model_if_needed(base_url: str, model: str) -> None:
    """Attempt to pull model via Ollama HTTP API if not present."""
    import json as _json
    import urllib.parse
    import urllib.request

    host = base_url.replace("/v1", "").rstrip("/")
    try:
        models_url = f"{host}/api/tags"
        with urllib.request.urlopen(models_url, timeout=5) as resp:
            data = _json.loads(resp.read())
        names = [m["name"] for m in data.get("models", [])]

        # Normalize: "qwen3:latest" matches "qwen3"
        def normalize(n):
            return n.split(":")[0]

        if any(normalize(n) == normalize(model) for n in names):
            return
        print(
            f"  Model '{model}' not found locally — pulling (this may take a while)..."
        )
        pull_url = f"{host}/api/pull"
        payload = _json.dumps({"name": model, "stream": False}).encode()
        req = urllib.request.Request(
            pull_url, data=payload, headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=300) as resp:
            resp.read()
        print(f"  Pulled '{model}' successfully.")
    except Exception as e:
        print(f"  Warning: could not auto-pull '{model}': {e}")


def run_benchmark(args: argparse.Namespace) -> None:
    cfg = load_config(args.config)

    base_url = args.ollama_url or cfg["ollama"]["base_url"]
    api_key = cfg["ollama"].get("api_key", "ollama")
    runs = args.runs if args.runs is not None else cfg["benchmark"].get("runs", 3)
    do_perf = not args.no_perf
    output_dir = cfg["output"].get("dir", "./results")
    save_json = cfg["output"].get("save_json", True)

    models = args.models or cfg.get("models", [])
    if not models:
        print("ERROR: no models specified. Use --models or set 'models' in config.yaml")
        sys.exit(1)

    # Select enabled tasks
    task_cfg = cfg.get("tasks", {})
    enabled_tasks = (
        args.tasks
        if args.tasks
        else [t for t, v in task_cfg.items() if v.get("enabled", False)]
    )
    if not enabled_tasks:
        print(
            "ERROR: no tasks enabled. Use --tasks or set enabled: true in config.yaml"
        )
        sys.exit(1)

    if not check_ollama(base_url, api_key):
        sys.exit(1)

    client = OpenAI(base_url=base_url, api_key=api_key)

    all_results: dict = {}

    for model in models:
        print(f"\n{'─'*60}")
        print(f"  Benchmarking model: {model}")
        print(f"{'─'*60}")
        pull_model_if_needed(base_url, model)

        model_results: dict = {}

        for task_name in enabled_tasks:
            if task_name not in TASK_REGISTRY:
                print(f"  WARNING: unknown task '{task_name}', skipping")
                continue

            task_cls = TASK_REGISTRY[task_name]
            raw_path = task_cfg.get(task_name, {}).get("test_file", "")
            if not raw_path:
                print(f"  WARNING: no test_file for '{task_name}', skipping")
                continue

            test_file = resolve_test_file(raw_path, args.config)
            if not test_file.exists():
                print(f"  WARNING: test file not found: {test_file}")
                continue

            print(f"\n  Task: {task_name}  ({test_file.name})")

            cases_data = task_cls.load(str(test_file))
            case_results = []
            for i, case in enumerate(cases_data):
                print(f"    case {i+1}/{len(cases_data)}\r", end="", flush=True)
                r = task_cls.run_case(client, model, case)
                case_results.append(r)
            print(" " * 40 + "\r", end="")

            passed = sum(1 for c in case_results if c["passed"])
            pct = passed / len(case_results) * 100 if case_results else 0
            print(f"    Accuracy: {passed}/{len(case_results)} ({pct:.0f}%)")

            task_r: dict = {"cases": case_results}

            if do_perf:
                print(f"    Running {runs} perf runs...", end="", flush=True)
                perf = run_perf(client, model, task_cls, runs)
                task_r.update(perf)
                ttft = perf.get("avg_ttft_ms")
                tps = perf.get("avg_tokens_per_s")
                print(
                    f" TTFT={ttft:.0f}ms  tok/s={tps:.1f}"
                    if ttft
                    else " (no perf data)"
                )

            model_results[task_name] = task_r

        all_results[model] = model_results
        rpt.print_model_table(model, model_results)

    if len(models) > 1:
        rpt.print_comparison_table(all_results)

    if save_json:
        path = rpt.save_json(all_results, output_dir)
        print(f"\nResults saved to: {path}")


def main():
    parser = argparse.ArgumentParser(
        description="NLP model benchmark for Jetson AI Labs models"
    )
    parser.add_argument(
        "--config", type=Path, default=CONFIG_PATH, help="Path to config.yaml"
    )
    parser.add_argument(
        "--models", nargs="+", help="Models to benchmark (e.g. qwen3 gemma3:4b)"
    )
    parser.add_argument(
        "--tasks", nargs="+", help="Tasks to run (extract_data is_positive ...)"
    )
    parser.add_argument("--ollama-url", help="Ollama base URL (overrides config)")
    parser.add_argument("--runs", type=int, help="Number of perf timing runs per task")
    parser.add_argument(
        "--no-perf", action="store_true", help="Skip performance timing, accuracy only"
    )
    parser.add_argument(
        "--thinking",
        action="store_true",
        help="Enable thinking mode (removes /no_think)",
    )
    args = parser.parse_args()

    run_benchmark(args)


if __name__ == "__main__":
    main()
