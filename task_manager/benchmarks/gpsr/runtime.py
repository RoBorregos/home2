"""Measure warmed public GPSR merger calls without importing ROS or sending goals."""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys
import time
from types import SimpleNamespace


def load_planner(path):
    if path is None:
        from task_manager.gpsr import merger

        return merger
    spec = importlib.util.spec_from_file_location("historical_gpsr_merger", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def stress_batch(sites):
    commands = []
    for command in range(3):
        actions = []
        for leg in range(4):
            actions.extend(
                [
                    {"action": "go_to", "location_to_go": sites[(command + leg) % len(sites)]},
                    {"action": "count", "target_to_count": "cups"},
                ]
            )
        commands.append(actions)
    return commands


def benchmark(planner, batches, options, calls):
    used_fallback = False
    original = planner._sequential_schedule

    def sequential(*args, **kwargs):
        nonlocal used_fallback
        used_fallback = True
        return original(*args, **kwargs)

    rows = []
    planner._sequential_schedule = sequential
    try:
        for name, batch in batches:
            commands = [
                SimpleNamespace(commands=[SimpleNamespace(**action) for action in actions])
                for actions in batch
            ]
            segments = sum(len(planner._decompose(i, cmd)) for i, cmd in enumerate(commands))
            planner.merge(commands, **options)
            samples = []
            fallbacks = 0
            for _ in range(calls):
                used_fallback = False
                started = time.perf_counter_ns()
                planner.merge(commands, **options)
                samples.append((time.perf_counter_ns() - started) / 1e6)
                fallbacks += used_fallback
            rows.append(
                dict(
                    batch=name,
                    segments=segments,
                    samples_ms=samples,
                    median_ms=statistics.median(samples),
                    p95_ms=sorted(samples)[math.ceil(0.95 * calls) - 1],
                    fallback_count=fallbacks,
                )
            )
    finally:
        planner._sequential_schedule = original
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--historical", type=Path)
    parser.add_argument(
        "--settings", type=Path, help="Optional JSON with public planner cost settings"
    )
    parser.add_argument("--calls", type=int, default=100)
    args = parser.parse_args()
    if args.calls < 1:
        parser.error("--calls must be positive")
    manifest = json.loads(args.manifest.read_text())
    locations = json.loads(args.layout.read_text())["locations"]
    coords = {name: tuple(pose[:2]) for name, pose in locations.items()}
    sites = [name for name in coords if name not in ("start", "start_location")]
    if "start_location" not in coords or len(sites) < 4:
        parser.error("--layout must contain at least four sites plus start_location")
    batches = [
        (batch["name"], [record["actions"] for record in batch["records"]])
        for batch in manifest["batches"]
    ]
    batches.append(("stress-3x4", stress_batch(sites)))
    settings = json.loads(args.settings.read_text()) if args.settings else {}
    planner = load_planner(args.historical)
    rows = benchmark(
        planner,
        batches,
        dict(locator=coords.get, origin=coords["start_location"], **settings),
        args.calls,
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x") as output:
        json.dump(
            dict(
                module=planner.__file__,
                manifest=str(args.manifest),
                layout=str(args.layout),
                settings=settings,
                results=rows,
            ),
            output,
            indent=2,
        )
        output.write("\n")


if __name__ == "__main__":
    main()
