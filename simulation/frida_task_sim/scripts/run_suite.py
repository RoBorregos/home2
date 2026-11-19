#!/usr/bin/env python3
"""Runs several seeded scenarios one after another and summarizes their scores.

    ros2 run frida_task_sim run_suite.py --tasks ppc --seeds 1,2,3

Exit code: 0 all runs scored, 2 at least one infrastructure failure.
"""

import argparse
import json
import os
import subprocess
import sys
import time


def run_one(task: str, seed: int, out: str) -> dict:
    """Run one scenario in its own process and return its score (or the failure)."""
    code = subprocess.run(
        [
            "ros2",
            "run",
            "frida_task_sim",
            "run_scenario.py",
            "--task",
            task,
            "--seed",
            str(seed),
            "--out",
            out,
            "--headless",
            "--no-rviz",
        ],
        check=False,
    ).returncode
    score_file = os.path.join(out, "score.json")
    if not os.path.exists(score_file):
        return {
            "task": task,
            "seed": seed,
            "total": 0,
            "max_in_sim": 0,
            "status": "no score written",
            "code": code,
        }
    with open(score_file) as f:
        score = json.load(f)
    return {
        **{k: score[k] for k in ("task", "seed", "total", "max_in_sim", "status")},
        "code": code,
    }


def summarize(results: list[dict]) -> str:
    lines = [
        "# Task simulation suite",
        "",
        "| Task | Seed | Score | Reachable in sim | Status |",
        "|---|---:|---:|---:|---|",
    ]
    for r in results:
        lines.append(
            f"| {r['task']} | {r['seed']} | {r['total']} | {r['max_in_sim']} | {r['status']} |"
        )
    lines.append("")
    for task in sorted({r["task"] for r in results}):
        scores = [r["total"] for r in results if r["task"] == task and r["code"] == 0]
        if scores:
            lines.append(
                f"**{task}**: mean {sum(scores) / len(scores):.0f}, min {min(scores)}, max {max(scores)} over {len(scores)} run(s)"
            )
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tasks", default="ppc")
    parser.add_argument("--seeds", default="1,2,3")
    parser.add_argument("--out", default="")
    args = parser.parse_args()
    out = args.out or f"/workspace/sim_logs/runs/suite_{time.strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(out, exist_ok=True)
    results = []
    for task in args.tasks.split(","):
        for seed in (int(s) for s in args.seeds.split(",")):
            results.append(run_one(task, seed, os.path.join(out, f"{task}_seed{seed}")))
            with open(os.path.join(out, "summary.json"), "w") as f:
                json.dump(results, f, indent=2)
            with open(os.path.join(out, "summary.md"), "w") as f:
                f.write(summarize(results))
    print(summarize(results))
    print(f"summary: {out}/summary.md")
    sys.exit(2 if any(r["code"] != 0 for r in results) else 0)


if __name__ == "__main__":
    main()
