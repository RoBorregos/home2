#!/usr/bin/env python3

"""
Issue #1152: compare "always ask for confirmation" (hri.confirm) with "say it and wait a
fixed timeout" for the GPSR fallbacks and chair removal. You play the human: for each trial,
answer out loud and the script logs the answer and how long the robot took.

Run inside the HRI container (./run.sh integration):
    cd /workspace/src/task_manager/scripts/test
    python3 test_confirm_scenarios.py                         # all scenarios, both modes
    python3 test_confirm_scenarios.py chairs grabbed --mode confirm --wait 5
"""

import argparse
import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from task_manager.subtask_managers.hri_tasks import HRITasks
from task_manager.utils.task import Task

# confirm: the exact hri.confirm call used by the task today (or the proposed one).
# say: the statement used by the "say and wait" alternative.
SCENARIOS = {
    "person_found": dict(  # gpsr_tasks.py find-person-by-attribute
        confirm=dict(question="Is this the person you were looking for?", retries=2),
        say="I found the person you were looking for.",
    ),
    "person_name": dict(  # gpsr_tasks.py find-person-by-name
        confirm=dict(question="Hi, I'm Frida. Are you John?", retries=2),
        say="Hi, I'm Frida. I am looking for John, please say yes if that is you.",
    ),
    "grabbed": dict(  # gpsr_tasks.py give-object
        confirm=dict(question="Have you grabbed the object?"),
        say="Please grab the object from my gripper.",
    ),
    "chairs": dict(  # pickandplace_task_manager.py chair removal (today: say + 5 s, no confirm)
        confirm=dict(question="Please remove the chairs from the dining table. Say yes when done."),
        say="Please remove the chairs from the dining table.",
    ),
}


def run_trial(hri, name, mode, wait):
    sc = SCENARIOS[name]
    start = time.monotonic()
    if mode == "confirm":
        status, answer = hri.confirm(**sc["confirm"])
    else:
        hri.say(sc["say"], wait=True)
        time.sleep(wait)
        status, answer = None, "(timeout)"
    return [name, mode, str(status), answer, f"{time.monotonic() - start:.1f}"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenarios", nargs="*", help=f"any of: {', '.join(SCENARIOS)} (default: all)")
    ap.add_argument("--mode", choices=["confirm", "timeout", "both"], default="both")
    ap.add_argument("--wait", type=float, default=5.0, help="seconds for the say+timeout mode")
    args = ap.parse_args()
    names = args.scenarios or list(SCENARIOS)
    unknown = [n for n in names if n not in SCENARIOS]
    if unknown:
        ap.error(f"unknown scenarios {unknown}; choose from {list(SCENARIOS)}")
    modes = ["confirm", "timeout"] if args.mode == "both" else [args.mode]

    rclpy.init()
    node = Node("test_confirm_scenarios")
    hri = HRITasks(node, task=Task.GPSR)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "confirm_scenarios_output")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, f"{datetime.now():%Y-%m-%d_%H-%M-%S}.csv")

    rows = []
    try:
        for name in names:
            for mode in modes:
                input(f"\n[{name} / {mode}] press Enter, then answer the robot out loud... ")
                rows.append(run_trial(hri, name, mode, args.wait))
                print("  ->", rows[-1])
    except KeyboardInterrupt:
        pass
    finally:
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["scenario", "mode", "status", "answer", "seconds"])
            w.writerows(rows)
        print(f"Results saved to {path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
