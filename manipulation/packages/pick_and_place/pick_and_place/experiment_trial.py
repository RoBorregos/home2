#!/usr/bin/env python3

"""Run one benchmark step per call, so an agent can drive trials and log operator verdicts."""

import argparse
import csv
import json
import os
import time

import rclpy

from pick_benchmark import CSV_HEADER, PickBenchmark


# Table height and reset spots in base_link, in metres.
TABLE_Z = 0.725
RESET_SPOTS = [(0.45, 0.0), (0.60, 0.25), (0.60, -0.25), (0.33, 0.30), (0.33, -0.30)]


def run_pick(node, args):
    node.set_method(args.method)
    success, duration = node.pick(args.object, False)
    print(json.dumps({"action_success": int(success), "pick_s": round(duration, 1)}))


def run_place(node, args):
    success, duration = node.place(args.request, args.reference)
    print(
        json.dumps(
            {"place_action_success": int(success), "place_s": round(duration, 1)}
        )
    )


def run_cycle(node, args):
    """Pick from the current spot, then place at the spot the next trial starts from."""
    node.set_method(args.method)
    success, pick_s = node.pick(args.object, False)
    result = {"action_success": int(success), "pick_s": round(pick_s, 1)}
    if success:
        # Both methods run at each spot, so the object moves every second trial.
        x, y = RESET_SPOTS[(args.trial // 2) % len(RESET_SPOTS)]
        place_ok, place_s = node.place("", "", (x, y, TABLE_Z))
        result.update(
            place_action_success=int(place_ok),
            place_s=round(place_s, 1),
            next_spot=[x, y],
        )
    print(json.dumps(result))


def run_log(args):
    row = {name: "" for name in CSV_HEADER}
    row.update(json.loads(args.row))
    row["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    row["git_sha"] = os.environ.get("EXPERIMENT_SHA", "unknown")
    row.setdefault("planner", "ompl")
    row["planner"] = row["planner"] or "ompl"
    unknown = set(row) - set(CSV_HEADER)
    if unknown:
        raise SystemExit(f"unknown columns: {sorted(unknown)}")
    new_file = not os.path.exists(args.out) or os.path.getsize(args.out) == 0
    with open(args.out, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
        if new_file:
            writer.writeheader()
        writer.writerow(row)
    print(f"logged to {args.out}")


def main():
    parser = argparse.ArgumentParser(description="One benchmark step")
    sub = parser.add_subparsers(dest="step", required=True)
    pick = sub.add_parser("pick")
    pick.add_argument("--method", choices=["auto", "geometric", "gpd"], required=True)
    pick.add_argument("--object", required=True)
    place = sub.add_parser("place")
    place.add_argument("--request", default="")
    place.add_argument("--reference", default="")
    cycle = sub.add_parser("cycle")
    cycle.add_argument("--method", choices=["auto", "geometric", "gpd"], required=True)
    cycle.add_argument("--object", required=True)
    cycle.add_argument("--trial", type=int, required=True)
    log = sub.add_parser("log")
    log.add_argument("--row", required=True, help="JSON object of CSV columns")
    log.add_argument("--out", default="/workspace/src/bags/experiment.csv")
    args = parser.parse_args()

    if args.step == "log":
        run_log(args)
        return
    rclpy.init()
    node = PickBenchmark()
    try:
        {"pick": run_pick, "place": run_place, "cycle": run_cycle}[args.step](
            node, args
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
