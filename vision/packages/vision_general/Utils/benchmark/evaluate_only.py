#!/usr/bin/env python3
"""
Quick evaluation script - runs only the evaluation step using existing tracker outputs
"""

import os
import sys
from pathlib import Path

# Add vision_general to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from benchmark_trackers import run_benchmark


def main():
    output_dir = "./output"
    gt_dir = f"{output_dir}/gt"
    results_dir = f"{output_dir}/results"

    # Find all tracker outputs
    tracker_outputs = {}
    trackers_dir = f"{output_dir}/trackers"

    if not os.path.exists(trackers_dir):
        print(f"Error: Trackers directory not found: {trackers_dir}")
        return

    for tracker_name in os.listdir(trackers_dir):
        tracker_path = os.path.join(trackers_dir, tracker_name)
        if os.path.isdir(tracker_path):
            tracker_outputs[tracker_name] = tracker_path
            print(f"Found tracker: {tracker_name}")

    if not tracker_outputs:
        print("No tracker outputs found!")
        return

    print("\nEvaluating...")
    os.makedirs(results_dir, exist_ok=True)
    results = run_benchmark(
        gt_dir, tracker_outputs, out_csv=f"{results_dir}/summary.csv"
    )

    print("\nResults:")
    for r in results:
        print(f"{r['tracker']}: MOTA={r['mota']:.3f}, IDF1={r['idf1']:.3f}")

    print(f"\nDone! Results saved to: {results_dir}/summary.csv")


if __name__ == "__main__":
    main()
