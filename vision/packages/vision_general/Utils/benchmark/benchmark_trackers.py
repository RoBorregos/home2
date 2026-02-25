#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import glob
import csv
from typing import Dict, List, Tuple

import numpy as np
import motmetrics as mm


def load_mot_txt(path: str) -> Dict[int, List[Tuple[int, List[float]]]]:
    frames: Dict[int, List[Tuple[int, List[float]]]] = {}
    if not os.path.exists(path):
        return frames
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            parts = s.split(",")
            if len(parts) < 6:
                continue
            frame = int(float(parts[0]))
            obj_id = int(float(parts[1]))
            x = float(parts[2])
            y = float(parts[3])
            w = float(parts[4])
            h = float(parts[5])
            bbox = [x, y, w, h]
            frames.setdefault(frame, []).append((obj_id, bbox))
    return frames


def iou_matrix(gt_boxes: List[List[float]], trk_boxes: List[List[float]]) -> np.ndarray:
    """
    Compute IOU distance matrix between ground truth and tracker boxes.
    Returns distance matrix (1 - IOU) for use with motmetrics.
    """
    if len(gt_boxes) == 0 and len(trk_boxes) == 0:
        return np.zeros((0, 0))
    if len(gt_boxes) == 0:
        return np.ones((0, len(trk_boxes)))
    if len(trk_boxes) == 0:
        return np.ones((len(gt_boxes), 0))

    # Convert to numpy arrays with proper dtype
    gt = np.asarray(gt_boxes, dtype=np.float64)
    tr = np.asarray(trk_boxes, dtype=np.float64)

    # Convert from [x, y, w, h] to [x1, y1, x2, y2]
    gt_xy = np.column_stack(
        (gt[:, 0], gt[:, 1], gt[:, 0] + gt[:, 2], gt[:, 1] + gt[:, 3])
    )
    tr_xy = np.column_stack(
        (tr[:, 0], tr[:, 1], tr[:, 0] + tr[:, 2], tr[:, 1] + tr[:, 3])
    )

    # Compute IOU matrix manually (NumPy 2.0 compatible)
    n_gt = len(gt_xy)
    n_tr = len(tr_xy)
    iou_mat = np.zeros((n_gt, n_tr), dtype=np.float64)

    for i in range(n_gt):
        for j in range(n_tr):
            # Compute intersection
            x1 = max(gt_xy[i, 0], tr_xy[j, 0])
            y1 = max(gt_xy[i, 1], tr_xy[j, 1])
            x2 = min(gt_xy[i, 2], tr_xy[j, 2])
            y2 = min(gt_xy[i, 3], tr_xy[j, 3])

            inter_area = max(0.0, x2 - x1) * max(0.0, y2 - y1)

            # Compute union
            gt_area = (gt_xy[i, 2] - gt_xy[i, 0]) * (gt_xy[i, 3] - gt_xy[i, 1])
            tr_area = (tr_xy[j, 2] - tr_xy[j, 0]) * (tr_xy[j, 3] - tr_xy[j, 1])
            union_area = gt_area + tr_area - inter_area

            # Compute IOU
            if union_area > 0:
                iou_mat[i, j] = inter_area / union_area
            else:
                iou_mat[i, j] = 0.0

    # Return distance matrix (1 - IOU)
    return 1.0 - iou_mat


def evaluate_sequence(gt_path: str, trk_path: str) -> mm.MOTAccumulator:
    gt = load_mot_txt(gt_path)
    trk = load_mot_txt(trk_path)

    frames = sorted(set(list(gt.keys()) + list(trk.keys())))
    acc = mm.MOTAccumulator(auto_id=True)

    for frame in frames:
        gt_objs = gt.get(frame, [])
        trk_objs = trk.get(frame, [])

        gt_ids = [g[0] for g in gt_objs]
        gt_boxes = [g[1] for g in gt_objs]
        trk_ids = [t[0] for t in trk_objs]
        trk_boxes = [t[1] for t in trk_objs]

        if len(gt_boxes) == 0 and len(trk_boxes) == 0:
            acc.update([], [], [])
            continue

        dist = iou_matrix(gt_boxes, trk_boxes)
        acc.update(gt_ids, trk_ids, dist)

    return acc


def find_sequence_files(gt_root: str) -> List[Tuple[str, str]]:
    seq_files = []
    for path in glob.glob(os.path.join(gt_root, "*.txt")):
        seq_name = os.path.basename(path)
        seq_files.append((seq_name, path))
    seq_files.sort()
    return seq_files


def run_benchmark(gt_root: str, trackers: Dict[str, str], out_csv: str = None):
    seqs = find_sequence_files(gt_root)
    if not seqs:
        raise FileNotFoundError(f"No GT .txt files found in {gt_root}")

    results = []
    metrics_list = mm.metrics.motchallenge_metrics

    for trk_name, trk_root in trackers.items():
        accs = []
        seq_names = []
        for seq_name, gt_path in seqs:
            trk_path = os.path.join(trk_root, seq_name)
            acc = evaluate_sequence(gt_path, trk_path)
            accs.append(acc.events)  # Pass the events DataFrame
            seq_names.append(seq_name)

        mh = mm.metrics.create()
        summary = mh.compute_many(
            accs, names=seq_names, metrics=metrics_list, generate_overall=True
        )
        overall = summary.loc["OVERALL"]

        row = {"tracker": trk_name}
        for m in ["mota", "motp", "idf1", "num_switches", "precision", "recall"]:
            row[m] = float(overall.get(m, np.nan))

        results.append(row)

        if out_csv:
            csv_path = out_csv.replace(".csv", f"_{trk_name}.csv")
            summary.to_csv(csv_path)

    if out_csv:
        keys = sorted(results[0].keys())
        with open(out_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            for r in results:
                writer.writerow(r)

    return results


def parse_trackers_arg(arg_list: List[str]) -> Dict[str, str]:
    trackers: Dict[str, str] = {}
    for item in arg_list:
        if ":" not in item:
            raise argparse.ArgumentTypeError("Tracker argument must be name:path")
        name, path = item.split(":", 1)
        trackers[name] = path
    return trackers


def main():
    parser = argparse.ArgumentParser(description="Benchmark trackers against MOT16 GT")
    parser.add_argument(
        "--gt", required=True, help="Path to MOT16 ground-truth folder (txt files)"
    )
    parser.add_argument(
        "--trackers",
        required=True,
        nargs="+",
        help="Tracker entries in the form name:folder_with_txts",
    )
    parser.add_argument(
        "--out", default="benchmark_summary.csv", help="Output CSV summary file"
    )

    args = parser.parse_args()
    trackers = parse_trackers_arg(args.trackers)

    print("Found trackers:", trackers)
    results = run_benchmark(args.gt, trackers, out_csv=args.out)
    print("Benchmark finished. Summary:")
    for r in results:
        print(r)


if __name__ == "__main__":
    main()
