#!/usr/bin/env python3
"""
MOT16 Tracker Benchmark
Usage: python3 benchmark.py --trackers botsort bytetrack hector_tracker
Commands to install: pip3 install ultralytics motmetrics opencv-python tqdm pretrainedmodels timm
"""
import argparse
import os
import glob
import cv2
import numpy as np
from pathlib import Path
from tqdm import tqdm
from ultralytics import YOLO

# Add vision_general to path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

# Import evaluation
from benchmark_trackers import run_benchmark
from prepare_mot_gt import prepare_ground_truth

# Import tracker wrappers
from ros_tracker_wrappers import (
    NewTrackerWrapper,
    OldTrackerWrapper,
    TrackerNodeWrapper,
    TrackerNodeFregosoWrapper,
    NorfairNodeWrapper,
    HectorTracker
)

# Ultralytics tracker wrapper
class UltralyticsTracker:
    def __init__(self, model="yolov8n.pt", tracker="bytetrack.yaml", conf=0.6):
        self.model = YOLO(model)
        self.tracker_config = tracker
        self.conf = conf

    def reset(self):
        self.model.predictor = None

    def process_frame(self, frame):
        results = self.model.track(frame, conf=self.conf, classes=[0], persist=True, tracker=self.tracker_config, verbose=False)
        detections = []
        for r in results:
            if r.boxes is None or r.boxes.id is None:
                continue
            boxes = r.boxes.xyxy.cpu().numpy()
            ids = r.boxes.id.cpu().numpy().astype(int)
            for box, tid in zip(boxes, ids):
                x1, y1, x2, y2 = box
                detections.append((int(tid), [float(x1), float(y1), float(x2-x1), float(y2-y1)]))
        return detections

def run_tracker_on_sequence(tracker, seq_path, output_path):
    tracker.reset()
    img_files = sorted(glob.glob(os.path.join(seq_path, "img1", "*.jpg")))
    results = []

    for frame_idx, img_path in enumerate(tqdm(img_files, desc=Path(seq_path).name)):
        frame = cv2.imread(img_path)
        if frame is None:
            continue
        detections = tracker.process_frame(frame)
        for track_id, bbox in detections:
            x, y, w, h = bbox
            results.append(f"{frame_idx + 1},{track_id},{x:.2f},{y:.2f},{w:.2f},{h:.2f},1,-1,-1")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        f.write("\n".join(results))

def main():
    parser = argparse.ArgumentParser(description="MOT16 Tracker Benchmark")
    parser.add_argument("--mot-root", default="/home/lalo/Desktop/home2/vision/MOT16", help="MOT16 root path")
    parser.add_argument("--split", default="train", choices=["train", "test"])
    parser.add_argument("--output", default="./output", help="Output folder")
    parser.add_argument("--trackers", nargs="+", required=True,
                        choices=["botsort", "bytetrack", "new_tracker", "old_tracker",
                                 "tracker_node", "tracker_node_fregoso", "norfair_node", "hector_tracker"])
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO model")
    parser.add_argument("--conf", type=float, default=0.6, help="Confidence threshold")
    args = parser.parse_args()

    # Setup paths
    gt_dir = f"{args.output}/gt"
    results_dir = f"{args.output}/results"

    # Prepare ground truth
    print("[1/3] Preparing ground truth...")
    prepare_ground_truth(args.mot_root, args.split, gt_dir)

    # Run trackers
    print("[2/3] Running trackers...")
    tracker_outputs = {}

    for tracker_name in args.trackers:
        print(f"\nRunning: {tracker_name}")
        tracker_dir = f"{args.output}/trackers/{tracker_name}"

        # Initialize tracker
        if tracker_name == "botsort":
            tracker = UltralyticsTracker(args.model, "botsort.yaml", args.conf)
        elif tracker_name == "bytetrack":
            tracker = UltralyticsTracker(args.model, "bytetrack.yaml", args.conf)
        elif tracker_name == "new_tracker":
            tracker = NewTrackerWrapper(args.conf)
        elif tracker_name == "old_tracker":
            tracker = OldTrackerWrapper(args.conf)
        elif tracker_name == "tracker_node":
            tracker = TrackerNodeWrapper(args.conf)
        elif tracker_name == "tracker_node_fregoso":
            tracker = TrackerNodeFregosoWrapper(0.8)
        elif tracker_name == "norfair_node":
            tracker = NorfairNodeWrapper(args.conf)
        elif tracker_name == "hector_tracker":
            tracker = HectorTracker(0.4)

        # Run on sequences
        sequences = sorted(glob.glob(os.path.join(args.mot_root, args.split, "MOT16-*")))
        for seq_path in sequences:
            seq_name = os.path.basename(seq_path)
            output_file = os.path.join(tracker_dir, f"{seq_name}.txt")
            run_tracker_on_sequence(tracker, seq_path, output_file)

        tracker_outputs[tracker_name] = tracker_dir

    # Evaluate
    print("[3/3] Evaluating...")
    os.makedirs(results_dir, exist_ok=True)
    results = run_benchmark(gt_dir, tracker_outputs, out_csv=f"{results_dir}/summary.csv")

    print("\nResults:")
    for r in results:
        print(f"{r['tracker']}: MOTA={r['mota']:.3f}, IDF1={r['idf1']:.3f}")

    print(f"\nDone! Results: {results_dir}/summary.csv")

if __name__ == "__main__":
    main()
