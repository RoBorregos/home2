#!/usr/bin/env python3
"""
Benchmark runner for various tracker implementations on MOT16 dataset.
Runs trackers offline on image sequences and outputs results in MOT format.
"""

from __future__ import annotations

import argparse
import os
import glob
import cv2
import numpy as np
from pathlib import Path
from typing import List, Tuple, Dict, Optional
from tqdm import tqdm

# Tracker implementations
from ultralytics import YOLO
try:
    from norfair import Detection, Tracker as NorfairTracker
    NORFAIR_AVAILABLE = True
except ImportError:
    NORFAIR_AVAILABLE = False
    print("Warning: Norfair not available. Norfair tracker will be skipped.")

# Import ROS tracker wrappers
try:
    from ros_tracker_wrappers import (
        NewTrackerWrapper as ROSNewTracker,
        OldTrackerWrapper as ROSOldTracker,
        TrackerNodeWrapper as ROSTrackerNode,
        TrackerNodeFregosoWrapper as ROSTrackerNodeFregoso,
        NorfairNodeWrapper as ROSNorfairNode,
        HectorTracker as ROSHectorTracker,
    )
    ROS_TRACKERS_AVAILABLE = True
except ImportError as e:
    ROS_TRACKERS_AVAILABLE = False
    print(f"Warning: ROS tracker wrappers not available: {e}")


class BaseTrackerWrapper:
    """Base class for tracker wrappers"""

    def __init__(self, conf_threshold: float = 0.6):
        self.conf_threshold = conf_threshold
        self.frame_count = 0

    def reset(self):
        """Reset tracker state for new sequence"""
        self.frame_count = 0

    def process_frame(self, frame: np.ndarray) -> List[Tuple[int, List[float]]]:
        """
        Process a single frame and return detections.
        Returns: List of (track_id, [x, y, w, h])
        """
        raise NotImplementedError


class UltralyticsTrackerWrapper(BaseTrackerWrapper):
    """Wrapper for Ultralytics YOLO with built-in tracking (BoTSORT, ByteTrack, etc.)"""

    def __init__(self, model_path: str = "yolov8n.pt", tracker_config: str = "botsort.yaml", conf_threshold: float = 0.6):
        super().__init__(conf_threshold)
        self.model = YOLO(model_path)
        self.tracker_config = tracker_config
        print(f"Loaded Ultralytics model: {model_path} with tracker: {tracker_config}")

    def reset(self):
        super().reset()
        # Reset tracker by creating new model instance
        self.model.predictor = None

    def process_frame(self, frame: np.ndarray) -> List[Tuple[int, List[float]]]:
        """Process frame with Ultralytics tracker"""
        self.frame_count += 1

        # Run tracking
        results = self.model.track(
            frame,
            conf=self.conf_threshold,
            classes=[0],  # person class
            persist=True,
            tracker=self.tracker_config,
            verbose=False,
        )

        detections = []
        for result in results:
            if result.boxes is None or result.boxes.id is None:
                continue

            boxes = result.boxes.xyxy.cpu().numpy()
            ids = result.boxes.id.cpu().numpy().astype(int)

            for box, track_id in zip(boxes, ids):
                x1, y1, x2, y2 = box
                w = x2 - x1
                h = y2 - y1
                detections.append((int(track_id), [float(x1), float(y1), float(w), float(h)]))

        return detections


class NorfairTrackerWrapper(BaseTrackerWrapper):
    """Wrapper for Norfair tracker with 2-point tracking"""

    def __init__(self, model_path: str = "yolov8n.pt", conf_threshold: float = 0.6,
                 distance_threshold: float = 65.0, hit_counter_max: int = 25, init_delay: int = 2):
        super().__init__(conf_threshold)

        if not NORFAIR_AVAILABLE:
            raise ImportError("Norfair is not installed. Install with: pip install norfair")

        self.detector = YOLO(model_path)
        self.tracker = NorfairTracker(
            distance_function=self._two_point_distance,
            distance_threshold=distance_threshold,
            hit_counter_max=hit_counter_max,
            initialization_delay=init_delay,
        )
        print(f"Loaded Norfair tracker with YOLO: {model_path}")

    def reset(self):
        super().reset()
        # Create new tracker instance
        self.tracker = NorfairTracker(
            distance_function=self._two_point_distance,
            distance_threshold=self.tracker.distance_threshold,
            hit_counter_max=self.tracker.hit_counter_max,
            initialization_delay=self.tracker.initialization_delay,
        )

    def _two_point_distance(self, detection: Detection, tracked_obj) -> float:
        """Average Euclidean distance over 2 points (center and top-center)"""
        det_pts = detection.points
        trk_pts = tracked_obj.estimate

        if det_pts is None or trk_pts is None:
            return 1e9
        if det_pts.shape != trk_pts.shape:
            det_pts = det_pts[:1]
            trk_pts = trk_pts[:1]

        d = np.linalg.norm(det_pts - trk_pts, axis=1)
        return float(np.mean(d))

    def process_frame(self, frame: np.ndarray) -> List[Tuple[int, List[float]]]:
        """Process frame with Norfair tracker"""
        self.frame_count += 1

        # YOLO detection
        results = self.detector.predict(
            frame,
            conf=self.conf_threshold,
            classes=[0],  # person class
            verbose=False,
        )

        # Convert to Norfair detections (2-point representation)
        norfair_detections = []
        bboxes = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())

                if conf < self.conf_threshold:
                    continue

                # Center point and top-center point
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                tcx = cx
                tcy = float(y1)

                points = np.array([[cx, cy], [tcx, tcy]], dtype=np.float32)
                norfair_detections.append(
                    Detection(
                        points=points,
                        scores=np.array([conf, conf], dtype=np.float32),
                    )
                )
                bboxes.append((x1, y1, x2, y2))

        # Update tracker
        tracks = self.tracker.update(norfair_detections)

        # Match tracks to bboxes and output
        detections = []
        for track, bbox in zip(tracks, bboxes[:len(tracks)]):
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                w = x2 - x1
                h = y2 - y1
                detections.append((int(track.id), [float(x1), float(y1), float(w), float(h)]))

        return detections


def run_tracker_on_sequence(
    tracker: BaseTrackerWrapper,
    sequence_path: str,
    output_path: str,
) -> None:
    """
    Run a tracker on a MOT sequence and save results in MOT format.

    Args:
        tracker: Tracker wrapper instance
        sequence_path: Path to MOT sequence (contains img1/ folder)
        output_path: Output file path for tracking results
    """
    # Reset tracker for new sequence
    tracker.reset()

    # Get image files
    img_folder = os.path.join(sequence_path, "img1")
    img_files = sorted(glob.glob(os.path.join(img_folder, "*.jpg")))

    if not img_files:
        print(f"No images found in {img_folder}")
        return

    # Process each frame
    results = []
    for frame_idx, img_path in enumerate(tqdm(img_files, desc=f"Processing {Path(sequence_path).name}")):
        frame = cv2.imread(img_path)
        if frame is None:
            continue

        detections = tracker.process_frame(frame)

        # Convert to MOT format: frame,id,x,y,w,h,conf,class,visibility
        for track_id, bbox in detections:
            x, y, w, h = bbox
            # MOT format uses 1-based frame indexing
            results.append(f"{frame_idx + 1},{track_id},{x:.2f},{y:.2f},{w:.2f},{h:.2f},1,-1,-1")

    # Save results
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        f.write("\n".join(results))

    print(f"Saved results to {output_path}")


def get_mot_sequences(mot_root: str, split: str = "train") -> List[str]:
    """Get list of MOT16 sequences"""
    split_path = os.path.join(mot_root, split)
    sequences = sorted([d for d in glob.glob(os.path.join(split_path, "MOT16-*")) if os.path.isdir(d)])
    return sequences


def main():
    parser = argparse.ArgumentParser(description="Run tracker benchmark on MOT16 dataset")
    parser.add_argument("--mot-root", required=True, help="Path to MOT16 root folder")
    parser.add_argument("--split", default="train", choices=["train", "test"], help="MOT16 split")
    parser.add_argument("--output-root", required=True, help="Output folder for tracking results")
    parser.add_argument("--trackers", nargs="+", required=True,
                        choices=["botsort", "bytetrack", "norfair",
                                 "new_tracker", "old_tracker", "tracker_node",
                                 "tracker_node_fregoso", "norfair_node", "hector_tracker"],
                        help="Trackers to run")
    parser.add_argument("--yolo-model", default="yolov8n.pt", help="YOLO model to use")
    parser.add_argument("--conf-threshold", type=float, default=0.6, help="Confidence threshold")

    args = parser.parse_args()

    # Get sequences
    sequences = get_mot_sequences(args.mot_root, args.split)
    print(f"Found {len(sequences)} sequences in {args.split} split")

    # Run each tracker
    for tracker_name in args.trackers:
        print(f"\n{'='*80}")
        print(f"Running tracker: {tracker_name}")
        print(f"{'='*80}\n")

        # Create tracker output folder
        tracker_output = os.path.join(args.output_root, tracker_name)
        os.makedirs(tracker_output, exist_ok=True)

        # Initialize tracker
        if tracker_name == "norfair":
            if not NORFAIR_AVAILABLE:
                print(f"Skipping {tracker_name}: Norfair not installed")
                continue
            tracker = NorfairTrackerWrapper(
                model_path=args.yolo_model,
                conf_threshold=args.conf_threshold,
            )
        elif tracker_name in ["botsort", "bytetrack"]:
            tracker = UltralyticsTrackerWrapper(
                model_path=args.yolo_model,
                tracker_config=f"{tracker_name}.yaml",
                conf_threshold=args.conf_threshold,
            )
        elif tracker_name == "new_tracker":
            if not ROS_TRACKERS_AVAILABLE:
                print(f"Skipping {tracker_name}: ROS tracker wrappers not available")
                continue
            tracker = ROSNewTracker(conf_threshold=args.conf_threshold)
        elif tracker_name == "old_tracker":
            if not ROS_TRACKERS_AVAILABLE:
                print(f"Skipping {tracker_name}: ROS tracker wrappers not available")
                continue
            tracker = ROSOldTracker(conf_threshold=args.conf_threshold)
        elif tracker_name == "tracker_node":
            if not ROS_TRACKERS_AVAILABLE:
                print(f"Skipping {tracker_name}: ROS tracker wrappers not available")
                continue
            tracker = ROSTrackerNode(conf_threshold=args.conf_threshold)
        elif tracker_name == "tracker_node_fregoso":
            if not ROS_TRACKERS_AVAILABLE:
                print(f"Skipping {tracker_name}: ROS tracker wrappers not available")
                continue
            tracker = ROSTrackerNodeFregoso(conf_threshold=0.8)  # Fregoso uses 0.8
        elif tracker_name == "norfair_node":
            if not ROS_TRACKERS_AVAILABLE or not NORFAIR_AVAILABLE:
                print(f"Skipping {tracker_name}: Norfair or ROS wrappers not available")
                continue
            tracker = ROSNorfairNode(conf_threshold=args.conf_threshold)
        elif tracker_name == "hector_tracker":
            if not ROS_TRACKERS_AVAILABLE:
                print(f"Skipping {tracker_name}: ROS tracker wrappers not available")
                continue
            tracker = ROSHectorTracker(conf_threshold=0.4)  # DeepSORT typically uses lower threshold
        else:
            print(f"Unknown tracker: {tracker_name}")
            continue

        # Run on each sequence
        for seq_path in sequences:
            seq_name = os.path.basename(seq_path)
            output_file = os.path.join(tracker_output, f"{seq_name}.txt")

            run_tracker_on_sequence(tracker, seq_path, output_file)

    print(f"\n{'='*80}")
    print("Benchmark complete! Now run evaluation:")
    print(f"python benchmark_trackers.py --gt {os.path.join(args.mot_root, args.split, 'gt')} \\")

    for tracker_name in args.trackers:
        print(f"  --trackers {tracker_name}:{os.path.join(args.output_root, tracker_name)} \\")

    print("  --out benchmark_results.csv")
    print(f"{'='*80}\n")


if __name__ == "__main__":
    main()
