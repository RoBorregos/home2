#!/usr/bin/env python3
"""
Quick test script to validate tracker benchmark on a single sequence.
"""

import os
import sys
import subprocess
from pathlib import Path

# Configuration
MOT_ROOT = "/home/lalo/Desktop/home2/vision/MOT16"
SPLIT = "train"
TEST_SEQUENCE = "MOT16-02"  # Use first sequence for testing
OUTPUT_ROOT = "./test_benchmark_output"
YOLO_MODEL = "yolov8n.pt"

def test_single_sequence():
    """Test benchmark pipeline on a single sequence"""

    print("="*80)
    print("Testing Tracker Benchmark on Single Sequence")
    print("="*80)
    print(f"Test Sequence: {TEST_SEQUENCE}")
    print(f"Output: {OUTPUT_ROOT}")
    print("="*80)
    print()

    # Check if MOT16 exists
    seq_path = os.path.join(MOT_ROOT, SPLIT, TEST_SEQUENCE)
    if not os.path.exists(seq_path):
        print(f"Error: Sequence not found at {seq_path}")
        print("Please update MOT_ROOT in this script to point to your MOT16 dataset.")
        return False

    # Create output directories
    os.makedirs(OUTPUT_ROOT, exist_ok=True)
    gt_dir = os.path.join(OUTPUT_ROOT, "ground_truth")
    tracker_dir = os.path.join(OUTPUT_ROOT, "tracker_results")
    results_dir = os.path.join(OUTPUT_ROOT, "results")

    os.makedirs(gt_dir, exist_ok=True)
    os.makedirs(tracker_dir, exist_ok=True)
    os.makedirs(results_dir, exist_ok=True)

    try:
        # Step 1: Prepare ground truth
        print("[1/3] Preparing ground truth...")
        result = subprocess.run([
            "python3", "prepare_mot_gt.py",
            "--mot-root", MOT_ROOT,
            "--split", SPLIT,
            "--output", gt_dir
        ], check=True, capture_output=True, text=True)
        print(result.stdout)

        # Verify ground truth was created
        gt_file = os.path.join(gt_dir, f"{TEST_SEQUENCE}.txt")
        if not os.path.exists(gt_file):
            print(f"Error: Ground truth file not created at {gt_file}")
            return False
        print(f"✓ Ground truth prepared: {gt_file}")
        print()

        # Step 2: Run a single tracker (ByteTrack is fast)
        print("[2/3] Running ByteTrack tracker...")
        result = subprocess.run([
            "python3", "run_tracker_benchmark.py",
            "--mot-root", MOT_ROOT,
            "--split", SPLIT,
            "--output-root", tracker_dir,
            "--trackers", "bytetrack",
            "--yolo-model", YOLO_MODEL,
            "--conf-threshold", "0.6"
        ], check=True, capture_output=True, text=True)
        print(result.stdout)

        # Verify tracker output was created
        tracker_file = os.path.join(tracker_dir, "bytetrack", f"{TEST_SEQUENCE}.txt")
        if not os.path.exists(tracker_file):
            print(f"Error: Tracker output not created at {tracker_file}")
            return False
        print(f"✓ Tracker output created: {tracker_file}")

        # Check file size
        file_size = os.path.getsize(tracker_file)
        print(f"  Output file size: {file_size} bytes")

        # Show sample output
        with open(tracker_file, 'r') as f:
            lines = f.readlines()
            print(f"  Number of detections: {len(lines)}")
            if lines:
                print(f"  Sample output (first 3 lines):")
                for line in lines[:3]:
                    print(f"    {line.strip()}")
        print()

        # Step 3: Evaluate
        print("[3/3] Evaluating tracker performance...")
        result = subprocess.run([
            "python3", "benchmark_trackers.py",
            "--gt", gt_dir,
            "--trackers", f"bytetrack:{os.path.join(tracker_dir, 'bytetrack')}",
            "--out", os.path.join(results_dir, "test_summary.csv")
        ], check=True, capture_output=True, text=True)
        print(result.stdout)

        # Check results
        summary_file = os.path.join(results_dir, "test_summary.csv")
        if os.path.exists(summary_file):
            print(f"✓ Evaluation complete: {summary_file}")
            print()
            print("Results:")
            with open(summary_file, 'r') as f:
                print(f.read())

        print()
        print("="*80)
        print("✓ Test completed successfully!")
        print("="*80)
        print()
        print("You can now run the full benchmark with:")
        print("  ./run_complete_benchmark.sh")
        print()
        return True

    except subprocess.CalledProcessError as e:
        print(f"Error running command: {e}")
        print(f"stdout: {e.stdout}")
        print(f"stderr: {e.stderr}")
        return False
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    # Change to script directory
    script_dir = Path(__file__).parent
    os.chdir(script_dir)

    success = test_single_sequence()
    sys.exit(0 if success else 1)
