#!/usr/bin/env python3
"""
Prepare MOT16 ground truth for benchmarking.
Copies ground truth files from MOT16 structure to a flat folder structure expected by benchmark_trackers.py
"""

import argparse
import os
import glob
import shutil


def prepare_ground_truth(mot_root: str, split: str, output_dir: str) -> None:
    """
    Copy ground truth files from MOT16 structure to flat folder.

    MOT16 structure: MOT16/train/MOT16-XX/gt/gt.txt
    Output structure: output_dir/MOT16-XX.txt
    """
    split_path = os.path.join(mot_root, split)
    sequences = sorted(
        [d for d in glob.glob(os.path.join(split_path, "MOT16-*")) if os.path.isdir(d)]
    )

    os.makedirs(output_dir, exist_ok=True)

    for seq_path in sequences:
        seq_name = os.path.basename(seq_path)
        gt_file = os.path.join(seq_path, "gt", "gt.txt")

        if not os.path.exists(gt_file):
            print(f"Warning: Ground truth not found for {seq_name}")
            continue

        # Copy to flat structure
        output_file = os.path.join(output_dir, f"{seq_name}.txt")
        shutil.copy(gt_file, output_file)
        print(f"Copied {seq_name} ground truth")

    print(f"\nGround truth prepared in: {output_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Prepare MOT16 ground truth for benchmarking"
    )
    parser.add_argument("--mot-root", required=True, help="Path to MOT16 root folder")
    parser.add_argument(
        "--split", default="train", choices=["train", "test"], help="MOT16 split"
    )
    parser.add_argument(
        "--output", required=True, help="Output folder for ground truth files"
    )

    args = parser.parse_args()

    prepare_ground_truth(args.mot_root, args.split, args.output)


if __name__ == "__main__":
    main()
