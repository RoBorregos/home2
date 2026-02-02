#!/bin/bash
# Complete benchmark pipeline for MOT16 tracker evaluation

set -e  # Exit on error

# Configuration
MOT_ROOT="${1:-/home/lalo/Desktop/home2/vision/MOT16}"
SPLIT="${2:-train}"
OUTPUT_ROOT="${3:-./benchmark_output}"
YOLO_MODEL="${4:-yolov8n.pt}"

# Trackers to benchmark
TRACKERS="botsort bytetrack"

echo "=========================================="
echo "MOT16 Tracker Benchmark Pipeline"
echo "=========================================="
echo "MOT16 Root: $MOT_ROOT"
echo "Split: $SPLIT"
echo "Output Root: $OUTPUT_ROOT"
echo "YOLO Model: $YOLO_MODEL"
echo "Trackers: $TRACKERS"
echo "=========================================="
echo ""

# Create output directories
mkdir -p "$OUTPUT_ROOT"
GT_DIR="$OUTPUT_ROOT/ground_truth"
TRACKER_DIR="$OUTPUT_ROOT/tracker_results"
RESULTS_DIR="$OUTPUT_ROOT/results"

mkdir -p "$GT_DIR" "$TRACKER_DIR" "$RESULTS_DIR"

# Step 1: Prepare ground truth
echo "[1/3] Preparing ground truth..."
python3 prepare_mot_gt.py \
    --mot-root "$MOT_ROOT" \
    --split "$SPLIT" \
    --output "$GT_DIR"

echo ""
echo "[Ground truth prepared]"
echo ""

# Step 2: Run trackers
echo "[2/3] Running trackers on MOT16 sequences..."
python3 run_tracker_benchmark.py \
    --mot-root "$MOT_ROOT" \
    --split "$SPLIT" \
    --output-root "$TRACKER_DIR" \
    --trackers $TRACKERS \
    --yolo-model "$YOLO_MODEL" \
    --conf-threshold 0.6

echo ""
echo "[Trackers complete]"
echo ""

# Step 3: Evaluate results
echo "[3/3] Evaluating tracker performance..."

# Build tracker arguments for benchmark_trackers.py
TRACKER_ARGS=""
for tracker in $TRACKERS; do
    tracker_path="$TRACKER_DIR/$tracker"
    if [ -d "$tracker_path" ]; then
        TRACKER_ARGS="$TRACKER_ARGS ${tracker}:${tracker_path}"
    fi
done

python3 benchmark_trackers.py \
    --gt "$GT_DIR" \
    --trackers $TRACKER_ARGS \
    --out "$RESULTS_DIR/benchmark_summary.csv"

echo ""
echo "=========================================="
echo "Benchmark Complete!"
echo "=========================================="
echo "Results saved to: $RESULTS_DIR/benchmark_summary.csv"
echo ""
echo "Individual tracker results:"
for tracker in $TRACKERS; do
    result_file="$RESULTS_DIR/benchmark_summary_${tracker}.csv"
    if [ -f "$result_file" ]; then
        echo "  - $tracker: $result_file"
    fi
done
echo "=========================================="
