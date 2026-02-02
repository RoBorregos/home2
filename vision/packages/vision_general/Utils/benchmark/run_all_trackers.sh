#!/bin/bash
# Run benchmark on all available trackers including ROS node implementations

set -e  # Exit on error

# Configuration
MOT_ROOT="${1:-/home/lalo/Desktop/home2/vision/MOT16}"
SPLIT="${2:-train}"
OUTPUT_ROOT="${3:-./full_benchmark_output}"
YOLO_MODEL="${4:-yolov8n.pt}"

# All available trackers (OCSORT removed - config not available)
ALL_TRACKERS="botsort bytetrack new_tracker old_tracker tracker_node tracker_node_fregoso norfair_node hector_tracker"

echo "=========================================="
echo "Complete Tracker Benchmark - All Trackers"
echo "=========================================="
echo "MOT16 Root: $MOT_ROOT"
echo "Split: $SPLIT"
echo "Output Root: $OUTPUT_ROOT"
echo "YOLO Model: $YOLO_MODEL"
echo "Trackers: $ALL_TRACKERS"
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
echo "[2/3] Running all trackers on MOT16 sequences..."
echo "This may take a while depending on your hardware..."
echo ""

python3 run_tracker_benchmark.py \
    --mot-root "$MOT_ROOT" \
    --split "$SPLIT" \
    --output-root "$TRACKER_DIR" \
    --trackers $ALL_TRACKERS \
    --yolo-model "$YOLO_MODEL" \
    --conf-threshold 0.6

echo ""
echo "[All trackers complete]"
echo ""

# Step 3: Evaluate results
echo "[3/3] Evaluating tracker performance..."

# Build tracker arguments for benchmark_trackers.py
TRACKER_ARGS=""
for tracker in $ALL_TRACKERS; do
    tracker_path="$TRACKER_DIR/$tracker"
    if [ -d "$tracker_path" ]; then
        TRACKER_ARGS="$TRACKER_ARGS ${tracker}:${tracker_path}"
    fi
done

if [ -z "$TRACKER_ARGS" ]; then
    echo "Error: No tracker results found!"
    exit 1
fi

python3 benchmark_trackers.py \
    --gt "$GT_DIR" \
    --trackers $TRACKER_ARGS \
    --out "$RESULTS_DIR/benchmark_summary.csv"

echo ""
echo "=========================================="
echo "Complete Benchmark Finished!"
echo "=========================================="
echo ""
echo "Results summary: $RESULTS_DIR/benchmark_summary.csv"
echo ""
echo "Individual tracker results:"
for tracker in $ALL_TRACKERS; do
    result_file="$RESULTS_DIR/benchmark_summary_${tracker}.csv"
    if [ -f "$result_file" ]; then
        echo "  ✓ $tracker: $result_file"
    else
        echo "  ✗ $tracker: Not available"
    fi
done
echo ""
echo "=========================================="
echo ""
echo "To view results:"
echo "  cat $RESULTS_DIR/benchmark_summary.csv"
echo ""
echo "To compare specific trackers:"
echo "  python3 -c \"import pandas as pd; df=pd.read_csv('$RESULTS_DIR/benchmark_summary.csv'); print(df.to_string())\""
echo ""
