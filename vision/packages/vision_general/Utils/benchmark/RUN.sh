#!/bin/bash
# MOT16 Tracker Benchmark - Run ALL Trackers
# Install: pip3 install ultralytics motmetrics opencv-python tqdm scipy pandas pretrainedmodels timm
# Usage: ./RUN.sh              # Run all trackers
#        ./RUN.sh botsort      # Run specific tracker(s)

cd "$(dirname "$0")"

# All available trackers (runs all by default)
ALL_TRACKERS="botsort bytetrack new_tracker old_tracker tracker_node tracker_node_fregoso norfair_node hector_tracker"
TRACKERS="${@:-$ALL_TRACKERS}"
MOT_ROOT="/home/lalo/Desktop/home2/vision/MOT16"
OUTPUT="./output"

echo "=========================================="
echo "MOT16 Tracker Benchmark"
echo "=========================================="
echo "Trackers: $TRACKERS"
echo "MOT16: $MOT_ROOT"
echo "Output: $OUTPUT"
echo "=========================================="
echo ""

python3 benchmark.py \
    --mot-root "$MOT_ROOT" \
    --split train \
    --output "$OUTPUT" \
    --trackers $TRACKERS \
    --model yolov8n.pt \
    --conf 0.6

echo ""
echo "=========================================="
echo "Benchmark Complete!"
echo "=========================================="
echo "Results: $OUTPUT/results/summary.csv"
echo ""
cat "$OUTPUT/results/summary.csv"
