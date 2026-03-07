#!/bin/bash
# Setup Contact-GraspNet checkpoints inside the manipulation container.
# Run this ONCE after the container is up.
#
# Usage: bash setup_graspnet.sh

set -e

GRASPNET_DIR="/workspace/src/manipulation/packages/contact_graspnet"
CHECKPOINT_DIR="$GRASPNET_DIR/checkpoints"

echo "=== Contact-GraspNet setup ==="

# 1. Clone repo if not already there
if [ ! -d "$GRASPNET_DIR" ]; then
    echo "[1/3] Cloning Contact-GraspNet..."
    git clone https://github.com/NVlabs/contact_graspnet "$GRASPNET_DIR"
else
    echo "[1/3] Contact-GraspNet repo already present at $GRASPNET_DIR"
fi

# 2. Download checkpoints if not already present
CHECKPOINT_SUBDIR="$CHECKPOINT_DIR/scene_test_2048_bs3_hor_sigma_001"
if [ ! -d "$CHECKPOINT_SUBDIR" ]; then
    echo "[2/3] Downloading checkpoints (~300MB)..."
    mkdir -p "$CHECKPOINT_DIR"
    pip3 install -q gdown
    # Checkpoint file ID from Contact-GraspNet README
    gdown --id 1tBHKf60K8DLM5arm-Chyf7jxkzOr5zGl -O /tmp/contact_graspnet_checkpoints.tar.gz
    tar -xzf /tmp/contact_graspnet_checkpoints.tar.gz -C "$CHECKPOINT_DIR"
    rm /tmp/contact_graspnet_checkpoints.tar.gz
    echo "[2/3] Checkpoints downloaded to $CHECKPOINT_SUBDIR"
else
    echo "[2/3] Checkpoints already present at $CHECKPOINT_SUBDIR"
fi

# 3. Quick import check
echo "[3/3] Verifying import..."
python3 -c "
import sys
sys.path.insert(0, '$GRASPNET_DIR')
from contact_graspnet.contact_grasp_estimator import GraspEstimator
print('  Contact-GraspNet import OK')
"

echo ""
echo "=== Setup complete! ==="
echo "Run the manipulation stack normally — graspnet_service.py will load the model on startup."
