#!/bin/bash
# setup_contact_graspnet.sh
INITIAL_FOLDER="$(pwd)"
WORKSPACE_DIR="/workspace/src"
CGN_DIR="$WORKSPACE_DIR/manipulation/packages/contact_graspnet"

# Check if Contact-GraspNet is in workspace
if [ ! -d "$CGN_DIR" ]; then
    echo "Contact-GraspNet submodule not found in $CGN_DIR"
    exit 1
fi

# Check for checkpoints/weights
CHECKPOINT_DIR="$CGN_DIR/checkpoints/contact_graspnet"
if [ ! -d "$CHECKPOINT_DIR" ]; then
    echo "Contact-GraspNet weights not found. You may need to download them."
    # Optional: Add download logic here if there is a public link
fi

# PointNet++ compilation (if needed)
# The elchun implementation uses some CUDA extensions that might need building
cd "$CGN_DIR"
if [ -f "requirements.txt" ]; then
    echo "Installing missing dependencies from submodule..."
    pip3 install -r requirements.txt
fi

if [ -f "setup.py" ]; then
    echo "Installing Contact-GraspNet python package and compiling extensions..."
    pip3 install -e .
fi

cd "$INITIAL_FOLDER"
