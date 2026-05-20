#!/bin/bash
# setup_contact_graspnet.sh
INITIAL_FOLDER="$(pwd)"
WORKSPACE_DIR="/workspace/src"
CGN_DIR="$WORKSPACE_DIR/manipulation/packages/contact_graspnet"

# Check if Contact-GraspNet is in workspace
if [ ! -d "$CGN_DIR" ]; then
    echo "[CGN] Contact-GraspNet submodule not found in $CGN_DIR"
    echo "[CGN] Run: git submodule update --init manipulation/packages/contact_graspnet"
    exit 1
fi

# Check for checkpoints/weights
# Expected path: contact_graspnet/checkpoints/contact_graspnet/checkpoints/model.pt
CHECKPOINT_DIR="$CGN_DIR/checkpoints/contact_graspnet"
if [ ! -f "$CHECKPOINT_DIR/checkpoints/model.pt" ]; then
    echo "[CGN] WARNING: model checkpoint not found at $CHECKPOINT_DIR/checkpoints/model.pt"
    echo "[CGN] Download from: https://github.com/elchun/contact_graspnet_pytorch (Releases)"
    echo "[CGN] and place it at: $CHECKPOINT_DIR/checkpoints/model.pt"
fi

# Install Contact-GraspNet as a Python package.
# Skip torch/torchvision/torchaudio on Jetson — JetPack provides the correct ARM CUDA build.
cd "$CGN_DIR"
if [ -f "requirements.txt" ]; then
    echo "[CGN] Installing Contact-GraspNet dependencies..."
    grep -vE "^torch(vision|audio)?$" requirements.txt > /tmp/cgn_reqs.txt
    pip3 install -q -r /tmp/cgn_reqs.txt
fi

# Ensure these are installed even if missing from submodule requirements.txt
pip3 install -q trimesh tqdm

if [ -f "setup.py" ]; then
    echo "[CGN] Installing Contact-GraspNet python package..."
    pip3 install -q -e .
fi

echo "[CGN] Setup complete."
cd "$INITIAL_FOLDER"
