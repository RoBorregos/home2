#!/bin/bash
# setup_contact_graspnet.sh — initialise Contact-GraspNet submodule and dependencies.
# Sourced from run.sh before colcon build or launch.

INITIAL_FOLDER="$(pwd)"
WORKSPACE_DIR="/workspace/src"
CGN_DIR="$WORKSPACE_DIR/manipulation/packages/contact_graspnet"

# Verify submodule is present
if [ ! -d "$CGN_DIR" ]; then
    echo "[CGN] Submodule not found at $CGN_DIR"
    echo "[CGN] Run inside the container: git submodule update --init manipulation/packages/contact_graspnet"
    exit 1
fi

# Verify model checkpoint
CHECKPOINT_DIR="$CGN_DIR/checkpoints/contact_graspnet"
if [ ! -f "$CHECKPOINT_DIR/checkpoints/model.pt" ]; then
    echo "[CGN] WARNING: model not found at $CHECKPOINT_DIR/checkpoints/model.pt"
    echo "[CGN] Download from the contact_graspnet_pytorch GitHub releases page"
    echo "[CGN] and place model.pt at: $CHECKPOINT_DIR/checkpoints/model.pt"
fi

# Install Python dependencies.
# torch/torchvision/torchaudio are excluded — JetPack supplies the correct CUDA ARM builds.
cd "$CGN_DIR"
if [ -f "requirements.txt" ]; then
    echo "[CGN] Installing dependencies (skipping torch*)..."
    grep -vE "^torch(vision|audio)?($|[>=<])" requirements.txt > /tmp/cgn_reqs.txt
    pip3 install -q -r /tmp/cgn_reqs.txt
fi

# Ensure trimesh and tqdm are present (used by CGN's mesh-handling utilities)
pip3 install -q trimesh tqdm

# Install CGN as an editable package so Python can import contact_graspnet_pytorch
if [ -f "setup.py" ]; then
    echo "[CGN] Installing CGN python package (editable)..."
    pip3 install -q -e .
fi

echo "[CGN] Setup complete."
cd "$INITIAL_FOLDER"
