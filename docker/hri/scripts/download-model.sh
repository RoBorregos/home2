#!/bin/sh

# Download model and Modelfile to the directory where this script is located
SCRIPT_DIR="../../hri/packages/nlp/assets"
[ ! -f "$SCRIPT_DIR/rbrgs.F16.gguf" ] && curl -L https://huggingface.co/diegohc/rbrgs-finetuning/resolve/paraphrased-dataset/q4/unsloth.Q4_K_M.gguf -o "$SCRIPT_DIR/rbrgs.F16.gguf"

# Detect available image
if docker images | grep -q "dustynv/ollama"; then
    IMAGE="dustynv/ollama:0.6.8-r36.4"
    COMMAND="ollama serve"
elif docker images | grep -q "ollama/ollama"; then
    IMAGE="ollama/ollama"
    COMMAND=""
else
    echo "Error: No compatible Ollama image found. Pulling the default image..."
    docker pull ollama/ollama:latest
    IMAGE="ollama/ollama"
    COMMAND=""
fi

echo "Running: docker run -d --rm --runtime=nvidia -v \"$SCRIPT_DIR\":/ollama -e OLLAMA_MODELS=/ollama $IMAGE $COMMAND"

# Don't quote $COMMAND to allow for multiple word commands
CONTAINER_ID=$(docker run -d --rm --runtime=nvidia -v "$SCRIPT_DIR":/ollama -e OLLAMA_MODELS=/ollama "$IMAGE" $COMMAND)

docker exec "$CONTAINER_ID" ollama pull qwen3
docker exec "$CONTAINER_ID" ollama pull nomic-embed-text
docker exec "$CONTAINER_ID" ollama create -f /ollama/Modelfile rbrgs

docker stop "$CONTAINER_ID"

# ── DeepFilterNet (noise cancellation) package ───────────────────────────────
# Install the Python package files into a local folder so `import df` works
DF_TARGET="../../hri/packages/speech/assets/downloads/DeepFilterNet"
mkdir -p "$DF_TARGET"

if [ -z "$(ls -A "$DF_TARGET" 2>/dev/null)" ]; then
    echo "Installing DeepFilterNet package into $DF_TARGET..."
    DF_TARGET_ABS=$(cd "$DF_TARGET" && pwd)

    # Try to use HRI image (if it exists) to perform the installation locally
    HRI_IMAGE=$(docker images --format "{{.Repository}}:{{.Tag}}" \
        | grep "roborregos/home2:hri-" | grep -v "<none>" | head -1)

    if [ -n "$HRI_IMAGE" ]; then
        echo "Using HRI image to install DeepFilterNet files: $HRI_IMAGE"
        docker run --rm \
            -v "$DF_TARGET_ABS":/df_target \
            "$HRI_IMAGE" \
            bash -lc "python3 -m pip install --no-deps --target /df_target deepfilternet"
    else
        echo "HRI image not found. Falling back to python:3.10-slim (will install package files)..."
        docker run --rm \
            -v "$DF_TARGET_ABS":/df_target \
            python:3.10-slim \
            bash -lc "pip install --no-deps --target /df_target deepfilternet torch --index-url https://download.pytorch.org/whl/cpu"
    fi

    # Verify `df` package is present
    if [ -d "$DF_TARGET/df" ] || [ -f "$DF_TARGET/df/__init__.py" ]; then
        echo "DeepFilterNet package installed at $DF_TARGET"
    else
        echo "Warning: 'df' package not found in $DF_TARGET. Installation may have failed."
    fi
else
    echo "DeepFilterNet package already present at $DF_TARGET. Skipping installation."
fi
