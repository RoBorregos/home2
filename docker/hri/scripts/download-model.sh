#!/bin/sh

# Download model and Modelfile to the directory where this script is located
SCRIPT_DIR="../../hri/packages/nlp/assets" 
[ ! -f "$SCRIPT_DIR/rbrgs.F16.gguf" ] && curl -L https://huggingface.co/diegohc/rbrgs-finetuning/resolve/paraphrased-dataset/q4/unsloth.Q4_K_M.gguf -o "$SCRIPT_DIR/rbrgs.F16.gguf"

# Download and unzip DeepFilterNet model
DF_MODEL_DIR="../../hri/packages/speech/assets/downloads"
DF_MODEL_URL="https://github.com/Rikorose/DeepFilterNet/raw/main/models/DeepFilterNet3.zip"
ZIP_FILE="$DF_MODEL_DIR/DeepFilterNet3.zip"

if [ ! -d "$DF_MODEL_DIR/DeepFilterNet3" ]; then
    echo "Downloading DeepFilterNet3 model..."
    mkdir -p "$DF_MODEL_DIR"
    curl -L "$DF_MODEL_URL" -o "$ZIP_FILE"
    echo "Unzipping the model..."
    unzip "$ZIP_FILE" -d "$DF_MODEL_DIR"
    rm "$ZIP_FILE"
    echo "DeepFilterNet3 model downloaded successfully."
else
    echo "DeepFilterNet3 model already exists. Skipping download."
fi

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
