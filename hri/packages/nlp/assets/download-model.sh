#!/bin/sh

# Download model and Modelfile to the directory where this script is located
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
[ ! -f "$SCRIPT_DIR/rbrgs/unsloth.Q4_K_M.gguf" ] && curl -L https://huggingface.co/diegohc/rbrgs-finetuning/resolve/paraphrased-dataset/q4/unsloth.Q4_K_M.gguf -o "$SCRIPT_DIR/rbrgs.F16.gguf"

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

CONTAINER_ID=$(docker run -d --rm --runtime=nvidia -v "$SCRIPT_DIR":/ollama -e OLLAMA_MODELS=/ollama "$IMAGE" "$COMMAND")

docker exec "$CONTAINER_ID" ollama pull qwen3
docker exec "$CONTAINER_ID" ollama create -f /ollama/Modelfile rbrgs
# docker exec "$CONTAINER_ID" ollama pull nomic-embed-text
# docker exec "$CONTAINER_ID" ollama pull qwen2.5

docker stop "$CONTAINER_ID"
