#!/bin/bash

curl -L https://huggingface.co/diegohc/robollm/resolve/main/rbrgs-finetuned-unsloth.F16.gguf -o rbrgs-finetuned.F16.gguf
curl -L https://huggingface.co/diegohc/robollm/resolve/main/Modelfile -o Modelfile

# Detect available image
if docker images | grep -q "dustynv/ollama"; then
    IMAGE="dustynv/ollama:r36.4.0"
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

echo "Running: docker run -d --rm --runtime=nvidia -v $(pwd):/ollama $IMAGE $COMMAND"

if [[ -n "$COMMAND" ]]; then
    CONTAINER_ID=$(docker run -d --rm --runtime=nvidia -v "$(pwd)":/ollama "$IMAGE" $COMMAND)
else
    CONTAINER_ID=$(docker run -d --rm --runtime=nvidia -v "$(pwd)":/ollama "$IMAGE")
fi

docker exec "$CONTAINER_ID" ollama create -f /ollama/Modelfile robollm
docker exec "$CONTAINER_ID" ollama pull nomic-embed-text

docker stop "$CONTAINER_ID"
