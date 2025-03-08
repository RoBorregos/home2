#!/bin/bash

curl -L https://huggingface.co/diegohc/robollm/resolve/main/rbrgs-finetuned-unsloth.F16.gguf -o rbrgs-finetuned.F16.gguf
curl -L https://huggingface.co/diegohc/robollm/resolve/main/Modelfile -o Modelfile

# Detect available image
if docker images | grep -q "dustynv/ollama"; then
    IMAGE="dustynv/ollama"
elif docker images | grep -q "ollama/ollama"; then
    IMAGE="ollama/ollama"
else
    echo "Error: No compatible Ollama image found. Pulling the default image..."
    docker pull ollama/ollama:latest
    IMAGE="ollama/ollama"
fi

echo "Using Docker image: $IMAGE"

CONTAINER_ID=$(docker run -d --rm --runtime=nvidia -v ./:/ollama "$IMAGE")
docker exec -it "$CONTAINER_ID" ollama create -f /ollama/Modelfile testtest
docker exec -it "$CONTAINER_ID" ollama pull nomic-embed-text
docker stop "$CONTAINER_ID"
