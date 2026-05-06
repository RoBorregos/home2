#!/bin/bash
set -e

echo "Starting with ROLE=$ROLE"

llama-server --port 11434 --models-preset /scripts/llama-models-preset.ini &
LLAMA_PID=$!

max_attempts=30
attempt=0

echo "Waiting for llama-server to start..."
while ! curl -sf http://localhost:11434/health &>/dev/null; do
  attempt=$((attempt + 1))
  if [ $attempt -ge $max_attempts ]; then
    echo "Failed to connect to llama-server after $max_attempts attempts. Exiting."
    exit 1
  fi
  echo "Waiting for llama-server (attempt $attempt/$max_attempts)..."
  sleep 2
done

echo "llama-server is up and running."

if [ "$ROLE" = "hric" ]; then
  curl -sf -X POST http://localhost:11434/models/load -d '{"model": "qwen3"}'
elif [ "$ROLE" = "carry" ]; then
  echo "Carry role: embeddings model skipped, no models loaded."
elif [ "$ROLE" = "gpsr" ]; then
  curl -sf -X POST http://localhost:11434/models/load -d '{"model": "qwen3"}'
  curl -sf -X POST http://localhost:11434/models/load -d '{"model": "rbrgs"}'
elif [ "$ROLE" = "storing" ]; then
  echo "Storing role detected, not loading any models..."
else
  echo "Unknown ROLE: $ROLE"
fi

echo "llama-server models loaded. Container will continue to run..."
wait $LLAMA_PID
