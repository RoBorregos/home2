#!/bin/bash
set -e

echo "Starting with ROLE=$ROLE"
# Wait for Ollama service to be available

max_attempts=30
attempt=0
ollama serve&
echo "Waiting for Ollama service to start..."

while ! curl -s http://localhost:11434/api/version &>/dev/null; do
  attempt=$((attempt + 1))
  if [ $attempt -ge $max_attempts ]; then
    echo "Failed to connect to Ollama after $max_attempts attempts. Exiting."
    exit 1
  fi
  echo "Waiting for Ollama service (attempt $attempt/$max_attempts)..."
  sleep 2
done

echo "Ollama service is up and running."

if [ "$ROLE" = "receptionist" ]; then
  curl http://localhost:11434/api/generate -d '{"model": "qwen3:0.6b", "keep_alive": -1}'
  curl http://localhost:11434/api/generate -d '{"model": "qwen2.5", "keep_alive": -1}'
elif [ "$ROLE" = "carry" ]; then
  curl http://localhost:11434/api/embeddings -d '{"model": "nomic-embed-text", "keep_alive": -1}'
elif [ "$ROLE" = "gpsr" ]; then
  curl http://localhost:11434/api/generate -d '{"model": "qwen2.5", "keep_alive": -1}'
elif [ "$ROLE" = "storing" ]; then
  curl http://localhost:11434/api/generate -d '{"model": "qwen3", "keep_alive": -1}'
else
  echo "Unknown ROLE: $ROLE"
fi

echo "Ollama models loaded. Container will continue to run..."
