#!/bin/bash
set -e

echo "Starting with ROLE=$ROLE"
# Wait for Ollama service to be available

max_attempts=30
attempt=0
ollama serve &
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

GENERAL_MODEL="${OLLAMA_MODEL:-qwen3.5:4b}"

ensure_general_model() {
  if ! ollama show "$GENERAL_MODEL" >/dev/null 2>&1; then
    echo "ERROR: required Ollama model '$GENERAL_MODEL' is not mounted in $OLLAMA_MODELS"
    echo "Run docker/hri/scripts/download-model.sh and select qwen3.5."
    exit 1
  fi

  echo "Updating frida-llm alias from $GENERAL_MODEL..."
  ollama cp "$GENERAL_MODEL" frida-llm
  ollama show frida-llm >/dev/null
}

if [ "$ROLE" = "hric" ] || [ "$ROLE" = "gpsr" ] || [ "$ROLE" = "restaurant" ]; then
  ensure_general_model
fi

if [ "$ROLE" = "hric" ] || [ "$ROLE" = "restaurant" ]; then
  curl -fsS http://localhost:11434/api/generate -d '{"model": "frida-llm", "keep_alive": -1}'
elif [ "$ROLE" = "carry" ]; then
  curl -fsS http://localhost:11434/api/embeddings -d '{"model": "nomic-embed-text", "keep_alive": -1}'
elif [ "$ROLE" = "gpsr" ]; then
  curl -fsS http://localhost:11434/api/generate -d '{"model": "frida-llm", "keep_alive": -1}'
  curl -fsS http://localhost:11434/api/generate -d '{"model": "rbrgs", "keep_alive": -1}'
elif [ "$ROLE" = "storing" ]; then
  echo "Storing role detected, not loading any models..."
else
  echo "Unknown ROLE: $ROLE"
fi

echo "Ollama models loaded. Container will continue to run..."
