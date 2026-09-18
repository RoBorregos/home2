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

# The ROS nodes ask for the frida-llm alias. Unlike llama-server, Ollama rejects
# an unknown model name, so make sure the tag exists before preloading.
if ! ollama list | grep -q "^frida-llm"; then
  echo "Creating frida-llm alias from qwen3.5..."
  ollama cp qwen3.5 frida-llm || echo "WARNING: could not create frida-llm alias"
fi

if [ "$ROLE" = "hric" ]; then
  curl http://localhost:11434/api/generate -d '{"model": "frida-llm", "keep_alive": -1}'
elif [ "$ROLE" = "carry" ]; then
  curl http://localhost:11434/api/embeddings -d '{"model": "nomic-embed-text", "keep_alive": -1}'
elif [ "$ROLE" = "gpsr" ]; then
  curl http://localhost:11434/api/generate -d '{"model": "frida-llm", "keep_alive": -1}'
  curl http://localhost:11434/api/generate -d '{"model": "rbrgs", "keep_alive": -1}'
elif [ "$ROLE" = "storing" ]; then
  echo "Storing role detected, not loading any models..."
else
  echo "Unknown ROLE: $ROLE"
fi

echo "Ollama models loaded. Container will continue to run..."
