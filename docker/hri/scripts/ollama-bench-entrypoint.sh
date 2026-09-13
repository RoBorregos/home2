#!/bin/bash
# Benchmark-only Ollama entrypoint: serves the SAME GGUF llama.cpp is given,
# imported via a generated Modelfile, so the comparison is weight-for-weight.
set -e

MODELS_DIR=/ollama
MODEL_FILE="${LLAMA_MODEL_FILE:-qwen3-4b.Q4_K_M.gguf}"
ALIAS="${LLAMA_ALIAS:-qwen3}"
CTX="${LLAMA_CTX_SIZE:-2048}"
PORT="${BENCH_PORT:-11434}"

export OLLAMA_HOST="0.0.0.0:$PORT"

if [ ! -f "$MODELS_DIR/$MODEL_FILE" ]; then
    echo "ERROR: $MODELS_DIR/$MODEL_FILE not found. Download it first."
    exit 1
fi

ollama serve &

echo "Waiting for Ollama on port $PORT..."
attempt=0
while ! curl -sf "http://localhost:$PORT/api/version" >/dev/null 2>&1; do
    attempt=$((attempt + 1))
    if [ $attempt -ge 30 ]; then
        echo "ERROR: Ollama did not start after 30 attempts."
        exit 1
    fi
    sleep 2
done
echo "Ollama is up."

cat > /tmp/Modelfile.bench <<MODELFILE
FROM $MODELS_DIR/$MODEL_FILE
PARAMETER num_ctx $CTX
MODELFILE

echo "Importing $MODEL_FILE as '$ALIAS'..."
ollama create "$ALIAS" -f /tmp/Modelfile.bench

curl -sf "http://localhost:$PORT/api/generate" \
    -d "{\"model\": \"$ALIAS\", \"keep_alive\": -1}" >/dev/null

echo "Ollama bench server ready (model=$ALIAS, ctx=$CTX)."
tail -f /dev/null
