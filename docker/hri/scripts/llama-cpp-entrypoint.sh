#!/bin/bash
set -e

echo "Starting with ROLE=$ROLE"

MODELS_DIR=/root/.cache/huggingface

wait_for_server() {
    local port=$1
    local max_attempts=30
    local attempt=0
    echo "Waiting for llama-server on port $port..."
    while ! curl -sf "http://localhost:$port/health" >/dev/null 2>&1; do
        attempt=$((attempt + 1))
        if [ $attempt -ge $max_attempts ]; then
            echo "ERROR: server on port $port did not start after $max_attempts attempts"
            exit 1
        fi
        echo "  attempt $attempt/$max_attempts..."
        sleep 2
    done
    echo "Server on port $port is ready."
}

# Main model on port 11434. LLAMA_MODEL_FILE / LLAMA_ALIAS override the defaults
# from the benchmark flow without changing this script.
MAIN_MODEL="${LLAMA_MODEL_FILE:-qwen3-4b.Q4_K_M.gguf}"
MAIN_ALIAS="${LLAMA_ALIAS:-qwen3}"

if [ "$ROLE" = "hric" ] || [ "$ROLE" = "gpsr" ] || [ "$ROLE" = "bench" ]; then
    echo "Starting $MAIN_MODEL on port 11434 (alias=$MAIN_ALIAS)..."
    llama-server \
        --model "$MODELS_DIR/$MAIN_MODEL" \
        --host 0.0.0.0 \
        --port 11434 \
        --ctx-size 2048 \
        -ngl 99 \
        --flash-attn on \
        --cache-type-k q8_0 \
        --cache-type-v q8_0 \
        --parallel 1 \
        --alias "$MAIN_ALIAS" \
        &
    wait_for_server 11434
fi

# rbrgs on port 11435, gpsr only
if [ "$ROLE" = "gpsr" ]; then
    echo "Starting rbrgs on port 11435..."
    llama-server \
        --model "$MODELS_DIR/rbrgs.F16.gguf" \
        --host 0.0.0.0 \
        --port 11435 \
        --ctx-size 4096 \
        -ngl 99 \
        --flash-attn on \
        --parallel 1 \
        --alias rbrgs \
        --temp 1.5 \
        --min-p 0.1 \
        &
    wait_for_server 11435
fi

echo "All servers ready. Container running..."
wait
