#!/bin/bash
set -e

# On Jetson, the real libcuda.so.1 is mounted by nvidia-container-runtime at this path.
# Without this, llama.cpp finds the stub in /usr/local/cuda/lib64/stubs/ and falls back to CPU.
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH:-}

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

if [ "$ROLE" = "storing" ]; then
    echo "Storing role: no models to load."
    tail -f /dev/null
fi

# qwen3-8b on port 11434 — hric, carry, gpsr
if [ "$ROLE" = "hric" ] || [ "$ROLE" = "carry" ] || [ "$ROLE" = "gpsr" ]; then
    echo "Starting qwen3-8b on port 11434..."
    llama-server \
        --model "$MODELS_DIR/qwen3-8b.Q4_K_M.gguf" \
        --host 0.0.0.0 \
        --port 11434 \
        --ctx-size 2048 \
        -ngl 99 \
        --flash-attn on \
        --cache-type-k q8_0 \
        --cache-type-v q8_0 \
        --parallel 1 \
        --alias qwen3 \
        &
    wait_for_server 11434
fi

# rbrgs on port 11435 — gpsr only
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
