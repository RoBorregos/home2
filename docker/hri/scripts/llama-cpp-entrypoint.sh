#!/bin/bash

echo "Starting llama.cpp servers with ROLE=$ROLE"

# Locate the server binary (path differs between dustynv and ghcr images)
if command -v llama-server &>/dev/null; then
    LLAMA_BIN=llama-server
elif [ -f /server ]; then
    LLAMA_BIN=/server
else
    echo "ERROR: llama-server binary not found in PATH or at /server"
    exit 1
fi

MODELS_DIR=/models

wait_for_server() {
    local port=$1
    local max_attempts=30
    local attempt=0
    echo "Waiting for llama.cpp server on port $port..."
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
    exit 0
fi

# qwen3 on port 11434 — used by hric, carry, and gpsr
if [ "$ROLE" = "hric" ] || [ "$ROLE" = "carry" ] || [ "$ROLE" = "gpsr" ]; then
    echo "Starting qwen3 server on port 11434..."
    $LLAMA_BIN \
        --model "$MODELS_DIR/qwen3.Q4_K_M.gguf" \
        --host 0.0.0.0 \
        --port 11434 \
        --ctx-size 8192 \
        -ngl 99 \
        --alias qwen3 \
        &
    wait_for_server 11434
fi

# rbrgs on port 11435 — only needed for gpsr (command interpreter)
if [ "$ROLE" = "gpsr" ]; then
    echo "Starting rbrgs server on port 11435..."
    $LLAMA_BIN \
        --model "$MODELS_DIR/rbrgs.F16.gguf" \
        --host 0.0.0.0 \
        --port 11435 \
        --ctx-size 4096 \
        -ngl 99 \
        --alias rbrgs \
        --temp 1.5 \
        --min-p 0.1 \
        &
    wait_for_server 11435
fi

echo "All llama.cpp servers started. Container ready."
