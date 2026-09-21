#!/bin/bash
set -e

MODELS_DIR=/root/.cache/huggingface

# Same switch as the ROS nodes: HRI_LOG_LEVEL=debug restores this script's progress
# echoes and llama.cpp's own startup log (model load, CORS banner, etc.).
if [ "${HRI_LOG_LEVEL:-info}" = "debug" ]; then
    export LLAMA_ARG_LOG_VERBOSITY=3
    debug_log() { echo "$@"; }
else
    export LLAMA_ARG_LOG_VERBOSITY=1
    debug_log() { :; }
fi

debug_log "Starting with ROLE=$ROLE"

wait_for_server() {
    local port=$1
    local alias=$2
    local max_attempts=30
    local attempt=0
    debug_log "Waiting for llama-server ($alias) on port $port..."
    while ! curl -sf "http://localhost:$port/health" >/dev/null 2>&1; do
        attempt=$((attempt + 1))
        if [ $attempt -ge $max_attempts ]; then
            echo "ERROR: llama-server ($alias) on port $port did not start after $max_attempts attempts"
            exit 1
        fi
        debug_log "  attempt $attempt/$max_attempts..."
        sleep 2
    done
    echo "llama-server ready: $alias on :$port (ROLE=$ROLE)"
}

# Main model on port 11434. LLAMA_MODEL_FILE / LLAMA_ALIAS override the defaults
# from the benchmark flow without changing this script.
MAIN_MODEL="${LLAMA_MODEL_FILE:-qwen3-4b.Q4_K_M.gguf}"
MAIN_ALIAS="${LLAMA_ALIAS:-qwen3}"
MAIN_CTX="${LLAMA_CTX_SIZE:-2048}"

if [ "$ROLE" = "hric" ] || [ "$ROLE" = "gpsr" ] || [ "$ROLE" = "bench" ]; then
    debug_log "Starting $MAIN_MODEL on port 11434 (alias=$MAIN_ALIAS)..."
    llama-server \
        --model "$MODELS_DIR/$MAIN_MODEL" \
        --host 0.0.0.0 \
        --port 11434 \
        --ctx-size "$MAIN_CTX" \
        -ngl 99 \
        --flash-attn on \
        --cache-type-k q8_0 \
        --cache-type-v q8_0 \
        --parallel 1 \
        --alias "$MAIN_ALIAS" \
        &
    wait_for_server 11434 "$MAIN_ALIAS"
fi

# rbrgs on port 11435, gpsr only
if [ "$ROLE" = "gpsr" ]; then
    debug_log "Starting rbrgs on port 11435..."
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
    wait_for_server 11435 rbrgs
fi

wait
