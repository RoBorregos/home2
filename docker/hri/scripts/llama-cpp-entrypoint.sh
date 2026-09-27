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
    local model=$3
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
    echo "llama-server ready: $model as '$alias' on :$port (ROLE=$ROLE)"
}

# Main model on port 11434. LLAMA_MODEL_FILE / LLAMA_ALIAS override the defaults
# from the benchmark flow without changing this script.
MAIN_MODEL="${LLAMA_MODEL_FILE:-Qwen3.5-4B-UD-Q4_K_XL.gguf}"
# Version-neutral alias: the GGUF filename is the only place the model version lives.
MAIN_ALIAS="${LLAMA_ALIAS:-frida-llm}"
MAIN_CTX="${LLAMA_CTX_SIZE:-2048}"
# Keep the correctness-first llama.cpp default until q8_0 is validated on the
# target Jetson; set LLAMA_CACHE_TYPE=q8_0 for that A/B comparison.
CACHE_TYPE="${LLAMA_CACHE_TYPE:-f16}"

if [ "$ROLE" = "hric" ] || [ "$ROLE" = "gpsr" ] || [ "$ROLE" = "restaurant" ] || [ "$ROLE" = "bench" ]; then
    if [ ! -s "$MODELS_DIR/$MAIN_MODEL" ]; then
        echo "ERROR: model not found or empty: $MODELS_DIR/$MAIN_MODEL"
        exit 1
    fi
    echo "llama.cpp serving $MAIN_MODEL (ctx=$MAIN_CTX, cache=$CACHE_TYPE, alias=$MAIN_ALIAS)"
    llama-server \
        --model "$MODELS_DIR/$MAIN_MODEL" \
        --host 0.0.0.0 \
        --port 11434 \
        --ctx-size "$MAIN_CTX" \
        -ngl 99 \
        --flash-attn on \
        --jinja \
        --reasoning off \
        --cache-type-k "$CACHE_TYPE" \
        --cache-type-v "$CACHE_TYPE" \
        --parallel 1 \
        --alias "$MAIN_ALIAS" \
        &
    wait_for_server 11434 "$MAIN_ALIAS" "$MAIN_MODEL"
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
    wait_for_server 11435 rbrgs rbrgs.F16.gguf
fi

wait
