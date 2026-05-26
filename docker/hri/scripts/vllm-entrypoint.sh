#!/bin/bash
# vLLM entrypoint — serves Qwen3 30B-A3B (MoE) via OpenAI-compatible API.
#
# Designed for Jetson Orin AGX via jetson-containers' dustynv/vllm image.
# vLLM's --quantization awq path is required because we don't have FP16
# weights and an AWQ build of Qwen3-30B-A3B is the smallest practical fit
# on Orin AGX (~17–18 GB after KV cache).

set -e

echo "Starting vLLM with ROLE=$ROLE"

PORT="${VLLM_PORT:-11436}"
MODEL="${VLLM_MODEL:-Qwen/Qwen3-30B-A3B-AWQ}"
MODEL_ALIAS="${VLLM_MODEL_ALIAS:-qwen3}"

# vLLM's OpenAI server uses /health, same as llama-server.
wait_for_vllm() {
    local max_attempts=120  # vLLM cold-start on Orin can take minutes
    local attempt=0
    echo "Waiting for vLLM on port $PORT (cold-start can take several minutes)..."
    while ! curl -sf "http://localhost:$PORT/health" >/dev/null 2>&1; do
        attempt=$((attempt + 1))
        if [ $attempt -ge $max_attempts ]; then
            echo "ERROR: vLLM did not start after $max_attempts attempts"
            exit 1
        fi
        echo "  attempt $attempt/$max_attempts..."
        sleep 5
    done
    echo "vLLM ready on port $PORT."
}

if [ "$ROLE" = "hric" ] || [ "$ROLE" = "gpsr" ] || [ "$ROLE" = "benchmark" ]; then
    echo "Launching vLLM serving $MODEL on port $PORT..."
    vllm serve "$MODEL" \
        --host 0.0.0.0 \
        --port "$PORT" \
        --served-model-name "$MODEL_ALIAS" \
        --quantization awq \
        --dtype float16 \
        --max-model-len 4096 \
        --gpu-memory-utilization 0.85 \
        --enforce-eager \
        --trust-remote-code \
        &

    wait_for_vllm
else
    echo "Unknown ROLE: $ROLE — not starting vLLM"
    exec tail -f /dev/null
fi

echo "vLLM ready. Container running..."
wait
