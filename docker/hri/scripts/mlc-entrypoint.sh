#!/bin/bash
# MLC-LLM entrypoint — serves Qwen3 30B-A3B (MoE) via OpenAI-compatible API.
#
# Uses the prebuilt MLC checkpoint (`mlc-ai/Qwen3-30B-A3B-q4f16_1-MLC`) by
# default — first run downloads and JIT-compiles for the current Orin AGX,
# subsequent runs reuse the cached .so in /root/.cache/mlc_llm.

set -e

echo "Starting MLC-LLM with ROLE=$ROLE"

PORT="${MLC_PORT:-11437}"
MODEL="${MLC_MODEL:-HF://mlc-ai/Qwen3-30B-A3B-q4f16_1-MLC}"
MODEL_ALIAS="${MLC_MODEL_ALIAS:-qwen3}"

wait_for_mlc() {
    local max_attempts=180  # First-run JIT-compile can take 10–30 minutes
    local attempt=0
    echo "Waiting for MLC server on port $PORT (first-run compile can take 10–30 minutes)..."
    while ! curl -sf "http://localhost:$PORT/v1/models" >/dev/null 2>&1; do
        attempt=$((attempt + 1))
        if [ $attempt -ge $max_attempts ]; then
            echo "ERROR: MLC server did not start after $max_attempts attempts"
            exit 1
        fi
        echo "  attempt $attempt/$max_attempts..."
        sleep 10
    done
    echo "MLC server ready on port $PORT."
}

if [ "$ROLE" = "hric" ] || [ "$ROLE" = "gpsr" ] || [ "$ROLE" = "benchmark" ]; then
    echo "Launching mlc_llm serve for $MODEL on port $PORT..."
    # --mode server keeps an OpenAI-compatible HTTP API up.
    mlc_llm serve "$MODEL" \
        --host 0.0.0.0 \
        --port "$PORT" \
        --device cuda \
        --mode server \
        --model-alias "$MODEL_ALIAS" \
        &

    wait_for_mlc
else
    echo "Unknown ROLE: $ROLE — not starting MLC"
    exec tail -f /dev/null
fi

echo "MLC ready. Container running..."
wait
