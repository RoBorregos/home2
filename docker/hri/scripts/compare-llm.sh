#!/bin/bash
# Compare llama.cpp vs Ollama across all 4 NLP use cases.
#
# Starts llama.cpp on 11434 and Ollama on 11436, benchmarks both, then stops.
#
# Usage:
#   ./compare-llm.sh [--runs N] [--thinking]
#
# Requirements: llama.cpp image already built/pulled, Ollama image available.
# Run from docker/hri/scripts/ or docker/hri/

set -euo pipefail

RUNS=3
NO_THINK=" /no_think"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ASSETS_DIR="$(realpath "$SCRIPT_DIR/../../../hri/packages/nlp/assets")"
TEGRA_DIR="/usr/lib/aarch64-linux-gnu/tegra"

LLAMA_CONTAINER="benchmark-llama"
OLLAMA_CONTAINER="benchmark-ollama"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --runs) RUNS="$2"; shift 2 ;;
        --thinking) NO_THINK=""; shift 1 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

cleanup() {
    echo
    echo "Stopping containers..."
    docker stop "$LLAMA_CONTAINER" 2>/dev/null || true
    docker rm  "$LLAMA_CONTAINER" 2>/dev/null || true
    docker stop "$OLLAMA_CONTAINER" 2>/dev/null || true
    docker rm  "$OLLAMA_CONTAINER" 2>/dev/null || true
}
trap cleanup EXIT

wait_for_port() {
    local url="$1" label="$2"
    local max=30 n=0
    echo -n "Waiting for $label..."
    while ! curl -sf "$url" >/dev/null 2>&1; do
        n=$((n+1))
        [ $n -ge $max ] && echo " TIMEOUT" && exit 1
        echo -n "."
        sleep 2
    done
    echo " ready."
}

# ── Start llama.cpp on 11434 ──────────────────────────────────────────────────
echo "Starting llama.cpp (port 11434)..."
docker run -d --name "$LLAMA_CONTAINER" \
    --runtime=nvidia \
    --network=host \
    -e ROLE=hric \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -v "$ASSETS_DIR:/root/.cache/huggingface" \
    -v "$TEGRA_DIR:$TEGRA_DIR" \
    -v "$SCRIPT_DIR/llama-cpp-entrypoint.sh:/scripts/entrypoint.sh" \
    ghcr.io/nvidia-ai-iot/llama_cpp:latest-jetson-orin \
    /bin/bash /scripts/entrypoint.sh

wait_for_port "http://localhost:11434/health" "llama.cpp"

# ── Start Ollama on 11436 ─────────────────────────────────────────────────────
echo "Starting Ollama (port 11436)..."
docker run -d --name "$OLLAMA_CONTAINER" \
    --runtime=nvidia \
    --network=host \
    -e ROLE=hric \
    -e OLLAMA_MODELS=/ollama \
    -e OLLAMA_HOST=0.0.0.0:11436 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -v "$ASSETS_DIR:/ollama" \
    -v "$TEGRA_DIR:$TEGRA_DIR" \
    -v "$SCRIPT_DIR/ollama-entrypoint.sh:/scripts/entrypoint.sh" \
    roborregos/home2:hri-ollama-l4t \
    /bin/bash -c "OLLAMA_HOST=0.0.0.0:11436 /scripts/entrypoint.sh && tail -f /dev/null"

wait_for_port "http://localhost:11436/api/version" "Ollama"

# ── Run benchmark across all use cases ───────────────────────────────────────
echo
echo "Running benchmark — $RUNS runs per use case..."
NO_THINK="$NO_THINK" bash "$SCRIPT_DIR/benchmark-llm.sh" \
    --runs "$RUNS" \
    --all \
    "http://localhost:11434/v1" "llama.cpp" \
    "http://localhost:11436/v1" "ollama"

# cleanup via trap
