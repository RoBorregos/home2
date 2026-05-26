#!/bin/bash
# Interactive NLP benchmark using llama.cpp.
# Selects a local GGUF model, spins up a temporary llama-server container,
# runs accuracy + performance tests, then cleans up.
#
# Called by: docker/hri/run.sh --benchmark
# Direct:    bash docker/hri/scripts/run-benchmark.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ASSETS_DIR="$(realpath "$SCRIPT_DIR/../../../hri/packages/nlp/assets")"
BENCHMARK_DIR="$(realpath "$SCRIPT_DIR/../../../hri/packages/nlp/benchmark")"
TEST_DATA_DIR="$(realpath "$SCRIPT_DIR/../../../hri/packages/nlp/test")"

LLAMA_IMAGE="ghcr.io/nvidia-ai-iot/llama_cpp:latest-jetson-orin"
BENCHMARK_IMAGE="benchmark-nlp"
LLAMA_CONTAINER="benchmark-llama-tmp"
PORT=11434

# ── Cleanup ───────────────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "Stopping llama.cpp container..."
    docker stop "$LLAMA_CONTAINER" 2>/dev/null || true
    docker rm  "$LLAMA_CONTAINER" 2>/dev/null || true
}
trap cleanup EXIT

# ── Header ────────────────────────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  FRIDA — NLP Benchmark  (llama.cpp)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# ── Pick a GGUF model ─────────────────────────────────────────────────────────
mapfile -t GGUF_FILES < <(find "$ASSETS_DIR" -maxdepth 1 -name "*.gguf" -type f 2>/dev/null | sort)

if [[ ${#GGUF_FILES[@]} -eq 0 ]]; then
    echo ""
    echo "ERROR: No .gguf files found in:"
    echo "  $ASSETS_DIR"
    echo ""
    echo "Download models first:  ./run.sh hri --download-model"
    exit 1
fi

echo ""
echo "Available models:"
for i in "${!GGUF_FILES[@]}"; do
    size=$(du -sh "${GGUF_FILES[$i]}" 2>/dev/null | cut -f1 || echo "?")
    printf "  %2d)  %-44s %s\n" "$((i+1))" "$(basename "${GGUF_FILES[$i]}")" "$size"
done
echo ""
printf "Select model [1]: "
read -r choice
choice="${choice:-1}"

if ! [[ "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#GGUF_FILES[@]} )); then
    echo "Invalid selection."
    exit 1
fi

MODEL_PATH="${GGUF_FILES[$((choice-1))]}"
MODEL_FILE="$(basename "$MODEL_PATH")"
# Use filename without extension as alias so benchmark.py can reference it
MODEL_ALIAS="${MODEL_FILE%.gguf}"

echo ""
echo "  Model : $MODEL_FILE"
echo "  Alias : $MODEL_ALIAS"
echo "  Port  : $PORT"

# ── Tasks menu ────────────────────────────────────────────────────────────────
echo ""
echo "Tasks to benchmark:"
echo "  1) All enabled (extract_data, is_positive, is_negative)"
echo "  2) extract_data only"
echo "  3) is_positive + is_negative only"
echo "  4) command_interpreter (slower, Jaccard scoring)"
echo ""
printf "Select tasks [1]: "
read -r task_choice
task_choice="${task_choice:-1}"

case "$task_choice" in
    1) TASK_ARGS=() ;;
    2) TASK_ARGS=(--tasks extract_data) ;;
    3) TASK_ARGS=(--tasks is_positive is_negative) ;;
    4) TASK_ARGS=(--tasks command_interpreter) ;;
    *) TASK_ARGS=() ;;
esac

# ── Performance runs ──────────────────────────────────────────────────────────
echo ""
printf "Number of perf timing runs per task [3]: "
read -r runs_input
runs_input="${runs_input:-3}"
RUNS_ARG=(--runs "$runs_input")

# ── Stop any old container on that port ──────────────────────────────────────
docker stop "$LLAMA_CONTAINER" 2>/dev/null || true
docker rm   "$LLAMA_CONTAINER" 2>/dev/null || true

# ── Start llama.cpp ───────────────────────────────────────────────────────────
echo ""
echo "Starting llama.cpp..."

DOCKER_ARGS=(
    -d
    --name "$LLAMA_CONTAINER"
    --network host
    -v "$ASSETS_DIR:/models"
)

# Nvidia runtime if available
if docker info 2>/dev/null | grep -qi "Runtimes.*nvidia\|nvidia.*Runtimes"; then
    DOCKER_ARGS+=(
        --runtime=nvidia
        -e NVIDIA_VISIBLE_DEVICES=all
        -e NVIDIA_DRIVER_CAPABILITIES=compute,utility
    )
fi

# Tegra libs for Jetson
TEGRA_DIR="/usr/lib/aarch64-linux-gnu/tegra"
if [[ -d "$TEGRA_DIR" ]]; then
    DOCKER_ARGS+=(
        -v "$TEGRA_DIR:$TEGRA_DIR"
        -e "LD_LIBRARY_PATH=$TEGRA_DIR"
    )
fi

docker run "${DOCKER_ARGS[@]}" "$LLAMA_IMAGE" \
    llama-server \
    --model  "/models/$MODEL_FILE" \
    --host   0.0.0.0 \
    --port   "$PORT" \
    --ctx-size 2048 \
    -ngl 99 \
    --flash-attn on \
    --parallel 1 \
    --alias  "$MODEL_ALIAS"

# ── Wait for server ───────────────────────────────────────────────────────────
echo -n "Waiting for llama-server on :$PORT"
for i in $(seq 1 40); do
    if curl -sf "http://localhost:$PORT/health" >/dev/null 2>&1; then
        echo "  ready."
        break
    fi
    printf "."
    sleep 3
    if (( i == 40 )); then
        echo " TIMEOUT"
        echo "ERROR: llama-server did not start. Check docker logs $LLAMA_CONTAINER"
        exit 1
    fi
done

# ── Build benchmark image ─────────────────────────────────────────────────────
echo ""
echo "Building benchmark image..."
docker build -q -t "$BENCHMARK_IMAGE" "$BENCHMARK_DIR"
mkdir -p "$BENCHMARK_DIR/results"

# ── Run benchmark ─────────────────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Running benchmark"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

docker run --rm \
    --network host \
    -v "$BENCHMARK_DIR/results:/benchmark/results" \
    -v "$TEST_DATA_DIR:/test_data:ro" \
    -it \
    "$BENCHMARK_IMAGE" \
    --ollama-url "http://localhost:$PORT/v1" \
    --models    "$MODEL_ALIAS" \
    "${TASK_ARGS[@]+"${TASK_ARGS[@]}"}" \
    "${RUNS_ARG[@]}"

echo ""
echo "Results saved in: hri/packages/nlp/benchmark/results/"
# cleanup via trap
