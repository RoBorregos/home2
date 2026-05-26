#!/bin/bash
# Interactive multi-model NLP benchmark using llama.cpp.
#
# Reads model registry from benchmark-models.txt, lets the user multi-select
# which to benchmark, downloads GGUFs on demand from Hugging Face, spins up a
# temporary llama-server per model, runs the benchmark, and tears down.
#
# Called by: docker/hri/run.sh --benchmark
# Direct:    bash docker/hri/scripts/run-benchmark.sh [flags]
#
# Flags:
#   --ephemeral         Delete each GGUF after its benchmark finishes
#                       (saves disk; pays the download cost every run)
#   --runs N            Perf timing runs per task (default 3)
#   --tasks N           Task preset 1-4 (skips interactive task menu)
#   --models "1 3 5"    Model selection (skips interactive model menu)
#   --all               Benchmark every model in the registry
#
# All output is tee'd to hri/packages/nlp/benchmark/results/logs/<timestamp>.log
# so you can leave it running and check later.

set -euo pipefail

# ── Paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ASSETS_DIR="$(realpath "$SCRIPT_DIR/../../../hri/packages/nlp/assets")"
BENCHMARK_DIR="$(realpath "$SCRIPT_DIR/../../../hri/packages/nlp/benchmark")"
TEST_DATA_DIR="$(realpath "$SCRIPT_DIR/../../../hri/packages/nlp/test")"
REGISTRY="$SCRIPT_DIR/benchmark-models.txt"

LLAMA_IMAGE="ghcr.io/nvidia-ai-iot/llama_cpp:latest-jetson-orin"
BENCHMARK_IMAGE="benchmark-nlp"
LLAMA_CONTAINER="benchmark-llama-tmp"
LIVE_CONTAINER="home2-hri-llamacpp-l4t"
PORT=11434

EPHEMERAL=false
RUNS=3
TASK_PRESET=""
MODEL_SELECT=""
SELECT_ALL=false

# ── Args ─────────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ephemeral) EPHEMERAL=true; shift ;;
        --runs)      RUNS="$2"; shift 2 ;;
        --tasks)     TASK_PRESET="$2"; shift 2 ;;
        --models)    MODEL_SELECT="$2"; shift 2 ;;
        --all)       SELECT_ALL=true; shift ;;
        -h|--help)
            sed -n '2,22p' "$0"; exit 0 ;;
        *)           echo "Unknown flag: $1"; exit 1 ;;
    esac
done

# ── Logging ──────────────────────────────────────────────────────────────────
LOG_DIR="$BENCHMARK_DIR/results/logs"
mkdir -p "$LOG_DIR"
TS="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="$LOG_DIR/benchmark-$TS.log"
echo "Logging to: $LOG_FILE"
# Tee all subsequent stdout/stderr to the log while keeping it on terminal.
exec > >(tee -a "$LOG_FILE") 2>&1

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  FRIDA — NLP Multi-Model Benchmark  (llama.cpp)"
echo "  $(date)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# ── Safety: refuse to run if the production llamacpp container is up ─────────
if docker ps --format '{{.Names}}' | grep -qx "$LIVE_CONTAINER"; then
    echo ""
    echo "ERROR: The live llamacpp container ($LIVE_CONTAINER) is running."
    echo "       The benchmark would conflict on port $PORT."
    echo "       Stop it first:  docker stop $LIVE_CONTAINER"
    exit 1
fi

# ── Parse registry ───────────────────────────────────────────────────────────
if [[ ! -f "$REGISTRY" ]]; then
    echo "ERROR: registry not found at $REGISTRY"
    exit 1
fi

declare -a MODEL_NAMES MODEL_URLS MODEL_FILES MODEL_CTXS MODEL_ARGS
while IFS='|' read -r name url file ctx args || [[ -n "$name" ]]; do
    [[ -z "$name" || "$name" =~ ^[[:space:]]*# ]] && continue
    MODEL_NAMES+=("$name")
    MODEL_URLS+=("$url")
    MODEL_FILES+=("$file")
    MODEL_CTXS+=("$ctx")
    MODEL_ARGS+=("$args")
done < "$REGISTRY"

if [[ ${#MODEL_NAMES[@]} -eq 0 ]]; then
    echo "ERROR: registry $REGISTRY has no entries."
    exit 1
fi

# ── Model menu ───────────────────────────────────────────────────────────────
echo ""
echo "Available models:"
for i in "${!MODEL_NAMES[@]}"; do
    f="$ASSETS_DIR/${MODEL_FILES[$i]}"
    if [[ -f "$f" ]]; then
        status="cached ($(du -h "$f" 2>/dev/null | cut -f1))"
    else
        status="needs download"
    fi
    printf "  %2d) %-22s [%s]\n" "$((i+1))" "${MODEL_NAMES[$i]}" "$status"
done
echo ""

declare -a SELECTED
if $SELECT_ALL; then
    for i in "${!MODEL_NAMES[@]}"; do SELECTED+=("$i"); done
elif [[ -n "$MODEL_SELECT" ]]; then
    read -r -a sel <<< "$MODEL_SELECT"
    for n in "${sel[@]}"; do SELECTED+=("$((n-1))"); done
else
    printf "Select models (space-separated nums, or empty=all): "
    read -r selection </dev/tty
    if [[ -z "$selection" ]]; then
        for i in "${!MODEL_NAMES[@]}"; do SELECTED+=("$i"); done
    else
        for n in $selection; do
            if ! [[ "$n" =~ ^[0-9]+$ ]] || (( n < 1 || n > ${#MODEL_NAMES[@]} )); then
                echo "Invalid selection: $n"; exit 1
            fi
            SELECTED+=("$((n-1))")
        done
    fi
fi

echo ""
echo "Will benchmark: ${#SELECTED[@]} model(s)"
for idx in "${SELECTED[@]}"; do echo "  • ${MODEL_NAMES[$idx]}"; done

# ── Task menu ────────────────────────────────────────────────────────────────
echo ""
echo "Tasks to benchmark:"
echo "  1) All enabled (extract_data, is_positive, is_negative)"
echo "  2) extract_data only"
echo "  3) is_positive + is_negative only"
echo "  4) command_interpreter (slower, Jaccard scoring)"
if [[ -z "$TASK_PRESET" ]]; then
    printf "Select tasks [1]: "
    read -r TASK_PRESET </dev/tty
fi
TASK_PRESET="${TASK_PRESET:-1}"
case "$TASK_PRESET" in
    1) TASK_ARGS=() ;;
    2) TASK_ARGS=(--tasks extract_data) ;;
    3) TASK_ARGS=(--tasks is_positive is_negative) ;;
    4) TASK_ARGS=(--tasks command_interpreter) ;;
    *) echo "Invalid task preset: $TASK_PRESET"; exit 1 ;;
esac

echo ""
echo "Perf runs per task: $RUNS"
$EPHEMERAL && echo "Ephemeral mode: GGUFs will be deleted after each model."

# ── Build the benchmark image once ───────────────────────────────────────────
echo ""
echo "Building benchmark image..."
docker build -q -t "$BENCHMARK_IMAGE" "$BENCHMARK_DIR" >/dev/null
mkdir -p "$BENCHMARK_DIR/results"

# ── Helpers ──────────────────────────────────────────────────────────────────
cleanup_llama() {
    docker stop "$LLAMA_CONTAINER" 2>/dev/null || true
    docker rm  "$LLAMA_CONTAINER" 2>/dev/null || true
}
trap cleanup_llama EXIT

start_llama() {
    local model_file="$1" alias="$2" ctx="$3" extra="$4"
    cleanup_llama

    local docker_args=(
        -d
        --name "$LLAMA_CONTAINER"
        --network host
        -v "$ASSETS_DIR:/models"
    )
    if docker info 2>/dev/null | grep -qi "Runtimes.*nvidia\|nvidia.*Runtimes"; then
        docker_args+=(
            --runtime=nvidia
            -e NVIDIA_VISIBLE_DEVICES=all
            -e NVIDIA_DRIVER_CAPABILITIES=compute,utility
        )
    fi
    local tegra_dir="/usr/lib/aarch64-linux-gnu/tegra"
    if [[ -d "$tegra_dir" ]]; then
        docker_args+=(-v "$tegra_dir:$tegra_dir" -e "LD_LIBRARY_PATH=$tegra_dir")
    fi

    # shellcheck disable=SC2086  # extra is meant to word-split
    docker run "${docker_args[@]}" "$LLAMA_IMAGE" \
        llama-server \
        --model "/models/$model_file" \
        --host 0.0.0.0 \
        --port "$PORT" \
        --ctx-size "$ctx" \
        -ngl 99 \
        --parallel 1 \
        --alias "$alias" \
        $extra >/dev/null
}

wait_for_health() {
    local name="$1"
    echo -n "  Waiting for llama-server"
    for i in $(seq 1 60); do
        if curl -sf "http://localhost:$PORT/health" >/dev/null 2>&1; then
            echo "  ready."
            return 0
        fi
        printf "."
        sleep 3
        if ! docker ps -q -f name="$LLAMA_CONTAINER" | grep -q .; then
            echo ""
            echo "  ERROR: llama-server container died for $name. Logs:"
            docker logs "$LLAMA_CONTAINER" 2>&1 | tail -30
            return 1
        fi
    done
    echo " TIMEOUT"
    docker logs "$LLAMA_CONTAINER" 2>&1 | tail -30
    return 1
}

download_model() {
    local url="$1" dest="$2"
    if [[ -f "$dest" ]]; then return 0; fi
    echo "  Downloading $(basename "$dest") ..."
    if ! curl -L --fail --progress-bar "$url" -o "$dest.partial"; then
        rm -f "$dest.partial"
        return 1
    fi
    mv "$dest.partial" "$dest"
}

# ── Main loop ────────────────────────────────────────────────────────────────
SUCCESSES=()
FAILURES=()

for idx in "${SELECTED[@]}"; do
    name="${MODEL_NAMES[$idx]}"
    url="${MODEL_URLS[$idx]}"
    file="${MODEL_FILES[$idx]}"
    ctx="${MODEL_CTXS[$idx]}"
    extra="${MODEL_ARGS[$idx]}"
    path="$ASSETS_DIR/$file"
    alias_name="${file%.gguf}"
    was_cached=false
    [[ -f "$path" ]] && was_cached=true

    echo ""
    echo "════════════════════════════════════════════════════"
    echo "  Model: $name   ($(date +%H:%M:%S))"
    echo "════════════════════════════════════════════════════"

    if ! download_model "$url" "$path"; then
        echo "  Download failed for $name — skipping."
        FAILURES+=("$name (download)")
        continue
    fi

    echo "  File : $file"
    echo "  Ctx  : $ctx"
    echo "  Args : $extra"
    echo "  Starting llama-server..."

    if ! start_llama "$file" "$alias_name" "$ctx" "$extra"; then
        echo "  llama-server failed to start — skipping."
        FAILURES+=("$name (server-start)")
        $EPHEMERAL && ! $was_cached && rm -f "$path"
        continue
    fi

    if ! wait_for_health "$name"; then
        FAILURES+=("$name (health)")
        cleanup_llama
        $EPHEMERAL && ! $was_cached && rm -f "$path"
        continue
    fi

    echo ""
    echo "  ── Running benchmark ─────────────────────────────"
    if docker run --rm \
        --network host \
        -v "$BENCHMARK_DIR/results:/benchmark/results" \
        -v "$TEST_DATA_DIR:/test_data:ro" \
        "$BENCHMARK_IMAGE" \
        --ollama-url "http://localhost:$PORT/v1" \
        --models    "$alias_name" \
        "${TASK_ARGS[@]+"${TASK_ARGS[@]}"}" \
        --runs "$RUNS"
    then
        SUCCESSES+=("$name")
    else
        echo "  Benchmark errored for $name."
        FAILURES+=("$name (benchmark)")
    fi

    cleanup_llama

    if $EPHEMERAL && ! $was_cached; then
        echo "  Removing $file (--ephemeral, was not cached before run)"
        rm -f "$path"
    fi
done

# ── Summary ──────────────────────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Done.  $(date)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Succeeded: ${#SUCCESSES[@]}"
for m in "${SUCCESSES[@]}"; do echo "    ✓ $m"; done
echo "  Failed:    ${#FAILURES[@]}"
for m in "${FAILURES[@]}"; do echo "    ✗ $m"; done
echo ""
echo "  Per-model JSON reports : $BENCHMARK_DIR/results/benchmark_*.json"
echo "  Full session log       : $LOG_FILE"
