#!/bin/bash
# NLP multi-model benchmark orchestrator.
#
# Reads model registry from models.json, lets the user pick which to benchmark,
# downloads GGUFs to hri/packages/nlp/assets/, spins up a temporary
# llama-server per model, and drives the existing integration tests
# (task_manager/scripts/test/test_hri_manager.py in TEST_NLP mode) inside
# the integration container.
#
# Flags:
#   --models "1 3 5"   Model selection by index (skip interactive menu)
#   --all              Benchmark every model in the registry
#   --tasks LIST       Comma-separated: is_positive,is_negative,extract_data
#   --runs N           Perf timing runs per task (default 3)
#   --ephemeral        Delete each GGUF after its benchmark (saves disk)
#   --delete           Open the delete-cached-models menu instead of running
#   --env l4t|cuda|cpu Override ENV_TYPE for integration run (default l4t)
#
# All output is tee'd to hri/benchmarks/nlp/results/logs/<timestamp>.log.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/../../..")"
ASSETS_DIR="$REPO_ROOT/hri/packages/nlp/assets"
INTEGRATION_RUN="$REPO_ROOT/docker/integration/run.sh"
REGISTRY="$SCRIPT_DIR/models.json"

LLAMA_IMAGE="ghcr.io/nvidia-ai-iot/llama_cpp:latest-jetson-orin"
LLAMA_CONTAINER="benchmark-llama-tmp"
LIVE_CONTAINER="home2-hri-llamacpp-l4t"
PORT=11434

EPHEMERAL=false
RUNS=3
MODEL_SELECT=""
SELECT_ALL=false
DELETE_MODE=false
TASKS_CSV="is_positive,is_negative,extract_data"
ENV_TYPE_ARG="${ENV_TYPE:-l4t}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --models)     MODEL_SELECT="$2"; shift 2 ;;
        --all)        SELECT_ALL=true; shift ;;
        --tasks)      TASKS_CSV="$2"; shift 2 ;;
        --runs)       RUNS="$2"; shift 2 ;;
        --ephemeral)  EPHEMERAL=true; shift ;;
        --delete)     DELETE_MODE=true; shift ;;
        --env)        ENV_TYPE_ARG="$2"; shift 2 ;;
        -h|--help)    sed -n '2,21p' "$0"; exit 0 ;;
        *) echo "Unknown flag: $1"; exit 1 ;;
    esac
done

mkdir -p "$ASSETS_DIR" "$SCRIPT_DIR/results/logs"

run_delete_menu() {
    echo "Cached GGUFs in $ASSETS_DIR:"
    mapfile -t gguf_files < <(find "$ASSETS_DIR" -maxdepth 1 -name "*.gguf" | sort)
    if [[ ${#gguf_files[@]} -eq 0 ]]; then
        echo "  (none)"; return 0
    fi
    for i in "${!gguf_files[@]}"; do
        size=$(du -h "${gguf_files[$i]}" 2>/dev/null | cut -f1)
        printf "  %2d) %-50s [%s]\n" "$((i+1))" "$(basename "${gguf_files[$i]}")" "$size"
    done
    printf "Select to delete (space-separated nums, empty=cancel): "
    read -r selection </dev/tty
    [[ -z "$selection" ]] && { echo "Cancelled."; return 0; }
    for n in $selection; do
        if ! [[ "$n" =~ ^[0-9]+$ ]] || (( n < 1 || n > ${#gguf_files[@]} )); then
            echo "Invalid: $n — skipped"; continue
        fi
        f="${gguf_files[$((n-1))]}"
        rm -f "$f" && echo "  Deleted: $(basename "$f")"
    done
}

# ── Delete-only mode (--delete flag) ──────────────────────────────────────────
if $DELETE_MODE; then
    run_delete_menu
    exit 0
fi

# ── Logging ───────────────────────────────────────────────────────────────────
TS="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="$SCRIPT_DIR/results/logs/benchmark-$TS.log"
echo "Logging to: $LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "── FRIDA NLP benchmark ── $(date) ──"

# Conflict check: production llama-server can't be up
if docker ps --format '{{.Names}}' | grep -qx "$LIVE_CONTAINER"; then
    echo "ERROR: $LIVE_CONTAINER is running. Stop it first: docker stop $LIVE_CONTAINER"
    exit 1
fi

# Require jq for JSON registry parsing
if ! command -v jq >/dev/null 2>&1; then
    echo "ERROR: jq is required to parse $REGISTRY"; exit 1
fi
[[ -f "$REGISTRY" ]] || { echo "ERROR: registry not found at $REGISTRY"; exit 1; }

# ── Parse registry ────────────────────────────────────────────────────────────
mapfile -t MODEL_NAMES < <(jq -r '.models[].name' "$REGISTRY")
mapfile -t MODEL_URLS  < <(jq -r '.models[].hf_url' "$REGISTRY")
mapfile -t MODEL_FILES < <(jq -r '.models[].filename' "$REGISTRY")
mapfile -t MODEL_CTXS  < <(jq -r '.models[].ctx_size' "$REGISTRY")
mapfile -t MODEL_ARGS  < <(jq -r '.models[].extra_llama_args // ""' "$REGISTRY")

[[ ${#MODEL_NAMES[@]} -eq 0 ]] && { echo "ERROR: registry empty."; exit 1; }

# ── Model selection ──────────────────────────────────────────────────────────
echo "Available models:"
for i in "${!MODEL_NAMES[@]}"; do
    f="$ASSETS_DIR/${MODEL_FILES[$i]}"
    status=$([[ -f "$f" ]] && echo "cached" || echo "needs download")
    printf "  %2d) %-22s [%s]\n" "$((i+1))" "${MODEL_NAMES[$i]}" "$status"
done
printf "  %2s) %-22s\n" "-1" "delete cached models"

declare -a SELECTED
if $SELECT_ALL; then
    for i in "${!MODEL_NAMES[@]}"; do SELECTED+=("$i"); done
elif [[ -n "$MODEL_SELECT" ]]; then
    for n in $MODEL_SELECT; do SELECTED+=("$((n-1))"); done
else
    printf "Select models (space-separated nums, -1=delete menu, empty=all): "
    read -r selection </dev/tty
    if [[ -z "$selection" ]]; then
        for i in "${!MODEL_NAMES[@]}"; do SELECTED+=("$i"); done
    elif [[ "$selection" =~ ^[[:space:]]*-1[[:space:]]*$ ]]; then
        run_delete_menu
        exit 0
    else
        for n in $selection; do
            if ! [[ "$n" =~ ^-?[0-9]+$ ]] || (( n < 1 || n > ${#MODEL_NAMES[@]} )); then
                echo "Invalid selection: $n"; exit 1
            fi
            SELECTED+=("$((n-1))")
        done
    fi
fi

echo "Will benchmark ${#SELECTED[@]} model(s): ${SELECTED[*]/#/idx-}"
echo "Tasks: $TASKS_CSV   Runs: $RUNS   Ephemeral: $EPHEMERAL"

# Preflight: HRI services container must be up for integration tests
if ! docker ps --format '{{.Names}}' | grep -q 'hri-ros'; then
    echo "ERROR: hri-ros container is not running."
    echo "       Start it first:  ./run.sh hri $ENV_TYPE_ARG"
    exit 1
fi

# ── Helpers ──────────────────────────────────────────────────────────────────
cleanup_llama() {
    docker stop "$LLAMA_CONTAINER" 2>/dev/null || true
    docker rm   "$LLAMA_CONTAINER" 2>/dev/null || true
}
trap cleanup_llama EXIT

start_llama() {
    local model_file="$1" alias="$2" ctx="$3" extra="$4"
    cleanup_llama
    local docker_args=(-d --name "$LLAMA_CONTAINER" --network host -v "$ASSETS_DIR:/models")
    if docker info 2>/dev/null | grep -qi "nvidia"; then
        docker_args+=(--runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility)
    fi
    local tegra_dir="/usr/lib/aarch64-linux-gnu/tegra"
    [[ -d "$tegra_dir" ]] && docker_args+=(-v "$tegra_dir:$tegra_dir" -e "LD_LIBRARY_PATH=$tegra_dir")
    # shellcheck disable=SC2086
    docker run "${docker_args[@]}" "$LLAMA_IMAGE" \
        llama-server --model "/models/$model_file" --host 0.0.0.0 --port "$PORT" \
        --ctx-size "$ctx" -ngl 99 --parallel 1 --alias "$alias" $extra >/dev/null
}

wait_for_health() {
    for _ in $(seq 1 60); do
        curl -sf "http://localhost:$PORT/health" >/dev/null 2>&1 && return 0
        sleep 3
        docker ps -q -f name="$LLAMA_CONTAINER" | grep -q . || {
            echo "  llama-server died. Logs:"; docker logs "$LLAMA_CONTAINER" 2>&1 | tail -20; return 1
        }
    done
    echo "  TIMEOUT waiting for llama-server"; return 1
}

download_model() {
    local url="$1" dest="$2"
    [[ -f "$dest" ]] && return 0
    echo "  Downloading $(basename "$dest")..."
    curl -L --fail --progress-bar "$url" -o "$dest.partial" && mv "$dest.partial" "$dest"
}

# ── Main loop ────────────────────────────────────────────────────────────────
SUCCESSES=(); FAILURES=()
INTEGRATION_DIR="$(dirname "$INTEGRATION_RUN")"

for idx in "${SELECTED[@]}"; do
    name="${MODEL_NAMES[$idx]}"
    file="${MODEL_FILES[$idx]}"
    path="$ASSETS_DIR/$file"
    alias_name="${file%.gguf}"
    was_cached=false; [[ -f "$path" ]] && was_cached=true

    echo ""
    echo "══ Model: $name ── $(date +%H:%M:%S) ══"

    if ! download_model "${MODEL_URLS[$idx]}" "$path"; then
        echo "  Download failed."; FAILURES+=("$name (download)"); continue
    fi

    if ! start_llama "$file" "$alias_name" "${MODEL_CTXS[$idx]}" "${MODEL_ARGS[$idx]}"; then
        echo "  llama-server start failed."; FAILURES+=("$name (server-start)")
        $EPHEMERAL && ! $was_cached && rm -f "$path"; continue
    fi
    if ! wait_for_health; then
        FAILURES+=("$name (health)"); cleanup_llama
        $EPHEMERAL && ! $was_cached && rm -f "$path"; continue
    fi

    echo "  Running integration tests (TEST_NLP=true)..."
    if (
        cd "$INTEGRATION_DIR" && \
        TEST_NLP=true \
        NLP_MODEL_ALIAS="$alias_name" \
        NLP_OLLAMA_URL="http://localhost:$PORT/v1" \
        NLP_TASKS="$TASKS_CSV" \
        NLP_RUNS="$RUNS" \
        NLP_RESULTS_DIR="/workspace/src/hri/benchmarks/nlp/results" \
        bash "$INTEGRATION_RUN" --test-hri "$ENV_TYPE_ARG"
    ); then
        SUCCESSES+=("$name")
    else
        echo "  Benchmark errored."; FAILURES+=("$name (benchmark)")
    fi

    cleanup_llama
    if $EPHEMERAL && ! $was_cached; then
        echo "  Removing $file (--ephemeral)"; rm -f "$path"
    fi
done

echo ""
echo "── Done. $(date) ──"
echo "  Succeeded (${#SUCCESSES[@]}): ${SUCCESSES[*]:-none}"
echo "  Failed    (${#FAILURES[@]}): ${FAILURES[*]:-none}"
echo "  Reports: $SCRIPT_DIR/results/   Log: $LOG_FILE"
