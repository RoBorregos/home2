#!/bin/bash
# NLP benchmark driver: llama.cpp vs Ollama on the same GGUF.
#
# Both backends bind port 11434 and are started one at a time, so the single
# Jetson GPU is never shared between them.
#
# Usage:
#   ./run.sh --backend both --model qwen3-4b --runs 5
#   ./run.sh --backend llamacpp                 # menu picks the model
#   ./run.sh --download-only --all              # just fetch GGUFs
#   ./run.sh --delete                           # delete-cached menu
#
# Flags:
#   --backend  llamacpp | ollama | both   (default: llamacpp)
#   --model    registry name or index     (default: interactive menu)
#   --runs     timed runs per task        (default: 5, after 1 discarded warmup)
#   --tasks    comma-separated task list  (default: all)
#   --download-only / --all / --delete / --no-build / --keep-up

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/../../..")"
ASSETS_DIR="$REPO_ROOT/hri/packages/nlp/assets"
REGISTRY="$SCRIPT_DIR/models.json"
HRI_COMPOSE_DIR="$REPO_ROOT/docker/hri/compose"
BENCH_COMPOSE="$HRI_COMPOSE_DIR/bench-l4t.yaml"
COMPOSE_ENV="$HRI_COMPOSE_DIR/.env"
RESULTS_DIR="$SCRIPT_DIR/results"
CONTAINER_RESULTS_DIR="/workspace/src/hri/benchmarks/nlp/results"
PORT=11434

BACKEND="llamacpp"
MODEL_SELECT=""
RUNS=5
TASKS="is_coherent,extract_data,is_positive,is_negative,llm_wrapper"
SELECT_ALL=false
DELETE_MODE=false
DOWNLOAD_ONLY=false
BUILD_FLAG="--build"
KEEP_UP=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --backend)       BACKEND="$2"; shift 2 ;;
        --model|--models) MODEL_SELECT="$2"; shift 2 ;;
        --runs)          RUNS="$2"; shift 2 ;;
        --tasks)         TASKS="$2"; shift 2 ;;
        --all)           SELECT_ALL=true; shift ;;
        --delete)        DELETE_MODE=true; shift ;;
        --download-only) DOWNLOAD_ONLY=true; shift ;;
        --no-build)      BUILD_FLAG=""; shift ;;
        --keep-up)       KEEP_UP=true; shift ;;
        -h|--help)       sed -n '2,19p' "$0"; exit 0 ;;
        *) echo "Unknown flag: $1"; exit 1 ;;
    esac
done

if ((BASH_VERSINFO[0] < 4)); then
    echo "ERROR: bash 4+ required (this is ${BASH_VERSION}). Run this on the Jetson."
    exit 1
fi

case "$BACKEND" in
    llamacpp) BACKENDS=(llamacpp) ;;
    ollama)   BACKENDS=(ollama) ;;
    both)     BACKENDS=(llamacpp ollama) ;;
    *) echo "ERROR: --backend must be llamacpp, ollama or both"; exit 1 ;;
esac

mkdir -p "$ASSETS_DIR" "$RESULTS_DIR"

upsert_env() {
    local file="$1" key="$2" value="$3"
    touch "$file"
    if grep -q "^${key}=" "$file" 2>/dev/null; then
        grep -v "^${key}=" "$file" > "$file.tmp"
        mv "$file.tmp" "$file"
    fi
    echo "${key}=${value}" >> "$file"
}

download_model() {
    local url="$1" dest="$2"
    [[ -f "$dest" ]] && { echo "  Already cached: $(basename "$dest")"; return 0; }
    echo "  Downloading $(basename "$dest")..."
    curl -L --fail --progress-bar "$url" -o "$dest.partial" && mv "$dest.partial" "$dest"
}

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
            echo "Invalid: $n - skipped"; continue
        fi
        rm -f "${gguf_files[$((n-1))]}" && echo "  Deleted: $(basename "${gguf_files[$((n-1))]}")"
    done
}

if $DELETE_MODE; then
    run_delete_menu
    exit 0
fi

echo "FRIDA NLP benchmark - $(date)"

command -v jq >/dev/null 2>&1 || { echo "ERROR: jq required."; exit 1; }
[[ -f "$REGISTRY" ]] || { echo "ERROR: $REGISTRY not found."; exit 1; }
[[ -f "$BENCH_COMPOSE" ]] || { echo "ERROR: $BENCH_COMPOSE not found."; exit 1; }

mapfile -t MODEL_NAMES < <(jq -r '.models[].name' "$REGISTRY")
mapfile -t MODEL_URLS  < <(jq -r '.models[].hf_url' "$REGISTRY")
mapfile -t MODEL_FILES < <(jq -r '.models[].filename' "$REGISTRY")
mapfile -t MODEL_CTX   < <(jq -r '.models[].ctx_size // 2048' "$REGISTRY")

[[ ${#MODEL_NAMES[@]} -eq 0 ]] && { echo "ERROR: registry empty."; exit 1; }

resolve_selection() {
    declare -ga SELECTED=()
    if $SELECT_ALL; then
        for i in "${!MODEL_NAMES[@]}"; do SELECTED+=("$i"); done
        return
    fi
    if [[ -n "$MODEL_SELECT" ]]; then
        for tok in ${MODEL_SELECT//,/ }; do
            if [[ "$tok" =~ ^[0-9]+$ ]]; then
                SELECTED+=("$((tok-1))")
            else
                local found=""
                for i in "${!MODEL_NAMES[@]}"; do
                    [[ "${MODEL_NAMES[$i]}" == "$tok" ]] && found="$i" && break
                done
                [[ -z "$found" ]] && { echo "ERROR: unknown model '$tok'"; exit 1; }
                SELECTED+=("$found")
            fi
        done
        return
    fi
    echo "Available models:"
    for i in "${!MODEL_NAMES[@]}"; do
        f="$ASSETS_DIR/${MODEL_FILES[$i]}"
        status=$([[ -f "$f" ]] && echo "cached" || echo "needs download")
        printf "  %2d) %-22s [%s]\n" "$((i+1))" "${MODEL_NAMES[$i]}" "$status"
    done
    printf "  %2s) %-22s\n" "-1" "delete cached models"
    printf "Select (space-separated nums, -1=delete, empty=cancel): "
    read -r selection </dev/tty
    if [[ -z "$selection" ]]; then
        echo "No selection. Nothing to do."; exit 0
    elif [[ "$selection" =~ ^[[:space:]]*-1[[:space:]]*$ ]]; then
        run_delete_menu; exit 0
    fi
    for n in $selection; do
        if ! [[ "$n" =~ ^[0-9]+$ ]] || (( n < 1 || n > ${#MODEL_NAMES[@]} )); then
            echo "Invalid selection: $n"; exit 1
        fi
        SELECTED+=("$((n-1))")
    done
}

resolve_selection

if $DOWNLOAD_ONLY || [[ ${#SELECTED[@]} -gt 1 ]]; then
    echo "Download mode (${#SELECTED[@]} model(s); no container will be started)."
    for idx in "${SELECTED[@]}"; do
        download_model "${MODEL_URLS[$idx]}" "$ASSETS_DIR/${MODEL_FILES[$idx]}" \
            || echo "  Failed: ${MODEL_NAMES[$idx]}"
    done
    echo "Done."
    exit 0
fi

IDX=${SELECTED[0]}
MODEL_NAME="${MODEL_NAMES[$IDX]}"
MODEL_FILE="${MODEL_FILES[$IDX]}"
MODEL_CTX_SIZE="${MODEL_CTX[$IDX]}"
ALIAS="${MODEL_FILE%.gguf}"

if [[ ! -f "$ASSETS_DIR/$MODEL_FILE" ]]; then
    echo "Model not cached; downloading first."
    download_model "${MODEL_URLS[$IDX]}" "$ASSETS_DIR/$MODEL_FILE"
fi

stop_backends() {
    (cd "$HRI_COMPOSE_DIR" && \
        docker compose -f bench-l4t.yaml --profile llamacpp --profile ollama down --remove-orphans >/dev/null 2>&1) || true
    for c in home2-hri-bench-llamacpp home2-hri-bench-ollama home2-hri-llamacpp-l4t home2-hri-ollama-l4t; do
        docker rm -f "$c" >/dev/null 2>&1 || true
    done
}

wait_healthy() {
    local backend="$1"
    local probe="http://localhost:$PORT/health"
    [[ "$backend" == "ollama" ]] && probe="http://localhost:$PORT/api/version"
    echo -n "  Waiting for $backend on $PORT"
    for _ in $(seq 1 90); do
        if curl -sf "$probe" >/dev/null 2>&1; then echo " - OK"; return 0; fi
        echo -n "."
        sleep 3
    done
    echo
    echo "ERROR: $backend failed to come up. Logs:"
    docker logs "home2-hri-bench-$backend" 2>&1 | tail -40
    return 1
}

trap 'if ! $KEEP_UP; then echo; echo "Stopping backends..."; stop_backends; fi' EXIT

upsert_env "$COMPOSE_ENV" "ROLE" "bench"
upsert_env "$COMPOSE_ENV" "LLAMA_MODEL_FILE" "$MODEL_FILE"
upsert_env "$COMPOSE_ENV" "LLAMA_ALIAS" "$ALIAS"
upsert_env "$COMPOSE_ENV" "LLAMA_CTX_SIZE" "$MODEL_CTX_SIZE"

echo
echo "Model:    $MODEL_NAME ($MODEL_FILE, ctx=$MODEL_CTX_SIZE, alias=$ALIAS)"
echo "Backends: ${BACKENDS[*]}  (sequential, one GPU)"
echo "Tasks:    $TASKS"
echo "Runs:     $RUNS per task (+1 discarded warmup)"

REPORTS=()

for backend in "${BACKENDS[@]}"; do
    echo
    echo "=============================================================="
    echo "BACKEND: $backend"
    echo "=============================================================="

    stop_backends
    echo "  Starting $backend..."
    (cd "$HRI_COMPOSE_DIR" && \
        docker compose -f bench-l4t.yaml --profile "$backend" up -d "bench-$backend")
    wait_healthy "$backend"

    before="$(ls -1 "$RESULTS_DIR"/benchmark_*.json 2>/dev/null | sort || true)"

    echo "  Running accuracy + perf via integration container..."
    set +e
    TEST_NLP=true \
    NLP_BACKEND="$backend" \
    NLP_MODEL_ALIAS="$ALIAS" \
    NLP_OLLAMA_URL="http://localhost:$PORT/v1" \
    NLP_TASKS="$TASKS" \
    NLP_RUNS="$RUNS" \
    NLP_RESULTS_DIR="$CONTAINER_RESULTS_DIR" \
        "$REPO_ROOT/run.sh" integration --test-hri $BUILD_FLAG
    rc=$?
    set -e
    [[ $rc -ne 0 ]] && echo "  WARNING: integration run exited $rc"

    after="$(ls -1 "$RESULTS_DIR"/benchmark_*.json 2>/dev/null | sort || true)"
    new_report="$(comm -13 <(echo "$before") <(echo "$after") | tail -1)"
    if [[ -n "$new_report" ]]; then
        echo "  Report: $new_report"
        REPORTS+=("$new_report")
    else
        echo "  WARNING: no new benchmark JSON produced for $backend"
    fi

    BUILD_FLAG=""   # only build the integration image once
done

echo
if [[ ${#REPORTS[@]} -gt 0 ]]; then
    echo "Reports:"
    printf '  %s\n' "${REPORTS[@]}"
    python3 "$SCRIPT_DIR/report.py" "${REPORTS[@]}" || true
else
    echo "No reports produced."
fi
