#!/bin/bash
# NLP benchmark model manager.
#
# Two-step flow:
#   1. Pick model(s) here → downloads GGUFs, optionally launches the production
#      llamacpp service (docker/hri/compose/llamacpp-l4t.yaml) with a single
#      cached model.
#   2. Run the benchmark via the integration container:
#        TEST_NLP=true NLP_MODEL_ALIAS=<alias> NLP_OLLAMA_URL=http://localhost:11434/v1 \
#          NLP_TASKS=is_positive,is_negative,extract_data \
#          ./run.sh integration --test-hri --build
#
# Selection rules:
#   "1 2 3"             → download only (no container).
#   single, not cached  → download only.
#   single, cached      → start production llamacpp with that GGUF, exit.
#   -1                  → delete-cached menu.
#   (empty)             → no-op message.
#
# Flags:
#   --models "N..."   Skip menu, treat as the selection.
#   --all             Download every model in models.json (no container).
#   --delete          Jump straight to delete menu.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/../../..")"
ASSETS_DIR="$REPO_ROOT/hri/packages/nlp/assets"
REGISTRY="$SCRIPT_DIR/models.json"
HRI_COMPOSE_DIR="$REPO_ROOT/docker/hri/compose"
COMPOSE_FILE="$HRI_COMPOSE_DIR/docker-compose-l4t.yml"
COMPOSE_ENV="$HRI_COMPOSE_DIR/.env"
PORT=11434
LIVE_CONTAINER="home2-hri-llamacpp-l4t"

MODEL_SELECT=""
SELECT_ALL=false
DELETE_MODE=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --models)  MODEL_SELECT="$2"; shift 2 ;;
        --all)     SELECT_ALL=true; shift ;;
        --delete)  DELETE_MODE=true; shift ;;
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "Unknown flag: $1"; exit 1 ;;
    esac
done

mkdir -p "$ASSETS_DIR" "$SCRIPT_DIR/results/logs"

# ── Helpers ──────────────────────────────────────────────────────────────────
upsert_env() {
    local file="$1" key="$2" value="$3"
    touch "$file"
    if grep -q "^${key}=" "$file" 2>/dev/null; then
        # Cross-platform in-place: use a tmp file.
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
            echo "Invalid: $n — skipped"; continue
        fi
        f="${gguf_files[$((n-1))]}"
        rm -f "$f" && echo "  Deleted: $(basename "$f")"
    done
}

# ── Delete-only mode ─────────────────────────────────────────────────────────
if $DELETE_MODE; then
    run_delete_menu
    exit 0
fi

# ── Logging ──────────────────────────────────────────────────────────────────
TS="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="$SCRIPT_DIR/results/logs/manager-$TS.log"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "── FRIDA NLP benchmark manager ── $(date) ──"

command -v jq >/dev/null 2>&1 || { echo "ERROR: jq required."; exit 1; }
[[ -f "$REGISTRY" ]] || { echo "ERROR: $REGISTRY not found."; exit 1; }

# ── Parse registry ───────────────────────────────────────────────────────────
mapfile -t MODEL_NAMES < <(jq -r '.models[].name' "$REGISTRY")
mapfile -t MODEL_URLS  < <(jq -r '.models[].hf_url' "$REGISTRY")
mapfile -t MODEL_FILES < <(jq -r '.models[].filename' "$REGISTRY")

[[ ${#MODEL_NAMES[@]} -eq 0 ]] && { echo "ERROR: registry empty."; exit 1; }

# ── Menu ─────────────────────────────────────────────────────────────────────
echo "Available models:"
for i in "${!MODEL_NAMES[@]}"; do
    f="$ASSETS_DIR/${MODEL_FILES[$i]}"
    status=$([[ -f "$f" ]] && echo "cached" || echo "needs download")
    printf "  %2d) %-22s [%s]\n" "$((i+1))" "${MODEL_NAMES[$i]}" "$status"
done
printf "  %2s) %-22s\n" "-1" "delete cached models"

declare -a SELECTED=()
if $SELECT_ALL; then
    for i in "${!MODEL_NAMES[@]}"; do SELECTED+=("$i"); done
elif [[ -n "$MODEL_SELECT" ]]; then
    for n in $MODEL_SELECT; do SELECTED+=("$((n-1))"); done
else
    printf "Select (space-separated nums, -1=delete, empty=cancel): "
    read -r selection </dev/tty
    if [[ -z "$selection" ]]; then
        echo "No selection. Nothing to do."
        exit 0
    elif [[ "$selection" =~ ^[[:space:]]*-1[[:space:]]*$ ]]; then
        run_delete_menu
        exit 0
    else
        for n in $selection; do
            if ! [[ "$n" =~ ^[0-9]+$ ]] || (( n < 1 || n > ${#MODEL_NAMES[@]} )); then
                echo "Invalid selection: $n"; exit 1
            fi
            SELECTED+=("$((n-1))")
        done
    fi
fi

# ── Dispatch: download-only vs. launch ───────────────────────────────────────
single_cached=false
if [[ ${#SELECTED[@]} -eq 1 ]] && [[ -f "$ASSETS_DIR/${MODEL_FILES[${SELECTED[0]}]}" ]]; then
    single_cached=true
fi

if ! $single_cached; then
    echo "Download mode (${#SELECTED[@]} model(s); no container will be started)."
    for idx in "${SELECTED[@]}"; do
        download_model "${MODEL_URLS[$idx]}" "$ASSETS_DIR/${MODEL_FILES[$idx]}" \
            || echo "  Failed: ${MODEL_NAMES[$idx]}"
    done
    echo "Done. Re-run with a single cached model to start llama-server."
    exit 0
fi

# Single + cached → launch the production llamacpp service with overrides
idx=${SELECTED[0]}
name="${MODEL_NAMES[$idx]}"
file="${MODEL_FILES[$idx]}"
alias_name="${file%.gguf}"

echo "Launch mode: $name ($file)"

# Stop existing llamacpp regardless of profile (same port, same container name)
if docker ps --format '{{.Names}}' | grep -qx "$LIVE_CONTAINER"; then
    echo "  Stopping existing $LIVE_CONTAINER..."
    docker stop "$LIVE_CONTAINER" >/dev/null
    docker rm "$LIVE_CONTAINER" 2>/dev/null || true
fi

upsert_env "$COMPOSE_ENV" "ROLE" "bench"
upsert_env "$COMPOSE_ENV" "LLAMA_MODEL_FILE" "$file"
upsert_env "$COMPOSE_ENV" "LLAMA_ALIAS" "$alias_name"
upsert_env "$COMPOSE_ENV" "COMPOSE_PROFILES" "bench"

echo "  docker compose up -d llama (profile=bench)..."
(cd "$HRI_COMPOSE_DIR" && docker compose -f docker-compose-l4t.yml --profile bench up -d llama)

echo -n "  Waiting for llama-server health on port $PORT"
for _ in $(seq 1 60); do
    if curl -sf "http://localhost:$PORT/health" >/dev/null 2>&1; then
        echo " — OK"
        break
    fi
    echo -n "."
    sleep 3
done
if ! curl -sf "http://localhost:$PORT/health" >/dev/null 2>&1; then
    echo
    echo "ERROR: llama-server failed to come up. Logs:"
    docker logs "$LIVE_CONTAINER" 2>&1 | tail -30
    exit 1
fi

cat <<EOF

✓ llama-server is up.
   Container: $LIVE_CONTAINER
   Model:     $file   (alias: $alias_name)
   Port:      $PORT

Next step — run the benchmark:

  TEST_NLP=true \\
  NLP_MODEL_ALIAS=$alias_name \\
  NLP_OLLAMA_URL=http://localhost:$PORT/v1 \\
  NLP_TASKS=is_positive,is_negative,extract_data \\
  ./run.sh integration --test-hri --build

To stop this server:
  docker compose -f $COMPOSE_FILE stop llama
EOF
