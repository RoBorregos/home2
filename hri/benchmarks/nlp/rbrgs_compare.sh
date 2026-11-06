#!/bin/bash
# A/B the GPSR command interpreter (rbrgs on :11435) across GGUFs using the BAML tests.
# Needs the HRI stack up with gpsr (./run.sh hri --gpsr) and the GGUFs in hri/packages/nlp/assets.
#
# Usage:
#   ./rbrgs_compare.sh [--runs N] [--no-restore] model_a.gguf model_b.gguf ...

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/../../..")"
ASSETS_DIR="$REPO_ROOT/hri/packages/nlp/assets"
HRI_DIR="$REPO_ROOT/docker/hri"
COMPOSE="compose/docker-compose-${ENV_TYPE:-l4t}.yml"
ROS_CONTAINER="${ROS_CONTAINER:-home2-hri-ros-${ENV_TYPE:-l4t}}"
RESULTS_DIR="$SCRIPT_DIR/results"
PORT=11435
LOAD_TIMEOUT=900

RUNS=1
RESTORE=true
MODELS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --runs)       RUNS="$2"; shift 2 ;;
        --no-restore) RESTORE=false; shift ;;
        -h|--help)    sed -n '2,6p' "$0"; exit 0 ;;
        *)            MODELS+=("$1"); shift ;;
    esac
done
[[ ${#MODELS[@]} -ge 1 ]] || { sed -n '2,6p' "$0"; exit 1; }

for m in "${MODELS[@]}"; do
    [[ -s "$ASSETS_DIR/$m" ]] || { echo "ERROR: $ASSETS_DIR/$m not found"; exit 1; }
done
docker ps -q -f name="$ROS_CONTAINER" | grep -q . || { echo "ERROR: $ROS_CONTAINER not running"; exit 1; }

mkdir -p "$RESULTS_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
TSV="$RESULTS_DIR/rbrgs_compare_${STAMP}.tsv"
printf "model\trun\ttest\tstatus\tsecs\n" > "$TSV"

serve_model() {
    local model="$1"
    echo "Loading ${model:-default rbrgs.F16.gguf} on :$PORT..."
    (cd "$HRI_DIR" && RBRGS_MODEL_FILE="$model" ROLE=gpsr COMPOSE_PROFILES=gpsr \
        docker compose -f "$COMPOSE" up -d --no-deps --force-recreate llama >/dev/null)
    local waited=0
    # Give the old server time to die so /health can't answer from it.
    sleep 5
    until curl -sf "localhost:$PORT/health" >/dev/null 2>&1; do
        sleep 5; waited=$((waited + 5))
        if (( waited >= LOAD_TIMEOUT )); then
            echo "ERROR: rbrgs did not come up in ${LOAD_TIMEOUT}s"; return 1
        fi
    done
    echo "  ready after ~${waited}s: $(curl -s "localhost:$PORT/props" | grep -o '"model_path":"[^"]*"')"
}

restore() {
    $RESTORE || return 0
    echo "Restoring default rbrgs model..."
    serve_model "" || true
}
trap restore EXIT

for m in "${MODELS[@]}"; do
    serve_model "$m"
    for ((r = 1; r <= RUNS; r++)); do
        log="$RESULTS_DIR/rbrgs_${m%.gguf}_${STAMP}_r${r}.log"
        echo "  run $r/$RUNS -> $(basename "$log")"
        docker exec "$ROS_CONTAINER" bash -c \
            "cd /workspace/src/task_manager/task_manager/utils && baml-cli test --from baml_src/ --parallel 1" \
            > "$log" 2>&1 || true
        # Only the summary block after "Test results:" has one line per test.
        sed 's/\x1b\[[0-9;]*m//g' "$log" | awk -v m="$m" -v r="$r" '
            /Test results:/ { s = 1; next }
            s && ($2 == "PASSED" || $2 == "FAILED") && $3 ~ /^GenerateCommandList::/ {
                secs = $1; sub(/s$/, "", secs); name = $3; sub(/^GenerateCommandList::/, "", name)
                printf "%s\t%s\t%s\t%s\t%s\n", m, r, name, $2, secs
            }' >> "$TSV"
    done
done

echo
echo "Summary (latency is per command, sequential):"
awk -F'\t' 'NR > 1 {
        n[$1]++; p[$1] += ($4 == "PASSED"); t[$1] += $5
        lat[$1] = lat[$1] " " $5
    }
    END {
        printf "%-40s %10s %10s %10s\n", "model", "pass", "mean_s", "median_s"
        for (m in n) {
            k = split(substr(lat[m], 2), v, " ")
            for (i = 1; i <= k; i++) for (j = i + 1; j <= k; j++) if (v[j] + 0 < v[i] + 0) { x = v[i]; v[i] = v[j]; v[j] = x }
            med = (k % 2) ? v[(k + 1) / 2] : (v[k / 2] + v[k / 2 + 1]) / 2
            printf "%-40s %6d/%-3d %10.2f %10.2f\n", m, p[m], n[m], t[m] / n[m], med
        }
    }' "$TSV"

echo
echo "Per test (passes/runs):"
awk -F'\t' -v models="${MODELS[*]}" 'NR > 1 { c[$3 SUBSEP $1] += ($4 == "PASSED"); tot[$3 SUBSEP $1]++; tests[$3] = 1 }
    END {
        k = split(models, M, " ")
        printf "%-36s", "test"; for (i = 1; i <= k; i++) printf " %20.20s", M[i]; print ""
        for (t in tests) {
            printf "%-36s", t
            for (i = 1; i <= k; i++) printf " %20s", (tot[t SUBSEP M[i]] ? c[t SUBSEP M[i]] "/" tot[t SUBSEP M[i]] : "-")
            print ""
        }
    }' "$TSV" | { read -r h; echo "$h"; sort; }

echo
echo "Raw results: $TSV"
