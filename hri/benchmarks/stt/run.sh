#!/bin/bash
# STT benchmark runner.
#
# Usage:
#   ./run.sh                          # interactive menu
#   ./run.sh --model distil-large-v3  # run accuracy + latency for one model
#   ./run.sh --all                    # benchmark every model in models.json
#   ./run.sh --tasks accuracy         # run only accuracy
#   ./run.sh --tasks latency          # run only latency
#   ./run.sh --runs 5                 # override run count for latency

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/../../..")"
REGISTRY="$SCRIPT_DIR/models.json"
RESULTS_DIR="$SCRIPT_DIR/results"
TASKS="accuracy,latency"
RUNS=3
MODEL=""
ALL_MODELS=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --model)   MODEL="$2"; shift 2 ;;
        --all)     ALL_MODELS=true; shift ;;
        --tasks)   TASKS="$2"; shift 2 ;;
        --runs)    RUNS="$2"; shift 2 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "Unknown flag: $1"; exit 1 ;;
    esac
done

mkdir -p "$RESULTS_DIR"

command -v jq >/dev/null 2>&1 || { echo "ERROR: jq required."; exit 1; }
[[ -f "$REGISTRY" ]] || { echo "ERROR: $REGISTRY not found."; exit 1; }

mapfile -t MODEL_NAMES < <(jq -r '.models[].name' "$REGISTRY")

echo "FRIDA STT benchmark — $(date)"
echo ""

# ── Select models ────────────────────────────────────────────────────────────

declare -a SELECTED=()
if $ALL_MODELS; then
    SELECTED=("${MODEL_NAMES[@]}")
elif [[ -n "$MODEL" ]]; then
    SELECTED=("$MODEL")
else
    echo "Available models:"
    for i in "${!MODEL_NAMES[@]}"; do
        printf "  %2d) %s\n" "$((i+1))" "${MODEL_NAMES[$i]}"
    done
    echo ""
    printf "Select (space-separated nums, empty=cancel): "
    read -r selection </dev/tty
    if [[ -z "$selection" ]]; then
        echo "No selection. Nothing to do."
        exit 0
    fi
    for n in $selection; do
        if ! [[ "$n" =~ ^[0-9]+$ ]] || (( n < 1 || n > ${#MODEL_NAMES[@]} )); then
            echo "Invalid selection: $n"; exit 1
        fi
        SELECTED+=("${MODEL_NAMES[$((n-1))]}")
    done
fi

# ── Run benchmarks ───────────────────────────────────────────────────────────

IFS=',' read -ra TASK_LIST <<< "$TASKS"

for model_name in "${SELECTED[@]}"; do
    echo ""
    echo "═══════════════════════════════════════════════════════════════"
    echo "  Model: $model_name"
    echo "  Tasks: ${TASK_LIST[*]}"
    echo "  Runs:  $RUNS"
    echo "═══════════════════════════════════════════════════════════════"

    ALL_RESULTS="{}"
    for task in "${TASK_LIST[@]}"; do
        task=$(echo "$task" | xargs)  # trim whitespace
        echo ""
        echo "── Running: $task ──"

        RESULT=$(cd "$SCRIPT_DIR" && python3 -c "
import json, sys
sys.path.insert(0, '.')
from tasks import TASK_REGISTRY
task_cls = TASK_REGISTRY.get('$task')
if task_cls is None:
    print(json.dumps({'error': 'unknown task $task'}))
    sys.exit(0)
r = task_cls.run(model_name='$model_name', runs=$RUNS)
print(json.dumps(r))
" 2>&1)

        ALL_RESULTS=$(echo "$ALL_RESULTS" | jq --argjson r "$RESULT" --arg t "$task" '. + {($t): $r}')
    done

    # Print table
    cd "$SCRIPT_DIR" && python3 -c "
import sys, json
sys.path.insert(0, '.')
import report as rpt
results = json.loads('''$ALL_RESULTS''')
rpt.print_model_table('$model_name', results)
"

    # Save JSON
    TS=$(date +%Y%m%d_%H%M%S)
    REPORT_PATH="$RESULTS_DIR/${model_name}_${TS}.json"
    echo "$ALL_RESULTS" | jq --arg ts "$(date -Iseconds)" '{"timestamp": $ts, "models": {"'$model_name"'}} + .' > "$REPORT_PATH"
    echo "  JSON report: $REPORT_PATH"
done

echo ""
echo "Done. Results in: $RESULTS_DIR"
