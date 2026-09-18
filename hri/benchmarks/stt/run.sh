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
#   ./run.sh --hotwords frida         # hint the model toward specific words
#   ./run.sh --no-vad                 # disable voice activity detection
#   ./run.sh --language en            # override language (default: en)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REGISTRY="$SCRIPT_DIR/models.json"
RESULTS_DIR="$SCRIPT_DIR/results"

TASKS="accuracy,latency"
RUNS=3
MODEL=""
ALL_MODELS=false
LANGUAGE="en"
VAD=true
HOTWORDS=""
INITIAL_PROMPT=""

usage() {
    sed -n '2,17p' "$0"
}

die() {
    echo "ERROR: $*" >&2
    exit 1
}

find_python() {
    local candidate exe dir ver
    local python311="" python_any=""

    for candidate in python3 python python.exe; do
        if command -v "$candidate" >/dev/null 2>&1; then
            if "$candidate" -c "import faster_whisper" 2>/dev/null; then
                echo "$candidate"
                return 0
            fi
            echo "WARNING: $candidate ($("$candidate" --version 2>&1)) is missing faster_whisper, scanning PATH..." >&2
            break
        fi
    done

    local dirs=()
    IFS=: read -ra dirs <<< "$PATH"
    for dir in "${dirs[@]}"; do
        for candidate in python3 python python.exe; do
            exe="$dir/$candidate"
            [[ -x "$exe" ]] || continue
            "$exe" -c "import faster_whisper" 2>/dev/null || continue
            ver=$("$exe" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2>/dev/null)
            if [[ "$ver" == "3.11" && -z "$python311" ]]; then
                python311="$exe"
            elif [[ -z "$python_any" ]]; then
                python_any="$exe"
            fi
        done
    done

    if [[ -n "${python311:-}" || -n "${python_any:-}" ]]; then
        echo "${python311:-$python_any}"
        return 0
    fi
    return 1
}

# Prompt the user to pick models interactively; populates SELECTED.
prompt_for_models() {
    local -n out=$1
    shift
    local names=("$@")
    local selection n

    echo "Available models:"
    for i in "${!names[@]}"; do
        printf "  %2d) %s\n" "$((i + 1))" "${names[$i]}"
    done
    echo ""
    printf "Select (space-separated nums, empty=cancel): "
    read -r selection </dev/tty

    if [[ -z "$selection" ]]; then
        echo "No selection. Nothing to do."
        exit 0
    fi

    for n in $selection; do
        if ! [[ "$n" =~ ^[0-9]+$ ]] || (( n < 1 || n > ${#names[@]} )); then
            die "Invalid selection: $n"
        fi
        out+=("${names[$((n - 1))]}")
    done
}

# Run a single task for a model and print its JSON result on stdout.
# Any failure/error detail goes to stderr exactly once.
run_task() {
    local task="$1" model="$2" kwargs="$3" runs="$4"
    local result

    if ! result=$(STT_KWARGS="$kwargs" STT_TASK="$task" STT_MODEL="$model" STT_RUNS="$runs" \
                   "$PYTHON" "$SCRIPT_DIR/_run_task.py"); then
        echo "  ERROR: task '$task' failed" >&2
        result='{"error": "task failed"}'
    elif echo "$result" | jq -e '.error' >/dev/null 2>&1; then
        echo "  WARNING: task '$task' reported an error: $result" >&2
    fi

    echo "$result"
}

# Run every requested task for one model, print the results table, and
# save the JSON report.
benchmark_model() {
    local model_name="$1" kwargs="$2" runs="$3"
    shift 3
    local task_list=("$@")
    local all_results="{}"
    local task result

    echo ""
    echo "═══════════════════════════════════════════════════════════════"
    echo "  Model: $model_name"
    echo "  Tasks: ${task_list[*]}"
    echo "  Runs:  $runs"
    echo "═══════════════════════════════════════════════════════════════"

    for task in "${task_list[@]}"; do
        task="${task// /}"  # trim whitespace
        echo ""
        echo "── Running: $task ──"

        result=$(run_task "$task" "$model_name" "$kwargs" "$runs")
        all_results=$(echo "$all_results" | jq --argjson r "$result" --arg t "$task" '. + {($t): $r}')
    done

    STT_ALL_RESULTS="$all_results" STT_MODEL="$model_name" "$PYTHON" "$SCRIPT_DIR/_print_table.py"

    local ts report_path
    ts=$(date +%Y%m%d_%H%M%S)
    report_path="$RESULTS_DIR/${model_name}_${ts}.json"
    echo "$all_results" | jq --arg ts "$(date -Iseconds)" --arg model "$model_name" \
        '{"timestamp": $ts, "models": {($model): .}}' > "$report_path"
    echo "  JSON report: $report_path"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --model)          MODEL="$2"; shift 2 ;;
        --all)            ALL_MODELS=true; shift ;;
        --tasks)          TASKS="$2"; shift 2 ;;
        --runs)           RUNS="$2"; shift 2 ;;
        --language)       LANGUAGE="$2"; shift 2 ;;
        --no-vad)         VAD=false; shift ;;
        --hotwords)       HOTWORDS="$2"; shift 2 ;;
        --initial-prompt) INITIAL_PROMPT="$2"; shift 2 ;;
        -h|--help)        usage; exit 0 ;;
        *) die "Unknown flag: $1" ;;
    esac
done

mkdir -p "$RESULTS_DIR"

command -v jq >/dev/null 2>&1 || die "jq required."
[[ -f "$REGISTRY" ]] || die "$REGISTRY not found."

PYTHON=$(find_python) || die "No python with faster_whisper found. Install it: pip install faster-whisper"
echo "Using python: $PYTHON ($("$PYTHON" --version 2>&1))"

mapfile -t MODEL_NAMES < <(jq -r '.models[].name' "$REGISTRY")

echo "FRIDA STT benchmark — $(date)"
echo ""

KWARGS=$(jq -n \
    --arg lang "$LANGUAGE" \
    --argjson vad "$VAD" \
    --arg hotwords "$HOTWORDS" \
    --arg prompt "$INITIAL_PROMPT" \
    '{language: $lang, vad: $vad, hotwords: $hotwords, initial_prompt: $prompt}')


declare -a SELECTED=()
if $ALL_MODELS; then
    SELECTED=("${MODEL_NAMES[@]}")
elif [[ -n "$MODEL" ]]; then
    SELECTED=("$MODEL")
else
    prompt_for_models SELECTED "${MODEL_NAMES[@]}"
fi

IFS=',' read -ra TASK_LIST <<< "$TASKS"

for model_name in "${SELECTED[@]}"; do
    benchmark_model "$model_name" "$KWARGS" "$RUNS" "${TASK_LIST[@]}"
done

echo ""
echo "Done. Results in: $RESULTS_DIR"