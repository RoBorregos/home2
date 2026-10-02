#!/bin/bash
# Embedding-gallery benchmark runner.
#
# Usage:
#   ./run.sh                                   # interactive task menu
#   ./run.sh --tasks boxes,embeddings          # run the listed tasks
#   ./run.sh --all --source /path/to/export    # every task (e2e_* need --source)
#   ./run.sh --tasks embeddings --backbones dinov2_vitb14,clip_vit_b32
#   ./run.sh --tasks e2e_calibrate --from-cache
#   ./run.sh prepare --source /path/to/export  # build data/ from a YOLO-seg export
#   ./run.sh experiment finetune-head          # optional, not used in production
#   ./run.sh experiment finetune-arcface --from-cache
#
# Tasks:
#   boxes          Phase 0: box-proposer recall
#   embeddings     Phase 1: backbone comparison
#   e2e_eval       recall/rejection with the production proposer's real crops
#   e2e_calibrate  per-class threshold calibration on real crops
#
# Task options (forwarded to tasks.py, which falls back to per-task defaults):
#   --backbones a,b   backbone names from models.json (embeddings; default all)
#   --backbone ID     timm backbone id (e2e_*)
#   --source PATH     YOLO-seg export (e2e_*)
#   --split S         dataset split (e2e_*, default test)
#   --n-images N      images to sample (e2e_*)
#   --seed N          sampling seed (e2e_*)
#   --from-cache      e2e_calibrate: reuse results/e2e_crops_cache.npz
#   --rounds N        e2e_calibrate: per-class descent rounds
#   --min-similarity X / --margin-min X   e2e_eval thresholds
#   --iou X / --data DIR                  boxes
#   --results-dir DIR                     where JSON results go
#
# Needs a Python with torch and timm (plus ultralytics/cv2 for box tasks, and
# clip for CLIP backbones), e.g. inside the vision container. Set
# EMBEDDING_PYTHON to pick an interpreter; .venv/ here is tried before python3.
#
# Python runs from ~/.cache/embedding_gallery (override with EMBEDDING_WORKDIR)
# so files ultralytics downloads into the cwd (mobileclip_blt.ts, 570 MB) stay
# out of the repo. Relative --source/--data/--results-dir are made absolute
# first; pass absolute paths to `prepare` and `experiment`.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPTS_DIR="$SCRIPT_DIR/../../packages/object_detector_2d/scripts"
ALL_TASKS=(boxes embeddings e2e_eval e2e_calibrate)
WORK_DIR="${EMBEDDING_WORKDIR:-${XDG_CACHE_HOME:-$HOME/.cache}/embedding_gallery}"
RESULTS_DIR="$SCRIPT_DIR/results"

die() {
    echo "ERROR: $*" >&2
    exit 1
}

usage() {
    sed -n '2,40p' "$0"
}

find_python() {
    local candidate
    for candidate in "${EMBEDDING_PYTHON:-}" "$SCRIPT_DIR/.venv/bin/python" python3 python; do
        [[ -n "$candidate" ]] || continue
        command -v "$candidate" >/dev/null 2>&1 || continue
        if "$candidate" -c "import torch, timm" 2>/dev/null; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

prompt_for_tasks() {
    local selection n
    echo "Available tasks:" >&2
    for i in "${!ALL_TASKS[@]}"; do
        printf "  %2d) %s\n" "$((i + 1))" "${ALL_TASKS[$i]}" >&2
    done
    echo "" >&2
    printf "Select (space-separated nums, empty=cancel): " >&2
    read -r selection </dev/tty
    [[ -n "$selection" ]] || { echo "No selection. Nothing to do." >&2; exit 0; }
    for n in $selection; do
        if ! [[ "$n" =~ ^[0-9]+$ ]] || (( n < 1 || n > ${#ALL_TASKS[@]} )); then
            die "Invalid selection: $n"
        fi
        echo "${ALL_TASKS[$((n - 1))]}"
    done
}

abs_path() {
    case "$1" in
        /*) echo "$1" ;;
        *)  echo "$PWD/$1" ;;
    esac
}

# Run a script of this directory with the right PYTHONPATH (this directory for
# our modules, and the object_detector_2d scripts/ dir for `detectors` and
# `embedding_gallery`).
run_py() {
    mkdir -p "$WORK_DIR"
    (
        cd "$WORK_DIR"
        PYTHONPATH="$SCRIPT_DIR:$SCRIPTS_DIR:${PYTHONPATH:-}" \
            exec "$PYTHON" "$@"
    )
}

MODE="tasks"
case "${1:-}" in
    prepare)    MODE="prepare"; shift ;;
    experiment) MODE="experiment"; shift ;;
esac

TASKS=""
ALL=false
SOURCE=""
FROM_CACHE=false
PASSTHROUGH=()

if [[ "$MODE" == "tasks" ]]; then
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --tasks)       TASKS="$2"; shift 2 ;;
            --all)         ALL=true; shift ;;
            --source)      [[ $# -ge 2 ]] || die "$1 needs a value"
                           SOURCE="$(abs_path "$2")"; PASSTHROUGH+=("$1" "$SOURCE"); shift 2 ;;
            --data|--results-dir)
                [[ $# -ge 2 ]] || die "$1 needs a value"
                [[ "$1" == "--results-dir" ]] && RESULTS_DIR="$(abs_path "$2")"
                PASSTHROUGH+=("$1" "$(abs_path "$2")"); shift 2 ;;
            --from-cache)  FROM_CACHE=true; PASSTHROUGH+=("$1"); shift ;;
            --backbones|--backbone|--split|--n-images|--seed|--rounds|--iou|--min-similarity|--margin-min)
                [[ $# -ge 2 ]] || die "$1 needs a value"
                PASSTHROUGH+=("$1" "$2"); shift 2 ;;
            -h|--help)     usage; exit 0 ;;
            *) die "Unknown flag: $1 (see ./run.sh --help)" ;;
        esac
    done
fi

if [[ "$MODE" != "tasks" && ( "${1:-}" == "-h" || "${1:-}" == "--help" ) ]]; then
    usage; exit 0
fi

PYTHON=$(find_python) || die "No python with torch and timm found. Activate the vision container or .venv, or set EMBEDDING_PYTHON."
echo "Using python: $PYTHON ($("$PYTHON" --version 2>&1))"

case "$MODE" in
    prepare)
        run_py "$SCRIPT_DIR/lib/prepare_dataset.py" "$@"
        exit 0
        ;;
    experiment)
        name="${1:-}"
        [[ -n "$name" ]] || die "experiment needs a name: finetune-head | finetune-arcface"
        shift
        case "$name" in
            finetune-head)   script="finetune_head.py" ;;
            finetune-arcface) script="finetune_arcface.py" ;;
            *) die "Unknown experiment: $name" ;;
        esac
        run_py "$SCRIPT_DIR/experiments/$script" "$@"
        exit 0
        ;;
esac

declare -a SELECTED=()
if $ALL; then
    SELECTED=("${ALL_TASKS[@]}")
elif [[ -n "$TASKS" ]]; then
    IFS=',' read -ra SELECTED <<< "$TASKS"
else
    while IFS= read -r t; do SELECTED+=("$t"); done < <(prompt_for_tasks)
fi

for task in "${SELECTED[@]}"; do
    valid=false
    for known in "${ALL_TASKS[@]}"; do [[ "$task" == "$known" ]] && valid=true; done
    $valid || die "Unknown task: $task (valid: ${ALL_TASKS[*]})"
    if [[ "$task" == "e2e_eval" && -z "$SOURCE" ]]; then
        die "e2e_eval needs --source (a YOLO-seg export)"
    fi
    if [[ "$task" == "e2e_calibrate" && -z "$SOURCE" ]] && ! $FROM_CACHE; then
        die "e2e_calibrate needs --source, or --from-cache to reuse the crops cache"
    fi
done

echo "Embedding-gallery benchmark - $(date)"

FAILED=false
for task in "${SELECTED[@]}"; do
    echo ""
    echo "== Task: $task =="
    if ! run_py "$SCRIPT_DIR/tasks.py" "$task" ${PASSTHROUGH[@]+"${PASSTHROUGH[@]}"}; then
        echo "  ERROR: task '$task' failed" >&2
        FAILED=true
    fi
done

echo ""
if $FAILED; then
    echo "Done with failures."
    exit 1
fi
echo "Done. Results in: $RESULTS_DIR"
