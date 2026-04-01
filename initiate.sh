#!/bin/bash
# initiate.sh - Launch a task across all relevant areas, one tmux session per area
# Usage: ./initiate.sh --gpsr [extra flags like --build --open-display --zed]

TASK=$1
shift
EXTRA_FLAGS=""
WITH_ZED=0
WITH_INTEGRATION=0

for arg in "$@"; do
  case $arg in
    --zed)        WITH_ZED=1 ;;
    --integration) WITH_INTEGRATION=1 ;;
    *)            EXTRA_FLAGS="$EXTRA_FLAGS $arg" ;;
  esac
done
EXTRA_FLAGS="${EXTRA_FLAGS# }"

if [ -z "$TASK" ]; then
  cat << HELP
Usage: ./initiate.sh <task> [flags]

Tasks:
  --gpsr               General Purpose Service Robot
  --hric               Human-Robot Interaction Challenge
  --ppc                Pick and Place Challenge
  --storing-groceries  Storing Groceries
  --finals             Finals routine

Flags:
  --build              Build ROS2 packages before running
  --open-display       Open HRI display (passed only to hri area)
  --recreate           Force container recreation
  --zed                Also start the ZED camera session
  --integration        Also start the integration session

Examples:
  ./initiate.sh --gpsr
  ./initiate.sh --hric --build --open-display --zed
HELP
  exit 0
fi

HOME2_DIR="$(cd "$(dirname "$0")" && pwd)"

# Areas per task
case $TASK in
  --gpsr)              AREAS="hri navigation vision manipulation" ;;
  --hric)              AREAS="hri navigation vision manipulation" ;;
  --ppc)               AREAS="hri navigation vision manipulation" ;;
  --storing-groceries) AREAS="hri navigation vision manipulation" ;;
  --finals)            AREAS="hri navigation vision manipulation" ;;
  *)
    echo "Unknown task: $TASK"
    echo "Run ./initiate.sh --help for available tasks."
    exit 1
    ;;
esac

[ "$WITH_ZED" -eq 1 ]         && AREAS="$AREAS zed"
[ "$WITH_INTEGRATION" -eq 1 ] && AREAS="$AREAS integration"

# Check for session conflicts before starting anything
CONFLICT=0
for area in $AREAS; do
  if tmux has-session -t "$area" 2>/dev/null; then
    echo "Warning: tmux session '$area' already exists. Please run: tmux kill-session -t $area"
    CONFLICT=1
  fi
done
[ "$CONFLICT" -eq 1 ] && exit 1

echo "Starting task '$TASK'"
echo "Areas: $AREAS"

for area in $AREAS; do
  if [ "$area" = "zed" ] || [ "$area" = "integration" ]; then
    CMD="cd $HOME2_DIR && ./run.sh $area"
  elif [ "$area" = "hri" ]; then
    CMD="cd $HOME2_DIR && ./run.sh $area $TASK $EXTRA_FLAGS"
  else
    FILTERED=$(echo "$EXTRA_FLAGS" | sed 's/--open-display//g' | xargs)
    CMD="cd $HOME2_DIR && ./run.sh $area $TASK $FILTERED"
  fi

  tmux new-session -d -s "$area"
  tmux send-keys -t "$area" "$CMD" C-m
  echo "  Started session: $area"
done

echo ""
echo "All sessions started. Attach with: tmux attach -t <area>"
echo "Sessions: $AREAS"
