#!/bin/bash
# Shared helpers for the experiment launchers (exp*.sh). Run them on the robot host.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LLM_URL="http://localhost:11434"

hri_running() {
  [ -n "$(docker ps -q -f name=home2-hri-ros)" ]
}

# Start the HRIC HRI stack (speech, nlp + dialogue agent, LLM, display) in a
# detached screen if it is not running yet. $1 = LLM context size.
start_hri() {
  local ctx="${1:-2048}"
  if hri_running; then
    echo "HRI already running."
  else
    echo "Starting HRI (./run.sh hri --hric, LLAMA_CTX_SIZE=$ctx) in screen 'hri'..."
    screen -dmS hri bash -c "cd '$REPO' && LLAMA_CTX_SIZE=$ctx ./run.sh hri --hric 2>&1 | tee /tmp/hri_run.log"
  fi

  echo -n "Waiting for the LLM server"
  for _ in $(seq 1 90); do
    curl -s "$LLM_URL/v1/models" | grep -q '"id"' && break
    echo -n "."
    sleep 2
  done
  echo

  local n_ctx
  n_ctx=$(curl -s "$LLM_URL/props" | python3 -c "import json,sys; print(json.load(sys.stdin)['default_generation_settings']['n_ctx'])" 2>/dev/null)
  if [ -n "$n_ctx" ] && [ "$n_ctx" -lt "$ctx" ]; then
    echo "WARNING: the LLM is running with context $n_ctx (< $ctx). Restart HRI with:"
    echo "  LLAMA_CTX_SIZE=$ctx ./run.sh hri --hric"
  fi
}

# Run one experiment in the integration container. $1 = run.sh flag, rest = experiment args.
run_experiment() {
  local flag="$1"
  shift
  cd "$REPO" && EXP_ARGS="$*" ./run.sh integration "$flag"
}
