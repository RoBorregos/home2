#!/bin/bash
# Benchmark LLM backends: measures time-to-first-token (TTFT) and tokens/second.
# Usage:
#   ./benchmark-llm.sh [--runs N] [endpoint1 label1] [endpoint2 label2] ...
#
# Examples:
#   # Compare llama.cpp vs Ollama (both on this machine, different ports)
#   ./benchmark-llm.sh \
#     "http://localhost:11434/v1" "llama.cpp-qwen3" \
#     "http://localhost:11435/v1" "llama.cpp-rbrgs"
#
#   # Default: just test llama.cpp on port 11434
#   ./benchmark-llm.sh

set -euo pipefail

RUNS=5
PROMPT=""
MODEL_OVERRIDE=""  # Leave empty to let the server pick its loaded model
USECASE=""

ENDPOINTS=()
LABELS=()

# ---------------------------------------------------------------------------
# Real use-case payloads (system + user, matching dialogs.py exactly)
# ---------------------------------------------------------------------------
usecase_payload() {
    local uc="$1"
    local model="$2"
    case "$uc" in
        extract_data)
            jq -n --arg model "$model" '{
                model: $model,
                stream: true,
                max_tokens: 60,
                temperature: 0.5,
                messages: [
                    {role:"system", content:"You will receive a text (`full_text`) and a specific target (`extract_data`). Your task is to extract and return the closest relevant word or phrase that directly answers the target.\nReturn JSON: {\"data\": \"<value or empty string>\"}"},
                    {role:"user",   content:"<full_text>My name is Carlos and I would like a glass of water.</full_text>\n<extract_data>drink</extract_data>"}
                ]
            }'
            ;;
        is_coherent)
            jq -n --arg model "$model" '{
                model: $model,
                stream: true,
                max_tokens: 30,
                temperature: 0.0,
                messages: [
                    {role:"system", content:"Determine if a command is complete and executable by a robot. Output JSON: {\"is_coherent\": true/false}"},
                    {role:"user",   content:"Command: Go to the kitchen and pick up the apple /no_think"}
                ]
            }'
            ;;
        llm_wrapper)
            jq -n --arg model "$model" '{
                model: $model,
                stream: true,
                max_tokens: 80,
                temperature: 0.5,
                messages: [
                    {role:"system", content:"You are an intelligent assistant. Answer clearly and concisely using the provided context.\n\nContext: The robot picked up a red apple from the kitchen table."},
                    {role:"user",   content:"What object did the robot pick up? /no_think"}
                ]
            }'
            ;;
        categorize_shelves)
            jq -n --arg model "$model" '{
                model: $model,
                stream: true,
                max_tokens: 40,
                temperature: 0.5,
                messages: [
                    {role:"system", content:"Assign a unique category to each shelf. Return only JSON: {\"categories\": [\"cat1\",\"cat2\",...]}. Number of categories must match number of shelves."},
                    {role:"user",   content:"Shelves: [[\"apple\",\"banana\"],[\"water\",\"cup\"],[\"chips\"]], table_objects: [\"soda\"]"}
                ]
            }'
            ;;
        *)
            echo "Unknown usecase: $uc" >&2
            exit 1
            ;;
    esac
}

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --runs) RUNS="$2"; shift 2 ;;
        --prompt) PROMPT="$2"; shift 2 ;;
        --model) MODEL_OVERRIDE="$2"; shift 2 ;;
        --usecase) USECASE="$2"; shift 2 ;;
        *)
            ENDPOINTS+=("$1")
            LABELS+=("${2:-$1}")
            shift 2
            ;;
    esac
done

# Default prompt if none set and no usecase
if [[ -z "$PROMPT" && -z "$USECASE" ]]; then
    PROMPT="You are a helpful assistant. In exactly two sentences, describe what a service robot does in a domestic environment."
fi

# Default endpoint if none provided
if [[ ${#ENDPOINTS[@]} -eq 0 ]]; then
    ENDPOINTS=("http://localhost:11434/v1" "http://localhost:11435/v1")
    LABELS=("qwen3.6" "rbrgs")
fi

command -v jq >/dev/null 2>&1 || { echo "ERROR: jq is required. Install with: apt-get install -y jq"; exit 1; }
command -v curl >/dev/null 2>&1 || { echo "ERROR: curl is required."; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "ERROR: python3 is required for stats."; exit 1; }

# Stats helper: min mean max from newline-separated floats
stats() {
    python3 -c '
import sys, statistics
vals = [float(l) for l in sys.stdin.read().strip().split("\n") if l.strip()]
if not vals:
    print("  no data")
    sys.exit(0)
print(f"  min={min(vals):.3f}  mean={statistics.mean(vals):.3f}  max={max(vals):.3f}  (n={len(vals)})")
'
}

run_single() {
    local base_url="$1"
    local label="$2"
    local model="${MODEL_OVERRIDE:-placeholder}"

    echo
    echo "=== $label ==="
    echo "    URL : $base_url/chat/completions"
    echo "    Runs: $RUNS"

    # Check server is reachable
    local health_url="${base_url%/v1}/health"
    if curl -sf "$health_url" >/dev/null 2>&1 || curl -sf "$base_url/models" >/dev/null 2>&1; then
        # Always resolve the loaded model name (skip "default" which is the [*] wildcard preset)
        if [[ -z "$MODEL_OVERRIDE" ]]; then
            model=$(curl -sf "$base_url/models" 2>/dev/null \
                | jq -r '[.data[] | select(.id != "default")] | .[0].id // empty')
            model="${model:-placeholder}"
        fi
        echo "    Health: OK (model: $model)"
    else
        echo "    Health: UNREACHABLE — skipping"
        return
    fi

    local ttft_vals=""
    local tps_vals=""

    for run in $(seq 1 "$RUNS"); do
        printf "    run %d/%d ... " "$run" "$RUNS"

        local payload
        if [[ -n "$USECASE" ]]; then
            payload=$(usecase_payload "$USECASE" "$model")
        else
            payload=$(jq -n \
                --arg model "$model" \
                --arg content "$PROMPT" \
                '{model: $model, messages: [{role: "user", content: $content}], stream: true, max_tokens: 200}')
        fi

        # Capture streaming response with timestamps
        local tmpfile
        tmpfile=$(mktemp)

        local t_start t_first t_end
        t_start=$(date +%s%3N)
        t_first=0
        local completion_tokens=0
        local done_seen=false

        while IFS= read -r line; do
            local now
            now=$(date +%s%3N)
            [[ -z "$line" || "$line" == "data: [DONE]" ]] && continue
            local data="${line#data: }"
            local delta
            delta=$(echo "$data" | jq -r '.choices[0].delta.content // empty' 2>/dev/null)
            if [[ -n "$delta" && "$t_first" -eq 0 ]]; then
                t_first=$now
            fi
            local finish
            finish=$(echo "$data" | jq -r '.choices[0].finish_reason // empty' 2>/dev/null)
            if [[ "$finish" == "stop" || "$finish" == "length" ]]; then
                t_end=$now
                done_seen=true
                # Try to read token count from the stream's usage field
                local usage_tokens
                usage_tokens=$(echo "$data" | jq -r '.usage.completion_tokens // empty' 2>/dev/null)
                [[ -n "$usage_tokens" ]] && completion_tokens=$usage_tokens
            fi
        done < <(curl -sS -N \
            -H "Content-Type: application/json" \
            -H "Authorization: Bearer ollama" \
            -d "$payload" \
            "$base_url/chat/completions" 2>"$tmpfile")

        rm -f "$tmpfile"

        if [[ "$t_first" -eq 0 || "$done_seen" != "true" ]]; then
            echo "FAILED (no response or stream error)"
            continue
        fi

        local ttft=$(( t_first - t_start ))
        local total=$(( t_end - t_start ))

        # Fallback token count: count words in response as rough estimate
        if [[ "$completion_tokens" -eq 0 ]]; then
            completion_tokens=50  # rough fallback; avoids division by zero
        fi

        local tps
        tps=$(python3 -c "print(f'{($completion_tokens / ($total / 1000)):.1f}')")

        printf "TTFT=%dms  total=%dms  ~%.1f tok/s\n" "$ttft" "$total" "$tps"

        ttft_vals+="${ttft}"$'\n'
        tps_vals+="${tps}"$'\n'
    done

    echo "  -- TTFT (ms) --"
    echo "$ttft_vals" | stats
    echo "  -- Tokens/s --"
    echo "$tps_vals" | stats
}

echo "LLM Backend Benchmark"
if [[ -n "$USECASE" ]]; then
    echo "Use case: $USECASE"
else
    echo "Prompt: $PROMPT"
fi
echo "Runs per endpoint: $RUNS"

for i in "${!ENDPOINTS[@]}"; do
    run_single "${ENDPOINTS[$i]}" "${LABELS[$i]}"
done

echo
echo "Done."
