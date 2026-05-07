#!/bin/bash
# Benchmark LLM backends: measures time-to-first-token (TTFT) and tokens/second.
# After the generic benchmark, runs the real extract_data test cases from
# hri/packages/nlp/test/data_extractor.json and reports latency + accuracy.
#
# Usage:
#   ./benchmark-llm.sh [--runs N] [--data-extractor-file PATH] [endpoint1 label1] [endpoint2 label2] ...
#
# Examples:
#   # Compare two llama.cpp instances
#   ./benchmark-llm.sh \
#     "http://localhost:11434/v1" "llama.cpp-qwen3" \
#     "http://localhost:11435/v1" "llama.cpp-rbrgs"
#
#   # Default: test llama.cpp on port 11434 with qwen3
#   ./benchmark-llm.sh

set -euo pipefail

RUNS=5
PROMPT=""
MODEL_OVERRIDE=""
USECASE=""
DATA_EXTRACTOR_FILE=""

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

# ---------------------------------------------------------------------------
# Parse args
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --runs) RUNS="$2"; shift 2 ;;
        --prompt) PROMPT="$2"; shift 2 ;;
        --model) MODEL_OVERRIDE="$2"; shift 2 ;;
        --usecase) USECASE="$2"; shift 2 ;;
        --data-extractor-file) DATA_EXTRACTOR_FILE="$2"; shift 2 ;;
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

# Default endpoint
if [[ ${#ENDPOINTS[@]} -eq 0 ]]; then
    ENDPOINTS=("http://localhost:11434/v1")
    LABELS=("llama.cpp-qwen3")
fi

# Default data extractor test file (host path; override if running inside Docker)
if [[ -z "$DATA_EXTRACTOR_FILE" ]]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
    DATA_EXTRACTOR_FILE="$REPO_ROOT/hri/packages/nlp/test/data_extractor.json"
fi

# ---------------------------------------------------------------------------
# Dependency checks
# ---------------------------------------------------------------------------
command -v jq     >/dev/null 2>&1 || { echo "ERROR: jq is required.   apt-get install -y jq";     exit 1; }
command -v curl   >/dev/null 2>&1 || { echo "ERROR: curl is required.";                            exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "ERROR: python3 is required for stats.";              exit 1; }

# ---------------------------------------------------------------------------
# Stats helper: min mean max from newline-separated floats
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# Resolve live model name from endpoint
# ---------------------------------------------------------------------------
resolve_model() {
    local base_url="$1"
    local override="$2"
    if [[ -n "$override" ]]; then
        echo "$override"
        return
    fi
    local m
    m=$(curl -sf "$base_url/models" 2>/dev/null | jq -r '.data[0].id // empty')
    echo "${m:-placeholder}"
}

# ---------------------------------------------------------------------------
# Check endpoint reachability; print status; return 0 if reachable
# ---------------------------------------------------------------------------
check_endpoint() {
    local base_url="$1"
    local health_url="${base_url%/v1}/health"
    if curl -sf "$health_url" >/dev/null 2>&1; then
        echo "    Health: OK ($health_url)"
        return 0
    elif curl -sf "$base_url/models" >/dev/null 2>&1; then
        local actual_model
        actual_model=$(curl -sf "$base_url/models" | jq -r '.data[0].id // "unknown"')
        echo "    Health: OK (model: $actual_model)"
        return 0
    else
        echo "    Health: UNREACHABLE — skipping"
        return 1
    fi
}

# ---------------------------------------------------------------------------
# Stream one request; set globals: g_ttft g_total g_tokens g_ok
# ---------------------------------------------------------------------------
stream_request() {
    local base_url="$1"
    local payload="$2"

    g_ttft=0; g_total=0; g_tokens=0; g_ok=false

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
        return
    fi

    [[ "$completion_tokens" -eq 0 ]] && completion_tokens=50

    g_ttft=$(( t_first - t_start ))
    g_total=$(( t_end   - t_start ))
    g_tokens=$completion_tokens
    g_ok=true
}

# ---------------------------------------------------------------------------
# Generic benchmark section
# ---------------------------------------------------------------------------
run_generic_benchmark() {
    local base_url="$1"
    local label="$2"

    echo
    echo "=== $label ==="
    echo "    URL : $base_url/chat/completions"
    echo "    Runs: $RUNS"

    check_endpoint "$base_url" || return

    local model
    model=$(resolve_model "$base_url" "$MODEL_OVERRIDE")

    local ttft_vals="" tps_vals=""

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

        stream_request "$base_url" "$payload"

        if [[ "$g_ok" != "true" ]]; then
            echo "FAILED (no response or stream error)"
            continue
        fi

        local tps
        tps=$(python3 -c "print(f'{($g_tokens / ($g_total / 1000)):.1f}')")
        printf "TTFT=%dms  total=%dms  ~%.1f tok/s\n" "$g_ttft" "$g_total" "$tps"

        ttft_vals+="${g_ttft}"$'\n'
        tps_vals+="${tps}"$'\n'
    done

    echo "  -- TTFT (ms) --"
    echo "$ttft_vals" | stats
    echo "  -- Tokens/s --"
    echo "$tps_vals" | stats
}

# ---------------------------------------------------------------------------
# Extract-data benchmark: runs real test cases from data_extractor.json
# ---------------------------------------------------------------------------
run_data_extractor_benchmark() {
    local base_url="$1"
    local label="$2"

    echo
    echo "=== $label — data_extractor test cases ==="
    echo "    URL : $base_url/chat/completions"
    echo "    File: $DATA_EXTRACTOR_FILE"

    if [[ ! -f "$DATA_EXTRACTOR_FILE" ]]; then
        echo "    SKIP: file not found ($DATA_EXTRACTOR_FILE)"
        echo "    Hint: pass --data-extractor-file <path>"
        return
    fi

    check_endpoint "$base_url" || return

    local model
    model=$(resolve_model "$base_url" "$MODEL_OVERRIDE")

    # Build the shared system prompt (matches dialogs.py exactly)
    local system_prompt
    system_prompt='You will receive a text (`full_text`) and a specific target (`extract_data`). (Optional) Additional explanation (`explanation`) to help clarify ambiguous cases. Your task is to extract and return the closest relevant word or phrase that directly answers the target.

### Extraction Rules:
- Return the MOST RELEVANT WORD OR PHRASE that best corresponds to `extract_data`, considering its contextual meaning within the sentence.
- Do NOT return the target word (`extract_data`) itself unless it is the best available answer.
- If multiple possible matches exist, return the MOST CONTEXTUALLY RELEVANT one (e.g., a noun or phrase describing the requested information).
- If no relevant match is found, return an empty string ("").
- If `full_text` is missing, empty, or consists of only a single word or short phrase that directly corresponds to `extract_data`, return `full_text` as the result.
- If present, use the `explanation` to help clarify ambiguous cases.

### Examples:

#### Example 1:
**INPUT:**
<full_text>
    There is a cat in the house.
</full_text>
<extract_data>
    food
</extract_data>
**OUTPUT:**
{"data": "", "rationale": ""}

#### Example 2:
**INPUT:**
<full_text>
    The restaurant serves delicious Italian food.
</full_text>
<extract_data>
    food
</extract_data>
**OUTPUT:**
{"data": "Italian food", "rationale": ""}

#### Example 3:
**INPUT:**
<full_text>
    My name is Juan and I like lemonade.
</full_text>
<extract_data>
    drink
</extract_data>
**OUTPUT:**
{"data": "lemonade", "rationale": ""}

#### Example 4:
**INPUT:**
<full_text>
    Juan and I like to play basketball.
</full_text>
<extract_data>
    name
</extract_data>
**OUTPUT:**
{"data": "Juan", "rationale": ""}

#### Example 5:
**INPUT:**
<full_text>
    Elis
</full_text>
<extract_data>
    name
</extract_data>
**OUTPUT:**
{"data": "Elis", "rationale": ""}

Ensure that the extracted data is always **the most contextually relevant** answer, not simply the target term itself.'

    local n_cases
    n_cases=$(jq 'length' "$DATA_EXTRACTOR_FILE")
    echo "    Cases: $n_cases"
    echo

    local ttft_vals="" total_vals="" tps_vals=""
    local passed=0 failed=0 errors=0

    for idx in $(seq 0 $(( n_cases - 1 ))); do
        local input_text query context expected
        input_text=$(jq -r ".[$idx][0]" "$DATA_EXTRACTOR_FILE")
        query=$(jq -r ".[$idx][1]" "$DATA_EXTRACTOR_FILE")
        context=$(jq -r ".[$idx][2]" "$DATA_EXTRACTOR_FILE")
        expected=$(jq -r ".[$idx][3]" "$DATA_EXTRACTOR_FILE")

        # Build user content (matches dialogs.py)
        local user_content
        user_content="<full_text>${input_text}</full_text>"$'\n'"<extract_data>${query}</extract_data>"
        if [[ -n "$context" ]]; then
            user_content+=$'\n'"<explanation>${context}</explanation>"
        fi

        local payload
        payload=$(jq -n \
            --arg model  "$model" \
            --arg system "$system_prompt" \
            --arg user   "$user_content" \
            '{
                model: $model,
                stream: true,
                max_tokens: 60,
                temperature: 0.5,
                messages: [
                    {role: "system", content: $system},
                    {role: "user",   content: $user}
                ]
            }')

        printf "  [%2d/%d] %-45s -> expected: %-20s " \
            $(( idx + 1 )) "$n_cases" \
            "\"${input_text:0:43}\"" \
            "\"$expected\""

        stream_request "$base_url" "$payload"

        if [[ "$g_ok" != "true" ]]; then
            echo "ERROR (no response)"
            (( errors++ )) || true
            continue
        fi

        # Collect the full streamed response text to extract the "data" field
        local raw_response
        raw_response=$(curl -sS -N \
            -H "Content-Type: application/json" \
            -H "Authorization: Bearer ollama" \
            -d "$payload" \
            "$base_url/chat/completions" 2>/dev/null \
            | grep '^data: ' \
            | grep -v '^\[DONE\]' \
            | sed 's/^data: //' \
            | jq -r '.choices[0].delta.content // empty' 2>/dev/null \
            | tr -d '\n')

        # Try to extract "data" field from JSON response
        local extracted
        extracted=$(echo "$raw_response" | python3 - <<'PYEOF'
import sys, json, re
raw = sys.stdin.read().strip()
try:
    obj = json.loads(raw)
    print(obj.get("data", raw))
    sys.exit(0)
except Exception:
    pass
m = re.search(r'"data"\s*:\s*"([^"]*)"', raw)
if m:
    print(m.group(1))
else:
    print(raw[:40])
PYEOF
        2>/dev/null || echo "$raw_response")

        local tps
        tps=$(python3 -c "print(f'{($g_tokens / ($g_total / 1000)):.1f}')" 2>/dev/null || echo "0")

        # Case-insensitive exact match
        local ok=false
        if [[ "${extracted,,}" == "${expected,,}" ]]; then
            ok=true
            (( passed++ )) || true
        else
            (( failed++ )) || true
        fi

        local status_icon
        $ok && status_icon="PASS" || status_icon="FAIL"

        printf "%s  got: %-20s  TTFT=%dms total=%dms ~%.1f tok/s\n" \
            "$status_icon" "\"${extracted:0:18}\"" "$g_ttft" "$g_total" "$tps"

        ttft_vals+="${g_ttft}"$'\n'
        total_vals+="${g_total}"$'\n'
        tps_vals+="${tps}"$'\n'
    done

    local total_cases=$(( passed + failed + errors ))
    echo
    echo "  ---- data_extractor results ----"
    printf "  Accuracy : %d/%d passed (%.1f%%)\n" \
        "$passed" "$total_cases" \
        "$(python3 -c "print(f'{$passed/$total_cases*100:.1f}')" 2>/dev/null || echo 0)"
    echo "  -- TTFT (ms) --"
    echo "$ttft_vals" | stats
    echo "  -- Total latency (ms) --"
    echo "$total_vals" | stats
    echo "  -- Tokens/s --"
    echo "$tps_vals" | stats
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
echo "LLM Backend Benchmark"
if [[ -n "$USECASE" ]]; then
    echo "Use case: $USECASE"
else
    echo "Prompt: $PROMPT"
fi
echo "Runs per endpoint: $RUNS"

for i in "${!ENDPOINTS[@]}"; do
    run_generic_benchmark "${ENDPOINTS[$i]}" "${LABELS[$i]}"
done

echo
echo "========================================"
echo "  data_extractor accuracy + latency"
echo "========================================"

for i in "${!ENDPOINTS[@]}"; do
    run_data_extractor_benchmark "${ENDPOINTS[$i]}" "${LABELS[$i]}"
done

echo
echo "Done."
