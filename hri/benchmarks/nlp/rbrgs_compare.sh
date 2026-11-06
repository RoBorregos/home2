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
docker ps -q -f name="^${ROS_CONTAINER}$" | grep -q . \
    || { echo "ERROR: $ROS_CONTAINER not running (start it with ./run.sh hri --gpsr)"; exit 1; }

# Test a copy pointed at localhost so the hardcoded robot IP in client.baml doesn't matter.
TEST_CMD="rm -rf /tmp/rbrgs_bench && mkdir -p /tmp/rbrgs_bench \
    && cp -r /workspace/src/task_manager/task_manager/utils/baml_src /tmp/rbrgs_bench/ \
    && sed -i 's#http://[0-9.]*:$PORT#http://localhost:$PORT#' /tmp/rbrgs_bench/baml_src/client.baml \
    && cd /tmp/rbrgs_bench && baml-cli test --from baml_src/ --parallel 1"
run_tests() { docker exec "$ROS_CONTAINER" bash -c "$TEST_CMD"; }
TOTAL_TESTS=$(cat "$REPO_ROOT"/task_manager/task_manager/utils/baml_src/*.baml | grep -c '^test ' || true)

mkdir -p "$RESULTS_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
TSV="$RESULTS_DIR/rbrgs_compare_${STAMP}.tsv"
METRICS="$RESULTS_DIR/rbrgs_metrics_${STAMP}.tsv"
printf "model\trun\ttest\tstatus\tsecs\n" > "$TSV"
printf "model\tkey\tvalue\n" > "$METRICS"

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
    LOAD_SECS=$((waited + 5))
    echo "  ready after ~${LOAD_SECS}s: $(curl -s "localhost:$PORT/props" | grep -o '"model_path":"[^"]*"')"
}

# llama.cpp counters (needs --metrics): prompt tokens, prompt secs, generated tokens, generation secs.
scrape() {
    curl -sf "localhost:$PORT/metrics" 2>/dev/null | awk '
        $1 == "llamacpp:prompt_tokens_total" { pt = $2 }
        $1 == "llamacpp:prompt_seconds_total" { ps = $2 }
        $1 == "llamacpp:tokens_predicted_total" { gt = $2 }
        $1 == "llamacpp:tokens_predicted_seconds_total" { gs = $2 }
        END { if (pt != "") print pt, ps, gt, gs }'
}

# System RAM in use (MB); on Jetson the GPU shares it, so this includes model weights.
ram_mb() {
    local v=""
    command -v tegrastats >/dev/null && v=$(timeout 3 tegrastats --interval 1000 2>/dev/null | head -1 | grep -o 'RAM [0-9]*' | awk '{print $2}' || true)
    [[ -n "$v" ]] || v=$(free -m | awk '/^Mem:/ { print $3 }')
    echo "$v"
}

warmup() {
    curl -s "localhost:$PORT/v1/chat/completions" -H 'Content-Type: application/json' \
        -d '{"messages":[{"role":"user","content":"hi"}],"max_tokens":8}' >/dev/null || true
}

restore() {
    $RESTORE || return 0
    echo "Restoring default rbrgs model..."
    serve_model "" || true
}
trap restore EXIT

for m in "${MODELS[@]}"; do
    serve_model "$m"
    warmup
    ram=$(ram_mb)
    printf "%s\tload_secs\t%s\n%s\tram_mb\t%s\n" "$m" "$LOAD_SECS" "$m" "$ram" >> "$METRICS"
    echo "  system RAM in use: ${ram} MB"
    for ((r = 1; r <= RUNS; r++)); do
        log="$RESULTS_DIR/rbrgs_${m%.gguf}_${STAMP}_r${r}.log"
        echo "  run $r/$RUNS -> $(basename "$log")"
        before=$(scrape || true)
        start=$SECONDS
        # Full output to the log; one live line per finished test to the terminal.
        run_tests 2>&1 | tee "$log" | sed -u 's/\x1b\[[0-9;]*m//g' | awk -v total="$TOTAL_TESTS" '
            /Test results:/ { s = 1 }
            !s && ($2 == "PASSED" || $2 == "FAILED") && $3 ~ /^GenerateCommandList::/ {
                name = $3; sub(/^GenerateCommandList::/, "", name)
                ms = $1; sub(/s$/, "", ms)
                printf "    [%2d/%d] %-6s %8.0f ms  %s\n", ++i, total, $2, ms * 1000, name; fflush()
            }' || true
        printf "%s\twall_secs\t%s\n" "$m" "$((SECONDS - start))" >> "$METRICS"
        after=$(scrape || true)
        if [[ -n "$before" && -n "$after" ]]; then
            echo "$before $after" | awk -v m="$m" -v OFS='\t' '{
                print m, "prompt_tokens", $5 - $1; print m, "prompt_secs", $6 - $2
                print m, "gen_tokens", $7 - $3;    print m, "gen_secs", $8 - $4 }' >> "$METRICS"
        fi
        # Only the summary block after "Test results:" has one line per test.
        sed 's/\x1b\[[0-9;]*m//g' "$log" | awk -v m="$m" -v r="$r" '
            /Test results:/ { s = 1; next }
            s && ($2 == "PASSED" || $2 == "FAILED") && $3 ~ /^GenerateCommandList::/ {
                secs = $1; sub(/s$/, "", secs); name = $3; sub(/^GenerateCommandList::/, "", name)
                printf "%s\t%s\t%s\t%s\t%s\n", m, r, name, $2, secs
            }' >> "$TSV"
    done
done

SUMMARY="$RESULTS_DIR/rbrgs_summary_${STAMP}.txt"
{
echo
awk -F'\t' -v models="${MODELS[*]}" -v mf="$METRICS" '
    FNR == 1 { next }
    FILENAME == mf { x[$1 SUBSEP $2] += $3; has[$1 SUBSEP $2] = 1; next }
    { n[$1]++; p[$1] += ($4 == "PASSED"); t[$1] += $5; lat[$1 SUBSEP n[$1]] = $5 * 1000 }
    function row(label, fmt, key,    i, m, v) {
        printf "  %-28s", label
        for (i = 1; i <= k; i++) {
            m = M[i]; v = val(m, key)
            printf " %18s", (v == "" ? "n/a" : sprintf(fmt, v))
        }
        print ""
    }
    function val(m, key,    q) {
        if (key == "acc")   return n[m] ? sprintf("%d/%d (%.1f%%)", p[m], n[m], 100 * p[m] / n[m]) : ""
        if (key == "mean")  return n[m] ? t[m] * 1000 / n[m] : ""
        if (key == "med")   return n[m] ? pct(m, 50) : ""
        if (key == "p90")   return n[m] ? pct(m, 90) : ""
        if (key == "min")   return n[m] ? pct(m, 0) : ""
        if (key == "max")   return n[m] ? pct(m, 100) : ""
        if (key == "ptok")  return has[m SUBSEP "prompt_tokens"] && n[m] ? x[m SUBSEP "prompt_tokens"] / n[m] : ""
        if (key == "gtok")  return has[m SUBSEP "gen_tokens"] && n[m] ? x[m SUBSEP "gen_tokens"] / n[m] : ""
        if (key == "pps")   return x[m SUBSEP "prompt_secs"] > 0 ? x[m SUBSEP "prompt_tokens"] / x[m SUBSEP "prompt_secs"] : ""
        if (key == "gps")   return x[m SUBSEP "gen_secs"] > 0 ? x[m SUBSEP "gen_tokens"] / x[m SUBSEP "gen_secs"] : ""
        if (key == "pms")   return has[m SUBSEP "prompt_secs"] && n[m] ? x[m SUBSEP "prompt_secs"] * 1000 / n[m] : ""
        if (key == "gms")   return has[m SUBSEP "gen_secs"] && n[m] ? x[m SUBSEP "gen_secs"] * 1000 / n[m] : ""
        q = m SUBSEP key
        return has[q] ? x[q] : ""
    }
    # Nearest-rank percentile of the per-command latencies (ms).
    function pct(m, q,    i, j, c, s, tmp, idx) {
        c = n[m]
        for (i = 1; i <= c; i++) s[i] = lat[m SUBSEP i]
        for (i = 2; i <= c; i++) { tmp = s[i]; for (j = i - 1; j >= 1 && s[j] > tmp; j--) s[j + 1] = s[j]; s[j + 1] = tmp }
        idx = int(q * c / 100 + 0.999999); if (idx < 1) idx = 1; if (idx > c) idx = c
        return s[idx]
    }
    END {
        k = split(models, M, " ")
        printf "%-30s", "SUMMARY"; for (i = 1; i <= k; i++) { lb = M[i]; sub(/\.gguf$/, "", lb); printf " %18.18s", lb }; print ""
        print "Accuracy"
        row("tests passed", "%s", "acc")
        print "Latency per command (ms, end to end)"
        row("mean", "%.0f", "mean"); row("median", "%.0f", "med"); row("p90", "%.0f", "p90")
        row("min", "%.0f", "min");   row("max", "%.0f", "max")
        print "llama.cpp"
        row("prompt tokens / command", "%.0f", "ptok"); row("generated tokens / command", "%.0f", "gtok")
        row("prefill speed (tok/s)", "%.1f", "pps");    row("generation speed (tok/s)", "%.1f", "gps")
        row("prefill time / command (ms)", "%.0f", "pms"); row("generation time / command (ms)", "%.0f", "gms")
        print "Resources"
        row("load time (s)", "%.0f", "load_secs"); row("system RAM in use (MB)", "%.0f", "ram_mb")
        row("total test time (s)", "%.0f", "wall_secs")
    }' "$METRICS" "$TSV"

echo
echo "Per test (PASS/FAIL, or passes/runs, and mean ms):"
awk -F'\t' -v models="${MODELS[*]}" 'NR > 1 {
        q = $3 SUBSEP $1; c[q] += ($4 == "PASSED"); tot[q]++; ms[q] += $5 * 1000; tests[$3] = 1
    }
    END {
        k = split(models, M, " ")
        printf "%-36s", "test"; for (i = 1; i <= k; i++) { lb = M[i]; sub(/\.gguf$/, "", lb); printf " %18.18s", lb }; print ""
        for (t in tests) {
            printf "%-36s", t
            for (i = 1; i <= k; i++) {
                q = t SUBSEP M[i]
                if (!tot[q]) { printf " %18s", "-"; continue }
                st = (tot[q] == 1) ? (c[q] ? "PASS" : "FAIL") : c[q] "/" tot[q]
                printf " %18s", sprintf("%s %6.0fms", st, ms[q] / tot[q])
            }
            print ""
        }
        if (k == 2) {
            for (t in tests) {
                a = c[t SUBSEP M[1]] / (tot[t SUBSEP M[1]] ? tot[t SUBSEP M[1]] : 1)
                b = c[t SUBSEP M[2]] / (tot[t SUBSEP M[2]] ? tot[t SUBSEP M[2]] : 1)
                if (a > b) only1 = only1 " " t
                else if (b > a) only2 = only2 " " t
                else if (a == 0) none = none " " t
            }
            l1 = M[1]; sub(/\.gguf$/, "", l1); l2 = M[2]; sub(/\.gguf$/, "", l2)
            print "~"
            print "~Only " l1 " passes:" (only1 == "" ? " none" : only1)
            print "~Only " l2 " passes:" (only2 == "" ? " none" : only2)
            print "~Both fail:" (none == "" ? " none" : none)
        }
    }' "$TSV" > "$SUMMARY.tmp"
# Header first, sorted test rows, then the head-to-head lines (marked with ~).
head -1 "$SUMMARY.tmp"
tail -n +2 "$SUMMARY.tmp" | grep -v '^~' | sort
grep '^~' "$SUMMARY.tmp" | sed 's/^~//' || true
rm -f "$SUMMARY.tmp"

echo
echo "Raw results: $TSV"
echo "Metrics:     $METRICS"
} | tee "$SUMMARY"
echo "Summary saved: $SUMMARY"

