#!/bin/bash
# 4-way LLM backend benchmark — Qwen3 30B-A3B (MoE) on Jetson Orin AGX.
#
# Starts each backend in turn (one at a time to avoid VRAM contention),
# runs benchmark-llm.sh against all NLP use cases, and emits:
#   - benchmark-results.csv   raw per-run numbers
#   - benchmark-results.md    human-readable summary table
#
# Backends:
#   llamacpp  port 11438   GGUF Q4_K_M
#   ollama    port 11439   ollama-managed Q4_K_M
#   vllm      port 11436   AWQ
#   mlc       port 11437   q4f16_1
#
# Usage:
#   ./run-4way-benchmark.sh [--runs N] [--backends llamacpp,ollama,vllm,mlc]
#
# Run from docker/hri/scripts/. ROLE=benchmark is set for all containers
# so the entrypoints serve the MoE model and skip production presets.

set -euo pipefail

RUNS=3
BACKENDS_CSV="llamacpp,ollama,vllm,mlc"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ASSETS_DIR="$(realpath "$SCRIPT_DIR/../../../hri/packages/nlp/assets")"
TEGRA_DIR="/usr/lib/aarch64-linux-gnu/tegra"
OUT_DIR="$SCRIPT_DIR"
CSV="$OUT_DIR/benchmark-results.csv"
MD="$OUT_DIR/benchmark-results.md"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --runs) RUNS="$2"; shift 2 ;;
        --backends) BACKENDS_CSV="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

IFS=',' read -r -a BACKENDS <<< "$BACKENDS_CSV"

# ── Per-backend launch metadata ──────────────────────────────────────────────
declare -A PORT=(
    [llamacpp]=11438
    [ollama]=11439
    [vllm]=11436
    [mlc]=11437
)
declare -A HEALTH=(
    [llamacpp]=/health
    [ollama]=/api/version
    [vllm]=/health
    [mlc]=/v1/models
)
declare -A CONTAINER=(
    [llamacpp]=benchmark-llamacpp
    [ollama]=benchmark-ollama
    [vllm]=benchmark-vllm
    [mlc]=benchmark-mlc
)

current_container=""
cleanup() {
    if [[ -n "$current_container" ]]; then
        echo
        echo "Stopping $current_container..."
        docker stop "$current_container" >/dev/null 2>&1 || true
        docker rm "$current_container" >/dev/null 2>&1 || true
        current_container=""
    fi
}
trap cleanup EXIT

wait_for() {
    local url="$1" label="$2" max="${3:-60}"
    local n=0
    echo -n "Waiting for $label..."
    while ! curl -sf "$url" >/dev/null 2>&1; do
        n=$((n+1))
        if [ $n -ge "$max" ]; then
            echo " TIMEOUT"
            return 1
        fi
        echo -n "."
        sleep 5
    done
    echo " ready."
}

launch_llamacpp() {
    docker run -d --name "${CONTAINER[llamacpp]}" \
        --runtime=nvidia --network=host \
        -e ROLE=benchmark \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
        -e LD_LIBRARY_PATH="$TEGRA_DIR" \
        -v "$ASSETS_DIR:/root/.cache/huggingface" \
        -v "$TEGRA_DIR:$TEGRA_DIR" \
        -v "$SCRIPT_DIR/llama-cpp-entrypoint.sh:/scripts/entrypoint.sh" \
        ghcr.io/nvidia-ai-iot/llama_cpp:latest-jetson-orin \
        /bin/bash /scripts/entrypoint.sh >/dev/null
}

launch_ollama() {
    docker run -d --name "${CONTAINER[ollama]}" \
        --runtime=nvidia --network=host \
        -e ROLE=benchmark \
        -e OLLAMA_MODELS=/ollama \
        -e OLLAMA_HOST=0.0.0.0:11439 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
        -v "$ASSETS_DIR:/ollama" \
        -v "$TEGRA_DIR:$TEGRA_DIR" \
        -v "$SCRIPT_DIR/ollama-entrypoint.sh:/scripts/entrypoint.sh" \
        roborregos/home2:hri-ollama-l4t \
        /bin/bash -c "OLLAMA_HOST=0.0.0.0:11439 /scripts/entrypoint.sh && tail -f /dev/null" >/dev/null
}

launch_vllm() {
    docker run -d --name "${CONTAINER[vllm]}" \
        --runtime=nvidia --network=host --shm-size 8g \
        -e ROLE=benchmark \
        -e VLLM_PORT=11436 \
        -e VLLM_MODEL="${VLLM_MODEL:-Qwen/Qwen3-30B-A3B-AWQ}" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
        -e LD_LIBRARY_PATH="$TEGRA_DIR" \
        -e HF_HUB_ENABLE_HF_TRANSFER=1 \
        -v "$ASSETS_DIR:/root/.cache/huggingface" \
        -v "$TEGRA_DIR:$TEGRA_DIR" \
        -v "$SCRIPT_DIR/vllm-entrypoint.sh:/scripts/entrypoint.sh" \
        dustynv/vllm:0.7.4-r36.4.0 \
        /bin/bash /scripts/entrypoint.sh >/dev/null
}

launch_mlc() {
    docker run -d --name "${CONTAINER[mlc]}" \
        --runtime=nvidia --network=host --shm-size 8g \
        -e ROLE=benchmark \
        -e MLC_PORT=11437 \
        -e MLC_MODEL="${MLC_MODEL:-HF://mlc-ai/Qwen3-30B-A3B-q4f16_1-MLC}" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
        -e LD_LIBRARY_PATH="$TEGRA_DIR" \
        -v "$ASSETS_DIR:/root/.cache/huggingface" \
        -v "$ASSETS_DIR/mlc-cache:/root/.cache/mlc_llm" \
        -v "$TEGRA_DIR:$TEGRA_DIR" \
        -v "$SCRIPT_DIR/mlc-entrypoint.sh:/scripts/entrypoint.sh" \
        dustynv/mlc:0.20.0-r36.4.0 \
        /bin/bash /scripts/entrypoint.sh >/dev/null
}

echo "backend,use_case,run,ttft_ms,total_ms,tokens_per_sec" > "$CSV"

run_backend() {
    local backend="$1"
    local port="${PORT[$backend]}"
    local health="${HEALTH[$backend]}"
    local cname="${CONTAINER[$backend]}"

    echo
    echo "════════════════════════════════════════════════════"
    echo "  BACKEND: $backend  (port $port)"
    echo "════════════════════════════════════════════════════"

    # vLLM and MLC can take many minutes on cold start; bump max waits.
    local max_waits=60
    [[ "$backend" == "vllm" ]] && max_waits=180
    [[ "$backend" == "mlc"  ]] && max_waits=240

    docker rm -f "$cname" >/dev/null 2>&1 || true
    current_container="$cname"

    case "$backend" in
        llamacpp) launch_llamacpp ;;
        ollama)   launch_ollama ;;
        vllm)     launch_vllm ;;
        mlc)      launch_mlc ;;
    esac

    if ! wait_for "http://localhost:${port}${health}" "$backend" "$max_waits"; then
        echo "ERROR: $backend failed to come up — skipping."
        docker logs "$cname" 2>&1 | tail -40 || true
        cleanup
        return 1
    fi

    # Hand off to the existing per-endpoint benchmark and tee for parsing.
    local log
    log=$(mktemp)
    bash "$SCRIPT_DIR/benchmark-llm.sh" \
        --runs "$RUNS" \
        --all \
        "http://localhost:${port}/v1" "$backend" | tee "$log"

    # Extract per-run lines of the form: "TTFT=123ms  total=456ms  ~78.9 tok/s"
    # Group by the most recently announced USE CASE banner.
    python3 - "$backend" "$log" "$CSV" <<'PY'
import re, sys
backend, log_path, csv_path = sys.argv[1], sys.argv[2], sys.argv[3]
uc, run_idx = None, 0
line_re = re.compile(r"TTFT=(\d+)ms\s+total=(\d+)ms\s+~([0-9.]+)\s+tok/s")
uc_re   = re.compile(r"USE CASE:\s+(\S+)")
with open(log_path) as f, open(csv_path, "a") as out:
    for line in f:
        m = uc_re.search(line)
        if m:
            uc, run_idx = m.group(1), 0
            continue
        m = line_re.search(line)
        if m and uc:
            run_idx += 1
            ttft, total, tps = m.groups()
            out.write(f"{backend},{uc},{run_idx},{ttft},{total},{tps}\n")
PY
    rm -f "$log"

    cleanup
}

for b in "${BACKENDS[@]}"; do
    if [[ -z "${PORT[$b]:-}" ]]; then
        echo "Unknown backend: $b — skipping."
        continue
    fi
    run_backend "$b" || true
done

# ── Markdown report ──────────────────────────────────────────────────────────
python3 - "$CSV" "$MD" <<'PY'
import csv, statistics, sys
from collections import defaultdict

csv_path, md_path = sys.argv[1], sys.argv[2]
rows = list(csv.DictReader(open(csv_path)))
if not rows:
    open(md_path, "w").write("# Benchmark results\n\nNo data collected.\n")
    sys.exit(0)

groups = defaultdict(list)
for r in rows:
    groups[(r["backend"], r["use_case"])].append(r)

backends  = sorted({r["backend"] for r in rows})
use_cases = sorted({r["use_case"] for r in rows})

def fmt(vals, key, prec):
    nums = [float(v[key]) for v in vals]
    return f"{statistics.mean(nums):.{prec}f}" if nums else "—"

with open(md_path, "w") as f:
    f.write("# 4-way LLM backend benchmark — Qwen3 30B-A3B (MoE)\n\n")
    f.write(f"Backends: {', '.join(backends)}\n\n")

    f.write("## Tokens per second (mean)\n\n")
    f.write("| use case | " + " | ".join(backends) + " |\n")
    f.write("|" + "---|" * (len(backends) + 1) + "\n")
    for uc in use_cases:
        cells = [fmt(groups[(b, uc)], "tokens_per_sec", 1) for b in backends]
        f.write(f"| {uc} | " + " | ".join(cells) + " |\n")

    f.write("\n## Time to first token (ms, mean)\n\n")
    f.write("| use case | " + " | ".join(backends) + " |\n")
    f.write("|" + "---|" * (len(backends) + 1) + "\n")
    for uc in use_cases:
        cells = [fmt(groups[(b, uc)], "ttft_ms", 0) for b in backends]
        f.write(f"| {uc} | " + " | ".join(cells) + " |\n")
PY

echo
echo "Done."
echo "  CSV:      $CSV"
echo "  Summary:  $MD"
