#!/bin/bash
# Run the NLP benchmark inside Docker.
# Assumes an Ollama server is already running on the host.
#
# Usage (desde hri/packages/nlp/benchmark/):
#   ./run-benchmark.sh                          # usa config.yaml defaults
#   ./run-benchmark.sh --models qwen3 gemma3:4b
#   ./run-benchmark.sh --tasks extract_data --no-perf
#   ./run-benchmark.sh --models qwen3 --runs 5

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TEST_DATA_DIR="$(realpath "$SCRIPT_DIR/../test")"
RESULTS_DIR="$SCRIPT_DIR/results"

IMAGE="benchmark-nlp"

echo "Building benchmark image..."
docker build -t "$IMAGE" "$SCRIPT_DIR"

mkdir -p "$RESULTS_DIR"

echo "Running benchmark (Ollama at host:11434)..."
docker run --rm \
    --network host \
    -v "$RESULTS_DIR:/benchmark/results" \
    -v "$TEST_DATA_DIR:/test_data:ro" \
    "$IMAGE" "$@"
