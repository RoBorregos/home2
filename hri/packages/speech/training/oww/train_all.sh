#!/usr/bin/env bash
# Runs train.py once per wake-word config, producing .onnx files under
# .data/models/. Safe to re-run after a crash: train.py skips already-generated
# positive/negative clips when it sees them on disk.
#
# Assumes setup.sh has already prepared .venv/ and .data/.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

if [[ ! -d "$HERE/.venv" ]]; then
  echo "ERROR: .venv not found. Run setup.sh first." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$HERE/.venv/bin/activate"

CONFIGS=(
  "configs/frida.yaml"
  "configs/yes.yaml"
  "configs/no.yaml"
  "configs/stop.yaml"
)

for config in "${CONFIGS[@]}"; do
  echo "=============================================================="
  echo "[train_all] training $config"
  echo "=============================================================="
  python "$HERE/train.py" \
    --training_config "$config" \
    --generate_clips \
    --augment_clips \
    --train_model
done

echo "[train_all] all four models produced under $HERE/.data/models/"
