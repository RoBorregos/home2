#!/usr/bin/env bash
# Copies freshly trained <word>.onnx files from .data/models/ into the
# hri/packages/speech/assets/oww/ directory that kws_oww.py loads at runtime.
#
# Existing .onnx assets are backed up to <word>.onnx.bak before being
# overwritten, so the user can roll back with a single mv if the new models
# misbehave. The .tflite files are left alone — kws_oww.py only loads *.onnx
# (see hri/packages/speech/scripts/kws_oww.py:66).
#
# Fails loudly if any of the four expected models is missing, so a partial
# training run never silently overwrites good models.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DIR="$HERE/.data/models"
ASSETS_DIR="$(cd "$HERE/../../assets/oww" && pwd)"

WORDS=(frida yes no stop)

# ---- 1. Verify every trained model exists before touching anything ---------
missing=()
for word in "${WORDS[@]}"; do
  src="$MODELS_DIR/$word.onnx"
  if [[ ! -f "$src" ]]; then
    missing+=("$src")
  fi
done

if (( ${#missing[@]} > 0 )); then
  echo "ERROR: refusing to integrate — the following trained models are missing:" >&2
  for m in "${missing[@]}"; do
    echo "  - $m" >&2
  done
  echo "Run train_all.sh until it completes successfully, then re-run integrate.sh." >&2
  exit 1
fi

# ---- 2. Back up existing assets and copy in the new ones -------------------
echo "[integrate] target: $ASSETS_DIR"
for word in "${WORDS[@]}"; do
  src="$MODELS_DIR/$word.onnx"
  dst="$ASSETS_DIR/$word.onnx"
  if [[ -f "$dst" ]]; then
    cp -f "$dst" "$dst.bak"
    echo "[integrate] backed up $word.onnx -> $word.onnx.bak"
  fi
  cp -f "$src" "$dst"
  echo "[integrate] installed $word.onnx"
done

echo "[integrate] done. kws_oww.py will pick up the new models on next launch."
echo "[integrate] to roll back, restore from the *.onnx.bak files in $ASSETS_DIR"
