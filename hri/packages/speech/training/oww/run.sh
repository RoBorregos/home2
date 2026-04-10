#!/usr/bin/env bash
# Top-level "one command" entry point for retraining all four wake-word
# models and installing them into hri/packages/speech/assets/oww/.
#
# Usage:
#   ./hri/packages/speech/training/oww/run.sh
#
# Pipeline:
#   1. setup.sh      — create venv, install deps, clone piper-sample-generator,
#                      download MIT IR / FMA / AudioSet / ACAV100M / dinner-party eval.
#   2. train_all.sh  — train frida / yes / no / stop via train.py.
#   3. integrate.sh  — copy .onnx outputs into the speech package assets dir
#                      (backing up the existing files to *.onnx.bak first).
#
# Re-running after a crash is cheap: each step is idempotent.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Non-interactive so unattended runs on a remote CUDA box don't block on the
# download confirmation prompt.
export OWW_SETUP_NONINTERACTIVE=1

bash "$HERE/setup.sh"
bash "$HERE/train_all.sh"
bash "$HERE/integrate.sh"

echo
echo "[run] pipeline complete."
echo "[run] new models are live under hri/packages/speech/assets/oww/"
echo "[run] backups of the previous models are at <word>.onnx.bak"
