#!/usr/bin/env bash
# Idempotent one-time environment setup for the openWakeWord training pipeline.
#
# Creates an isolated venv at .venv/, installs pinned deps, clones
# piper-sample-generator under .third_party/, and downloads every dataset
# train.py needs into .data/.
#
# Safe to re-run: every step short-circuits if its output already exists.
#
# Requirements:
#   - python3.10 or python3.11 on PATH (the repo-level /Desktop/home2/venv
#     runs on 3.13 which is too new for several training deps, so we build a
#     dedicated venv here instead).
#   - git (to clone piper-sample-generator)
#   - ~15 GB free disk under this directory for the datasets.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

VENV_DIR="$HERE/.venv"
THIRD_PARTY_DIR="$HERE/.third_party"
PIPER_DIR="$THIRD_PARTY_DIR/piper-sample-generator"

# ---- 1. Pick a supported Python interpreter --------------------------------
PYTHON_BIN=""
for candidate in python3.11 python3.10; do
  if command -v "$candidate" >/dev/null 2>&1; then
    PYTHON_BIN="$candidate"
    break
  fi
done

if [[ -z "$PYTHON_BIN" ]]; then
  echo "ERROR: need python3.10 or python3.11 on PATH (found neither)." >&2
  echo "       The repo-level venv at /Desktop/home2/venv runs Python 3.13," >&2
  echo "       which is too new for several training dependencies." >&2
  echo "       Install one of the supported versions and re-run setup.sh." >&2
  exit 1
fi
echo "[setup] using interpreter: $($PYTHON_BIN --version) ($(command -v "$PYTHON_BIN"))"

# ---- 2. Create venv if missing ---------------------------------------------
if [[ ! -d "$VENV_DIR" ]]; then
  echo "[setup] creating venv at $VENV_DIR"
  "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip wheel setuptools

# ---- 3. Install Python deps -------------------------------------------------
# Use CUDA wheels when an NVIDIA GPU is visible, CPU wheels otherwise.
if command -v nvidia-smi >/dev/null 2>&1; then
  echo "[setup] NVIDIA GPU detected, installing CUDA torch wheels"
  pip install --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch==2.2.* torchaudio==2.2.*
else
  echo "[setup] no NVIDIA GPU, installing CPU torch wheels"
  pip install --extra-index-url https://download.pytorch.org/whl/cpu \
    torch==2.2.* torchaudio==2.2.*
fi

pip install -r "$HERE/requirements.txt"

# ---- 4. Clone piper-sample-generator ---------------------------------------
mkdir -p "$THIRD_PARTY_DIR"
if [[ ! -d "$PIPER_DIR/.git" ]]; then
  echo "[setup] cloning piper-sample-generator into $PIPER_DIR"
  git clone --depth 1 https://github.com/rhasspy/piper-sample-generator.git "$PIPER_DIR"
else
  echo "[setup] piper-sample-generator already cloned, skipping"
fi

# Install piper-sample-generator's own requirements if it ships a file.
if [[ -f "$PIPER_DIR/requirements.txt" ]]; then
  pip install -r "$PIPER_DIR/requirements.txt"
fi

# ---- 5. Fetch datasets ------------------------------------------------------
# Non-interactive when invoked from run.sh so CI / scripted runs don't block.
if [[ "${OWW_SETUP_NONINTERACTIVE:-0}" == "1" ]]; then
  OWW_DOWNLOAD_YES=1 python "$HERE/download_data.py"
else
  python "$HERE/download_data.py"
fi

echo "[setup] done. Activate the venv with:  source $VENV_DIR/bin/activate"
