#!/bin/bash
# Embedding-gallery benchmark driver (Phase 0 + Phase 1 of the few-shot
# object recognition plan). Run inside the vision container — needs
# ultralytics (box recall) and timm/torch/clip/PIL (embedding benchmark).
#
# Usage:
#   ./run.sh boxes                       # Phase 0: box-proposer recall
#   ./run.sh embeddings                  # Phase 1: all backbones in models.json
#   ./run.sh embeddings --backbones dinov2_vits14 --backbones clip_vit_b32
#
# Both phases need real data under data/ first — see README.md.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

MODE="${1:-}"
shift || true

case "$MODE" in
    boxes)
        python3 box_recall_eval.py "$@"
        ;;
    embeddings)
        python3 report.py "$@"
        ;;
    *)
        echo "Usage: ./run.sh {boxes|embeddings} [extra args]"
        exit 1
        ;;
esac
