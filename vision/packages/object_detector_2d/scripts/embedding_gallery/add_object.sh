#!/bin/bash
# Add objects to the few-shot gallery from the photos in gallery_photos/<object>/.
#
# Usage:
#   ./add_object.sh <object_name> [<object_name> ...] [--no-crop]
#   ./add_object.sh --all [--no-crop]
#
# The gallery is built into TENSORRT_CACHE_DIR/gallery (a persistent mount) and
# then copied beside every detectors/registry.py by fetch_models.py. Restart
# ObjectDetect2D afterwards.
set -e

case "${1:-}" in
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    "") sed -n '2,10p' "$0"; exit 1 ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# scripts/ on PYTHONPATH so gallery_build.py can import `detectors` and `embedding_gallery`.
BUILD_RC=0
PYTHONPATH="$SCRIPT_DIR/..:${PYTHONPATH:-}" python3 core/gallery_build.py "$@" || BUILD_RC=$?

# Sync what was built even if another object failed. fetch_models.py exits 1 if
# any custom weight is missing, which says nothing about the gallery sync.
python3 "$SCRIPT_DIR/../../../../scripts/fetch_models.py" || true

echo ""
if [ "$BUILD_RC" -ne 0 ]; then
    echo "[add_object] Some objects failed (see above). Restart ObjectDetect2D for the rest."
    exit "$BUILD_RC"
fi
echo "[add_object] Done. Restart ObjectDetect2D to see the new objects in /vision/detections."
