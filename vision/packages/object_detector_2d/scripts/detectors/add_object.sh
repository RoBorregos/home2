#!/bin/bash
# One-command wrapper: build the gallery entry for <object_name> from
# gallery_photos/<object_name>/*.jpg, writing into TENSORRT_CACHE_DIR/gallery
# (a persistent mount, same as the TRT engine cache) instead of the source
# tree directly — then reuse fetch_models.py's sync_gallery() to propagate
# that to every detectors/ dir it finds (source, install, any other
# checkout). Writing straight into the source tree (an earlier version of
# this script did) survives a node restart but NOT a fresh clone/container:
# gallery/ is gitignored, so a competition-day fresh setup would silently
# lose every object ever added. The persistent-mount + fetch_models.py path
# is the same offline-safe pattern already used for weights/TRT engines —
# see fetch_models.py's docstring.
#
# Usage:
#   ./add_object.sh <object_name> [photos_glob]
#
# Examples:
#   ./add_object.sh ps5_controller
#   ./add_object.sh ps5_controller "gallery_photos/ps5_controller/*.jpeg"
set -e

OBJECT="$1"
PHOTOS="${2:-gallery_photos/$OBJECT/*.jpg}"

if [ -z "$OBJECT" ]; then
    echo "Usage: $0 <object_name> [photos_glob]"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CACHE_DIR="${TENSORRT_CACHE_DIR:-/workspace/trt_cache}"
GALLERY_DIR="$CACHE_DIR/gallery"

python3 gallery_build.py --object "$OBJECT" --photos "$PHOTOS" --gallery-dir "$GALLERY_DIR"

echo "[add_object] syncing $GALLERY_DIR -> every detectors/ dir (source, install, ...)"
# fetch_models.py exits 1 if ANY custom weight is missing (e.g. a different
# task's .pt this checkout never had) — unrelated to whether the gallery
# sync itself worked, which already happened by the time it could fail. Do
# not let that abort this script under set -e.
python3 "$SCRIPT_DIR/../../../../scripts/fetch_models.py" || true

echo ""
echo "[add_object] Done. Restart ObjectDetect2D for '$OBJECT' to appear in /vision/detections."
