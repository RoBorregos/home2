#!/bin/bash
# One-command wrapper: build the gallery entry for <object_name> from
# gallery_photos/<object_name>/*.jpg AND sync it into install/, so a
# running node (which reads from install/, not source) actually picks it
# up. Without the sync step, gallery_build.py alone has zero effect on a
# live node — see README.md's "install/src sync gotcha".
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

python3 gallery_build.py --object "$OBJECT" --photos "$PHOTOS" --gallery-dir gallery/

INSTALL_DETECTORS="/workspace/install/object_detector_2d/lib/object_detector_2d/detectors"
if [ -d "$INSTALL_DETECTORS" ]; then
    echo "[add_object] syncing gallery/ -> $INSTALL_DETECTORS/gallery"
    rm -rf "$INSTALL_DETECTORS/gallery"
    cp -r gallery "$INSTALL_DETECTORS/gallery"
else
    echo "[add_object] NOTE: $INSTALL_DETECTORS not found (not running inside" \
         "the container, or a different layout) — sync gallery/ to wherever" \
         "the live node's install/ tree lives before restarting it."
fi

echo ""
echo "[add_object] Done. Restart ObjectDetect2D for '$OBJECT' to appear in /vision/detections."
