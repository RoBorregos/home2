#!/bin/bash
# Downloads the speaker-verification model (issue #1459) into the gitignored
# speech assets folder. Safe to re-run: skips files that are already present.
#
# WeSpeaker ResNet34-LM (VoxCeleb), CC-BY-4.0:
#   https://huggingface.co/Wespeaker/wespeaker-voxceleb-resnet34-LM
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="${SCRIPT_DIR}/../../../hri/packages/speech/assets/downloads/speaker_id"
mkdir -p "${DEST}/profiles"

download() {
  local url="$1" out="$2" size="$3"
  if [ -f "$out" ] && [ "$(stat -c %s "$out" 2>/dev/null || stat -f %z "$out")" = "$size" ]; then
    echo "OK   $(basename "$out") already present"
    return
  fi
  echo "GET  $(basename "$out")"
  curl -fL --retry 3 -o "${out}.part" "$url"
  mv "${out}.part" "$out"
}

download \
  "https://huggingface.co/Wespeaker/wespeaker-voxceleb-resnet34-LM/resolve/main/voxceleb_resnet34_LM.onnx" \
  "${DEST}/voxceleb_resnet34_LM.onnx" \
  26530309

echo "Speaker models in ${DEST}"
