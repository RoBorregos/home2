#!/bin/bash
# Download every .gguf (all shards) of a Hugging Face repo into hri/packages/nlp/assets for rbrgs_compare.sh.
# Private repos need HF_TOKEN in the environment. Partial downloads resume.
#
# Usage:
#   [HF_TOKEN=hf_...] ./fetch_gguf.sh [--dry-run] <user/repo> [revision]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ASSETS_DIR="$(realpath "$SCRIPT_DIR/../../packages/nlp/assets")"

DRY_RUN=false
[[ "${1:-}" == --dry-run ]] && { DRY_RUN=true; shift; }
[[ $# -ge 1 ]] || { sed -n '2,6p' "$0"; exit 1; }
REPO="$1"
REV="${2:-main}"

# Token goes through a file descriptor so it never shows up in `ps` on the shared Orin.
hf_curl() {
    if [[ -n "${HF_TOKEN:-}" ]]; then
        curl -H @<(printf 'Authorization: Bearer %s\n' "$HF_TOKEN") "$@"
    else
        curl "$@"
    fi
}

listing=$(hf_curl -sfL "https://huggingface.co/api/models/$REPO/tree/$REV?recursive=true") \
    || { echo "ERROR: cannot list $REPO@$REV (private repo? set HF_TOKEN)"; exit 1; }
# "<size> <path>" per .gguf file
files=$(python3 -I -c '
import json, sys
for f in json.load(sys.stdin):
    if f.get("type") == "file" and f["path"].endswith(".gguf"):
        print(f.get("lfs", {}).get("size", f.get("size", 0)), f["path"])
' <<< "$listing")
[[ -n "$files" ]] || { echo "ERROR: no .gguf files in $REPO@$REV"; exit 1; }

need=0
while read -r size path; do
    dest="$ASSETS_DIR/$(basename "$path")"
    have=$(stat -c %s "$dest" 2>/dev/null || stat -c %s "$dest.partial" 2>/dev/null || echo 0)
    [[ -f "$dest" && "$have" == "$size" ]] && continue
    need=$((need + size - have))
done <<< "$files"
free=$(df -B1 --output=avail "$ASSETS_DIR" | tail -1)

echo "$REPO@$REV:"
awk '{ printf "  %6.1f GB  %s\n", $1 / 1e9, $2 }' <<< "$files"
awk -v n="$need" -v f="$free" 'BEGIN { printf "Still to download: %.1f GB, free on disk: %.1f GB\n", n / 1e9, f / 1e9 }'
$DRY_RUN && exit 0
(( need < free )) || { echo "ERROR: not enough disk space"; exit 1; }

while read -r size path; do
    dest="$ASSETS_DIR/$(basename "$path")"
    if [[ -f "$dest" && "$(stat -c %s "$dest")" == "$size" ]]; then
        echo "$(basename "$dest") already downloaded"
        continue
    fi
    if [[ "$(stat -c %s "$dest.partial" 2>/dev/null)" != "$size" ]]; then
        echo "Downloading $path..."
        hf_curl --fail --location --retry 5 --continue-at - \
            -o "$dest.partial" "https://huggingface.co/$REPO/resolve/$REV/$path"
    fi
    [[ "$(stat -c %s "$dest.partial")" == "$size" ]] || { echo "ERROR: $path size mismatch"; exit 1; }
    mv "$dest.partial" "$dest"
done <<< "$files"
echo "Done. Benchmark with: ./rbrgs_compare.sh $(awk 'NR == 1 { n = split($2, p, "/"); print p[n] }' <<< "$files")"
