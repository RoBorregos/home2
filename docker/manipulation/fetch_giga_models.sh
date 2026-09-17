#!/usr/bin/env bash
# Fetch the GIGA authors' pretrained checkpoints into data/models.
#
# Optional: only needed to evaluate against the paper's models. Training
# FRIDA's own detector from scratch never reads them. The archive is the
# "data" bundle linked from GIGA's README (packed/pile datasets *and*
# data/models); only data/models is kept here -- the datasets in it are for
# the paper's YCB object sets, not FRIDA's.
set -euo pipefail

GIGA_DIR="${GIGA_DIR:-/workspace/src/manipulation/packages/giga}"
URL="https://utexas.box.com/shared/static/h3ferwjhuzy6ja8bzcm3nu9xq1wkn94s.zip"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Downloading pretrained models ..."
wget --no-check-certificate -O "$TMP/data.zip" "$URL"
echo "Extracting data/models ..."
unzip -q -o "$TMP/data.zip" 'data/models/*' -d "$TMP"
mkdir -p "$GIGA_DIR/data/models"
cp -r "$TMP/data/models/." "$GIGA_DIR/data/models/"
echo "Done: $(ls -1 "$GIGA_DIR/data/models" | wc -l) entries in $GIGA_DIR/data/models"
