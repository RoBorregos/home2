#!/bin/bash
# Downloads the textured object meshes used by the sim object models (Gazebo Fuel, YCB / Google Scanned Objects).
set -e
DEST=${1:-$HOME/.gz/sim_models}
mkdir -p "$DEST"

# Fuel OBJs share the material name "material_0", which Gazebo caches across meshes
fix_materials() {
  local dir=$1
  find "$DEST/$dir" \( -name "*.obj" -o -name "*.mtl" \) -exec sed -i "s/material_0/${dir}_material/g" {} +
  if [ -f "$DEST/$dir/materials/textures/texture.png" ] && [ -d "$DEST/$dir/meshes" ]; then
    [ -f "$DEST/$dir/meshes/texture.png" ] || cp "$DEST/$dir/materials/textures/texture.png" "$DEST/$dir/meshes/texture.png"
  fi
}

fetch() {
  local dir=$1 owner=$2 name=$3
  if [ -f "$DEST/$dir/model.config" ]; then
    fix_materials "$dir"
    return
  fi
  local enc
  enc=$(python3 -c "import urllib.parse,sys;print(urllib.parse.quote(sys.argv[1]))" "$name")
  echo "Downloading $owner/$name -> $DEST/$dir"
  curl -fsSL -o "/tmp/$dir.zip" "https://fuel.gazebosim.org/1.0/$owner/models/$enc/tip/$enc.zip"
  mkdir -p "$DEST/$dir"
  python3 -c "import zipfile,sys;zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "/tmp/$dir.zip" "$DEST/$dir"
  rm -f "/tmp/$dir.zip"
  fix_materials "$dir"
}

fetch fuel_mustard_bottle Gambit "Mustard Bottle"
fetch fuel_mug GoogleResearch "Threshold_Porcelain_Coffee_Mug_All_Over_Bead_White"
fetch fuel_medicine_bottle GoogleResearch "Phillips_Milk_of_Magnesia_Saline_Laxative_Liquid_Original"
fetch fuel_bowl GoogleResearch "Room_Essentials_Bowl_Turquiose"
echo "Object meshes ready in $DEST"
