
INITIAL_FOLDER="$(pwd)"
WORKSPACE_DIR="/workspace/src"
GIGA_DIR="$WORKSPACE_DIR/manipulation/packages/giga"
MODELS_DIR="$GIGA_DIR/data/models"

# Check if the GIGA submodule is checked out
if [ ! -d "$GIGA_DIR" ] || [ -z "$(ls -A "$GIGA_DIR" 2>/dev/null)" ]; then
    echo "GIGA submodule not found, initializing ..."
    cd "$WORKSPACE_DIR" && git submodule update --init --recursive -- manipulation/packages/giga
    echo "GIGA submodule initialized, continuing ..."
fi

# Check if the vgn python package (GIGA's actual code) is importable
if python3 -c "import vgn" >/dev/null 2>&1; then
    echo "GIGA (vgn) python package already installed, continuing ..."
else
    echo "Installing GIGA (vgn) python package ..."
    pip3 install catkin_pkg
    grep -v '^torch==' "$GIGA_DIR/requirements.txt" > /tmp/giga_requirements_no_torch.txt
    pip3 install -r /tmp/giga_requirements_no_torch.txt
    rm -f /tmp/giga_requirements_no_torch.txt
    pip3 install -e "$GIGA_DIR"

    echo "Building ConvONets compiled extensions ..."
    (cd "$GIGA_DIR" && python3 scripts/convonet_setup.py build_ext --inplace)

    cat <<'EOF'

EOF
fi

if [ ! -d "$MODELS_DIR" ] || [ -z "$(ls -A "$MODELS_DIR" 2>/dev/null)" ]; then
    cat <<EOF

EOF
else
    echo "GIGA pretrained models found in $MODELS_DIR, continuing ..."
fi

cd "$INITIAL_FOLDER"
