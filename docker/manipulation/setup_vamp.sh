#!/bin/bash
INITIAL_FOLDER="$(pwd)"
WORKSPACE_DIR="/workspace/src"
MANIPULATION_PKG="$WORKSPACE_DIR/manipulation/packages"

# Submodules paths
FOAM_DIR="$MANIPULATION_PKG/foam"
VAMP_DIR="$MANIPULATION_PKG/vamp"

# Compile and install FOAM and VAMP
if [ -d "$FOAM_DIR" ]; then
    if [ ! -f "$FOAM_DIR/build/external/manifold" ]; then
        echo "FOAM/Manifold not found. Compiling..."
        mkdir -p "$FOAM_DIR/build"
        cd "$FOAM_DIR/build" && cmake .. && make -j$(nproc)
    else
        echo "FOAM is already compiled."
    fi
else
    echo "Error: Submodule FOAM not found in $FOAM_DIR"
fi

# 2. Compile and install VAMP
if [ -d "$VAMP_DIR" ]; then
    if [ ! -d "$VAMP_DIR/build" ] || [ -z "$(ls -A $VAMP_DIR/build)" ]; then
        echo "VAMP not compiled. Compiling and installing..."
        mkdir -p "$VAMP_DIR/build"
        cd "$VAMP_DIR/build" && cmake .. && make -j$(nproc) && sudo make install
    else
        echo "VAMP is already compiled."
    fi
else
    echo "Error: Submodule VAMP not found in $VAMP_DIR"
fi

cd "$INITIAL_FOLDER"