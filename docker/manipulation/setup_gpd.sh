#!/bin/bash
INITIAL_FOLDER="$(pwd)"
WORKSPACE_DIR="/workspace/src"
GPD_DIR="$WORKSPACE_DIR/manipulation/packages/gpd"
BUILD_DIR="$GPD_DIR/build/"
LIB_DIR="/usr/local/include/gpd"

# Check if GPD is in home2 workspace
if [ -z "$GPD_DIR" ]; then
    echo "GPD not found downloading ... "
    cd $WORKSPACE_DIR && git submodule update --init --recursive
    echo "GPD downloaded continuing ... "  
fi
# Check if GPD is built in home2
if [ -d "$BUILD_DIR" ]; then
    if [ "$(ls -A "$BUILD_DIR")" ] && [ -d "$LIB_DIR" ] ; then
        echo "GPD build found continuing ... "
    else
        echo "GPD build folder found but not compiled"
        echo "Compiling ...."
        cd $BUILD_DIR && cmake .. && make && sudo make install
        echo "Compiling completed continuing ..."
    fi
else
    echo "GPD build folder not found, creating and compiling ..."
    echo "Creating ...."
    cd $GPD_DIR && mkdir -p build
    echo "Compiling ...."
    cd $BUILD_DIR && cmake .. && make && sudo make install
    echo "Compiling completed continuing ..."
fi

cd "$INITIAL_FOLDER" && clear