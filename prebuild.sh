#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd $SCRIPT_DIR

# ROSDEP INSTALLING DEPENDENCIES
rosdep update
rosdep install --from-paths . --ignore-src --skip-keys gpd

#Checking GPD 
GPD_DIR=$SCRIPT_DIR/manipulacion/packages/gpd
if [ ! -d "$DIR_PATH" ]; then
    echo "Error: gdp does not exist"
    git submodule update --init --recursive

fi

CFG_DIR="$SCRIPT_DIR/eigen_params.cfg"

if [ ! -f "$CFG_DIR" ]; then
    echo "Error: File '$CFG_DIR' does not exist."
    exit 1
fi

echo " the path is '$SCRIPT_DIR' "