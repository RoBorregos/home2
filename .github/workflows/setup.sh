#!/bin/bash

DEBIAN_FRONTEND=noninteractive
SCRIPT_DIR=$(realpath "$(dirname "$0")/../..")
CURRENT_DIR="$(pwd)"

echo "LS MANIPULATION"
ls ./manipulation/packages

echo "APT UPDATE"
sudo apt update

# nanobind's stubgen.py (invoked by vamp's CMakeLists during build) needs
# typing.get_type_hints(include_extras=True) AND typing_extensions.TypeVarTuple.
# include_extras only exists on Python 3.11+; TypeVarTuple was added to
# typing_extensions 4.1.0 (Jan 2022). Ubuntu 22.04's apt package
# python3-typing-extensions ships 3.10.0.2-1 (no TypeVarTuple) so we install
# via pip instead. Without this the vamp build aborts at the __init__.pyi
# generation step on the ros:humble CI image.
echo "INSTALL PYTHON BUILD DEPS (modern typing_extensions for vamp/nanobind stubgen)"
sudo apt install -y python3-pip
sudo pip install --upgrade 'typing_extensions>=4.6'


echo "ROSDEP INIT"
rosdep init

echo "ROSDEP UPDATE"
rosdep update

#For testing
echo "ROSDEP INSTALL"
rosdep install --from-paths . -y --ignore-src -r --skip-keys gpd

#Running prebuild
echo "GPD BUILD"

LIB_DIR="$(pwd)/install/gpd"
echo "The path is $LIB_DIR"
cd $SCRIPT_DIR

#Checking GPD 
GPD_DIR=$SCRIPT_DIR/manipulation/packages/gpd

if [ ! -d "$GPD_DIR" ]; then
    echo "Error: GPD does not exist"
    git submodule update --init --recursive
    
fi

# Preparing GPD
if grep -q '^export GPD_INSTALL_DIR=' ~/.bashrc; then
    # Update existing entry
    sed -i 's|^export GPD_INSTALL_DIR=.*|export GPD_INSTALL_DIR='"$LIB_DIR"'|' ~/.bashrc
    echo "Updated GPD_INSTALL_DIR in ~/.bashrc to: $LIB_DIR"
else 
    # Add the entry if it does not exist
    echo "export GPD_INSTALL_DIR=$LIB_DIR" >> ~/.bashrc
    echo "Added GPD_INSTALL_DIR to ~/.bashrc: $LIB_DIR"
fi


# Set GPD_INSTALL_DIR for the current session
export GPD_INSTALL_DIR=$LIB_DIR

# Always echo the value
echo "GPD_INSTALL_DIR is set to: $GPD_INSTALL_DIR"

cd $GPD_DIR
mkdir build

CFG_DIR="$GPD_DIR/cfg/eigen_params.cfg"

if [ ! -f "$CFG_DIR" ]; then
    echo "Error: File '$CFG_DIR' does not exist."
    exit 1
fi

sed -i "2s|.*|hand_geometry_filename = $GPD_DIR/cfg/hand_geometry.cfg|" $CFG_DIR
sed -i "5s|.*|image_geometry_filename = $GPD_DIR/cfg/image_geometry_15channels.cfg|" $CFG_DIR
sed -i "8s|.*|weights_file = $GPD_DIR/models/lenet/15channels/params/|" $CFG_DIR

# build gpd
cd build
cmake ..
sudo make install
