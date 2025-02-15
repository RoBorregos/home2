#!/bin/bash

# Display a warning message
cat << 'EOF'

###############################################################
#                                                             #
#                       !!! WARNING !!!                       #
#                                                             #
#    You should execute this script in the directory where    #
#    you plan to compile your project (ROS WORKSPACE). where  #
#    the directory install/ log/ build/ will be. This is      #
#    critical to avoid unexpected behavior or errors during   #
#    compilation.                                             #
#                                                             #
#    Press Ctrl+C to exit if you are not in the right path.   #
#                                                             #
###############################################################

EOF

# Pause for a few seconds to let the user read the warning
sleep 5
SCRIPT_DIR=$(dirname "$(realpath "$0")")
CURRENT_DIR="$(pwd)"

# Show the current directory
echo "You are currently in: $CURRENT_DIR"

#Check the correct execution path
if [[ "$CURRENT_DIR" == "$SCRIPT_DIR" ]]; then
    echo "Executing from the script's directory. Proceeding..."
elif [[ "$CURRENT_DIR" == "$SCRIPT_DIR"/* ]]; then
    echo "You are in a subfolder of the script's directory. Execution stopped."
    exit 1
else
    echo "Executing from outside the script's directory. Proceeding..."
fi

#Exporting source directory
if grep -q '^export SRC_HOME_PATH=' ~/.bashrc; then
    echo "SRC_HOME_PATH ALREADY ADDED TO BASHRC"
else 
    echo "export SRC_HOME_PATH=$SCRIPT_DIR" >> ~/.bashrc
fi
export SRC_HOME_PATH=$SCRIPT_DIR

LIB_DIR="$(pwd)/install/gpd"
echo "The path is $LIB_DIR"
cd $SCRIPT_DIR

# ROSDEP INSTALLING DEPENDENCIES
rosdep update
rosdep install --from-paths . --ignore-src --skip-keys gpd

#Checking GPD 
GPD_DIR=$SCRIPT_DIR/manipulation/packages/gpd

if [ ! -d "$GPD_DIR" ]; then
    echo "Error: GPD does not exist"
    git submodule update --init --recursive
    
fi

#Preparing gpd
if grep -q '^export GPD_INSTALL_DIR=' ~/.bashrc; then
    echo "GPD INSTALL PATH ALREADY ADDED TO BASHRC"
else 
    echo "export GPD_INSTALL_DIR=$LIB_DIR" >> ~/.bashrc
fi
export GPD_INSTALL_DIR=$LIB_DIR

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