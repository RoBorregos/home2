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
sudo rosdep init
rosdep update
if [[ -f /etc/nv_tegra_release ]]; then
    echo "Using rosdep jetson"
    rosdep install --from-paths . --ignore-src --skip-keys "gpd gazebo_ros gazebo_plugins gazebo_ros2_control" -y
else
    rosdep install --from-paths . --ignore-src --skip-keys gpd -y 
fi
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