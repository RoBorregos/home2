#!/bin/bash

AUTO_ACCEPT=false
if [[ "$1" == "-y" ]]; then
    AUTO_ACCEPT=true
fi

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

# Show the current directory
echo "You are currently in: $(pwd)"

# Confirm with the user if not auto-accepted
if [[ "$AUTO_ACCEPT" == false ]]; then
    read -p "Are you sure you want to proceed? (yes/no): " choice
    if [[ "$choice" != "yes" ]]; then
        echo "Aborting script. Please navigate to the correct directory and try again."
        exit 1
    fi
else
    echo "Auto-accept flag (-y) detected. Proceeding without confirmation."
fi

echo "Proceeding with the script..."

LIB_DIR="$(pwd)/install/gpd"
echo "The path is $LIB_DIR"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd $SCRIPT_DIR

# ROSDEP INSTALLING DEPENDENCIES
rosdep update
rosdep install --from-paths . --ignore-src --skip-keys gpd

#Checking GPD 
GPD_DIR=$SCRIPT_DIR/manipulacion/packages/gpd

if [ ! -d "$GPD_DIR" ]; then
    echo "Error: gdp does not exist"
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