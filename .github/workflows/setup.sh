#!/bin/bash

sudo apt update
DEBIAN_FRONTEND=noninteractive 

rosdep init
rosdep update

#install vcs-tools
DEBIAN_FRONTEND=noninteractive 

#rosdep install
DEBIAN_FRONTEND=noninteractive 
rosdep install --from-paths . -y --ignore-src -r --skip-keys gpd

#Running prebuild
bash ./prebuild.sh -y

# - Importing all dependencies
colcon build --symlink-install