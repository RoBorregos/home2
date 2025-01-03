#!/bin/bash

sudo apt update
DEBIAN_FRONTEND=noninteractive 

rosdep init
rosdep update

#install vcs-tools
DEBIAN_FRONTEND=noninteractive 

#rosdep install
DEBIAN_FRONTEND=noninteractive 
rosdep install --from-paths . -y --ignore-src -r

# - Importing all dependencies
colcon build --symlink-install