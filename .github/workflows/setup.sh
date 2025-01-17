#!/bin/bash

echo "APT UPDATE" 
sudo apt update
DEBIAN_FRONTEND=noninteractive
 
echo "ROSDEP INIT" 
rosdep init

echo "ROSDEP UPDATE"
rosdep update

#install vcs-tools
DEBIAN_FRONTEND=noninteractive 

#rosdep install
DEBIAN_FRONTEND=noninteractive 

#For testing 
echo "LS MANIPULATION"
ls ./manipulation/package/

echo "ROSDEP INSTALL"
rosdep install --from-paths . -y --ignore-src -r --skip-keys gpd

#Running prebuild
echo "PREBUILD SCRIPT"
bash ./prebuild.sh -y
