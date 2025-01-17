#!/bin/bash

DEBIAN_FRONTEND=noninteractive

echo "LS MANIPULATION"
ls ./manipulation/packages

echo "APT UPDATE" 
sudo apt update

 
echo "ROSDEP INIT" 
rosdep init

echo "ROSDEP UPDATE"
rosdep update

#For testing
echo "ROSDEP INSTALL"
rosdep install --from-paths . -y --ignore-src -r --skip-keys gpd

#Running prebuild
echo "PREBUILD SCRIPT"
./prebuild.sh -y
