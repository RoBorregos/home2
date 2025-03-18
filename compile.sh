#!/bin/bash

if [[ -f /etc/nv_tegra_release ]]; then
    echo "Compiling for jetson"
    colcon build --packages-ignore realsense_gazebo_plugin xarm_gazebo
else
    colcon build
fi