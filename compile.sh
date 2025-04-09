#!/bin/bash

echo "Select an option:"
echo "1. Prebuild Setup"
echo "2. Dependency Install"
echo "3. Compile"
echo "4. Exit"

read -p "Enter your choice (1-4): " choice

case $choice in
  1)
    echo "Running setup..."
    ./prebuld.sh
    ;;
  2)
    echo "Installing dependencies..."
    sudo rosdep init
    rosdep update
    if [[ -f /etc/nv_tegra_release ]]; then
        echo "Rosdep for jetson"
        rosdep install --from-paths . --ignore-src --skip-keys "gpd opencv message_package eigen3 pcl-ros gazebo_ros gazebo_plugins libpcl-dev gazebo_ros2_control" -y
    else
        rosdep install --from-paths . --ignore-src --skip-keys gpd -y
    fi
    
    ;;
  3)
    echo "Compiling..."
    if [[ -f /etc/nv_tegra_release ]]; then
        echo "Compiling for jetson"
        colcon build --packages-ignore realsense_gazebo_plugin xarm_gazebo
    else
        colcon build
    fi
    
    ;;
  4)
    echo "Exiting..."
    exit 0
    ;;
  *)
    echo "Invalid choice. Please select a number between 1 and 4."
    ;;
esac
