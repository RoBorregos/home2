cmake_minimum_required(VERSION 3.8)
project(frida_constants)

# Find dependencies
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})


ament_package()
