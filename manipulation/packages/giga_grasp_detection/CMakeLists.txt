cmake_minimum_required(VERSION 3.8)
project(giga_grasp_detection)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(frida_interfaces REQUIRED)
find_package(frida_constants REQUIRED)

# Python components
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/giga_grasp_detection_service.py DESTINATION lib/${PROJECT_NAME})

ament_package()
