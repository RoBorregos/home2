cmake_minimum_required(VERSION 3.8)
project(arm_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python components
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/graspnet_service.py
  scripts/test_env.py
  scripts/grasp_markers.py
  examples/grasp_detection_example.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})

ament_package()