cmake_minimum_required(VERSION 3.5)
project(pouring_services)

# Default to C++14 (not needed for Python, but kept for compatibility)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies (Python-specific)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Python client library for ROS 2
find_package(std_msgs REQUIRED)  # Standard ROS 2 messages

# Install Python scripts
install(PROGRAMS
  scripts/pouring_srv.py  # Path to your Python script
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies (Python-specific)
ament_export_dependencies(rclpy std_msgs)

# Finalize the package
ament_package()