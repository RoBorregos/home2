cmake_minimum_required(VERSION 3.8)
project(object_detector_2d)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(frida_interfaces REQUIRED)

#Install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

######################### CPP #########################
# Include Cpp "include" directory
include_directories(include)

######################### Message Generation #########################
# Generate message and service interfaces
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/ObjectDetection.msg"
#   "msg/ObjectDetectionArray.msg"
#   DEPENDENCIES std_msgs geometry_msgs sensor_msgs shape_msgs tf2_geometry_msgs # Add any dependencies required by your message files
# )

######################### Python  #########################
#Python compatability
ament_python_install_package(${PROJECT_NAME})


#file (GLOB NODES scripts/*.py)
#Install python scripts
install(PROGRAMS
  scripts/Detection2D.py
  scripts/DetectionPicker.py
  scripts/object_detector_node.py
  # scripts/YoloV5ObjectDetector.py
  # scripts/ObjectDetector.py
  scripts/test/local_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

######################### Testing  #########################
##For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()