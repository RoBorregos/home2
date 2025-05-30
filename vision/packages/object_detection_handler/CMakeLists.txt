cmake_minimum_required(VERSION 3.8)
project(object_detection_handler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(frida_interfaces REQUIRED)
find_package(frida_constants REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

add_executable(object_detection_handler src/object_detection_handler.cpp)

ament_target_dependencies(object_detection_handler 
  frida_interfaces 
  frida_constants
  rclcpp
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS
  object_detection_handler
  DESTINATION lib/${PROJECT_NAME})

ament_package()
