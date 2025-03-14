cmake_minimum_required(VERSION 3.8)
project(frida_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

file(GLOB_RECURSE action_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.action")
file(GLOB_RECURSE srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.srv")
file(GLOB_RECURSE msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.msg")

# Specify the .srv and .msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES std_msgs unique_identifier_msgs action_msgs geometry_msgs sensor_msgs shape_msgs
)

# Export dependencies for other packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
