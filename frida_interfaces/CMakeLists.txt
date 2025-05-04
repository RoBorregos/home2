cmake_minimum_required(VERSION 3.8)
#if(POLICY CMP0148)
#  cmake_policy(SET CMP0148 NEW)
#endif()
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
set(action_files "")
set(srv_files "")
set(msg_files "")

file(GLOB_RECURSE hri_action_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "hri/action/*.action")
file(GLOB_RECURSE hri_srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "hri/srv/*.srv")
file(GLOB_RECURSE hri_msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "hri/msg/*.msg")
list(APPEND action_files ${hri_action_files})
list(APPEND srv_files ${hri_srv_files})
list(APPEND msg_files ${hri_msg_files})

file(GLOB_RECURSE manip_action_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "manipulation/action/*.action")
file(GLOB_RECURSE manip_srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "manipulation/srv/*.srv")
file(GLOB_RECURSE manip_msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "manipulation/msg/*.msg")
list(APPEND action_files ${manip_action_files})
list(APPEND srv_files ${manip_srv_files})
list(APPEND msg_files ${manip_msg_files})

file(GLOB_RECURSE nav_action_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "navigation/action/*.action")
file(GLOB_RECURSE nav_srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "navigation/srv/*.srv")
file(GLOB_RECURSE nav_msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "navigation/msg/*.msg")
list(APPEND action_files ${nav_action_files})
list(APPEND srv_files ${nav_srv_files})
list(APPEND msg_files ${nav_msg_files})

file(GLOB_RECURSE tm_action_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "task_manager/action/*.action")
file(GLOB_RECURSE tm_srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "task_manager/srv/*.srv")
file(GLOB_RECURSE tm_msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "task_manager/msg/*.msg")
list(APPEND action_files ${tm_action_files})
list(APPEND srv_files ${tm_srv_files})
list(APPEND msg_files ${tm_msg_files})

file(GLOB_RECURSE vis_action_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "vision/action/*.action")
file(GLOB_RECURSE vis_srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "vision/srv/*.srv")
file(GLOB_RECURSE vis_msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "vision/msg/*.msg")
list(APPEND action_files ${vis_action_files})
list(APPEND srv_files ${vis_srv_files})
list(APPEND msg_files ${vis_msg_files})

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
