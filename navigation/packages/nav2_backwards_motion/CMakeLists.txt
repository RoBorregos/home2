cmake_minimum_required(VERSION 3.5)
project(nav2_backwards_motion)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)  # Added missing dependency

# Include directories
include_directories(
  include
)

# Create plugin library
add_library(nav2_backwards_motion_bt_node SHARED
  src/backwards_motion.cpp
)

ament_target_dependencies(nav2_backwards_motion_bt_node
  rclcpp
  rclcpp_components
  nav2_behavior_tree
  behaviortree_cpp_v3
  geometry_msgs
  pluginlib  # Added pluginlib dependency
)

# Plugin export
pluginlib_export_plugin_description_file(nav2_behavior_tree behavior_tree_plugins.xml)

# Install plugin
install(TARGETS nav2_backwards_motion_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install includes
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(nav2_backwards_motion_bt_node)
ament_export_dependencies(
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  geometry_msgs
  pluginlib  # Added pluginlib dependency
)

ament_package()