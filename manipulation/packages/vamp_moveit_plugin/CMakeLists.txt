cmake_minimum_required(VERSION 3.8)
project(vamp_moveit_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(octomap REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VampPlan.srv"
  DEPENDENCIES geometry_msgs trajectory_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_lib SHARED
  src/scene_conversion.cpp
  src/vamp_client.cpp
  src/vamp_planner_manager.cpp
  src/vamp_planning_context.cpp
)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lib
  ${cpp_typesupport_target}
  ${OCTOMAP_LIBRARIES}
  geometric_shapes::geometric_shapes
  moveit_core::moveit_constraint_samplers
  moveit_core::moveit_kinematic_constraints
  moveit_core::moveit_planning_interface
  moveit_core::moveit_planning_scene
  moveit_core::moveit_robot_state
  moveit_core::moveit_robot_trajectory
  moveit_core::moveit_trajectory_processing
  moveit_core::moveit_utils
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2_eigen::tf2_eigen
)

pluginlib_export_plugin_description_file(moveit_core vamp_moveit_plugin.xml)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS
  scripts/benchmark_vamp.py
  scripts/check_collision_model.py
  scripts/export_robot_model.py
  scripts/vamp_server.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
