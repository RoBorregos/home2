cmake_minimum_required(VERSION 3.8)
project(vamp_moveit_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VampPlan.srv"
)

add_library(vamp_moveit_plugin_lib
  src/vamp_planner_manager.cpp
  src/vamp_planning_context.cpp
)

target_compile_features(vamp_moveit_plugin_lib PUBLIC cxx_std_17)

target_include_directories(vamp_moveit_plugin_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(vamp_moveit_plugin_lib
  rclcpp
  moveit_core
  moveit_ros_planning
  geometric_shapes
  tf2_eigen
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(vamp_moveit_plugin_lib 
  ${OCTOMAP_LIBRARIES}
  ${cpp_typesupport_target}
)

pluginlib_export_plugin_description_file(moveit_core vamp_moveit_plugin.xml)

install(TARGETS vamp_moveit_plugin_lib
  EXPORT export_vamp_moveit_plugin_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_vamp_moveit_plugin_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp 
  moveit_core 
  moveit_ros_planning 
  geometric_shapes 
  tf2_eigen 
  octomap
)

ament_package()