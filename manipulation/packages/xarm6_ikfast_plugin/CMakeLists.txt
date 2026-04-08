cmake_minimum_required(VERSION 3.8)
project(xarm6_ikfast_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable -Wno-unused-parameter -Wno-sign-compare)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(LAPACK REQUIRED)

add_library(xarm6_ikfast_plugin SHARED
  src/xarm6_ikfast_moveit_plugin.cpp
)

target_include_directories(xarm6_ikfast_plugin PRIVATE
  src
)

target_compile_definitions(xarm6_ikfast_plugin PRIVATE
  IKFAST_NO_MAIN
  IKFAST_HAS_LIBRARY
)

ament_target_dependencies(xarm6_ikfast_plugin
  moveit_core
  pluginlib
  rclcpp
  tf2_kdl
  tf2_geometry_msgs
)

target_link_libraries(xarm6_ikfast_plugin
  ${LAPACK_LIBRARIES}
)

pluginlib_export_plugin_description_file(moveit_core ikfast_plugin_description.xml)

install(TARGETS xarm6_ikfast_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
