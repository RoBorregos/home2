cmake_minimum_required(VERSION 3.8)
project(perception_3d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(frida_interfaces REQUIRED)
find_package(frida_constants REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(tf_transformations REQUIRED)
find_package(rclpy REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)

add_executable(down_sample_pc src/down_sample_pc.cpp)
add_executable(publish_cloud src/publish_pcl.cpp)
add_executable(plane_service src/remove_plane.cpp)
add_executable(pick_primitives src/add_primitives.cpp)
add_executable(test_only_orchestrator src/masive_testin.cpp)
add_executable(read_pcd src/read_cluster.cpp)
add_executable(publish_handle src/publish_handle.cpp)

ament_target_dependencies(down_sample_pc
  frida_interfaces
  frida_constants
  rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros tf2 pcl_ros pcl_conversions tf2_eigen PCL)

ament_target_dependencies(publish_cloud
  frida_interfaces
  frida_constants
  rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros tf2 pcl_ros pcl_conversions tf2_eigen PCL)

ament_target_dependencies(plane_service
  frida_interfaces
  frida_constants
  rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros tf2 pcl_ros pcl_conversions tf2_eigen PCL)

ament_target_dependencies(pick_primitives
  frida_interfaces
  frida_constants
  rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros tf2 pcl_ros pcl_conversions tf2_eigen PCL)

ament_target_dependencies(test_only_orchestrator
  frida_interfaces
  frida_constants
  rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros tf2 pcl_ros pcl_conversions tf2_eigen PCL)

ament_target_dependencies(read_pcd
  frida_interfaces
  frida_constants
  rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros tf2 pcl_ros pcl_conversions tf2_eigen PCL)

ament_target_dependencies(publish_handle
  frida_interfaces
  frida_constants
  rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros tf2 pcl_ros pcl_conversions tf2_eigen PCL yaml_cpp_vendor)

install(TARGETS 
  plane_service

  DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  publish_cloud
  
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  down_sample_pc

  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  pick_primitives

  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  test_only_orchestrator

  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  read_pcd

  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  publish_handle
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)



install(FILES include/perception_3d/macros.hpp DESTINATION include/perception_3d)



install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


# find_package(ament_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/record_relative_pose.py
  scripts/dock_to_handle.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() 
