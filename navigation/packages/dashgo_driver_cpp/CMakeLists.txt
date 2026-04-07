cmake_minimum_required(VERSION 3.8)
project(dashgo_driver_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# C++ Node
add_executable(dashgo_driver_cpp
  src/dashgo_driver_cpp.cpp
)
ament_target_dependencies(dashgo_driver_cpp
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(dashgo_driver_cpp
  ${Boost_LIBRARIES}
)

# Install
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  dashgo_driver_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Automated Real-Time Priority permission (requires sudo on host or container
install(CODE "execute_process(COMMAND sudo setcap 'cap_sys_nice=eip' ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${PROJECT_NAME})")

ament_package()
