cmake_minimum_required(VERSION 3.8)
project(mujoco_spawn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(xarm_msgs REQUIRED)  # Correct package name
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_srvs REQUIRED)

# Executable
add_executable(Xarm_gripper_mujoco_bridge
  src/xarm_gripper_mujoco_bridge.cpp
)

ament_target_dependencies(Xarm_gripper_mujoco_bridge
  rclcpp
  rclcpp_lifecycle
  xarm_msgs
  controller_manager
  hardware_interface
  std_srvs
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install (TARGETS 
  Xarm_gripper_mujoco_bridge
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_package()
