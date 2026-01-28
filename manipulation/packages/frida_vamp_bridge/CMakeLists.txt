cmake_minimum_required(VERSION 3.8)
project(frida_vamp_bridge)

add_compile_options(-std=c++20)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -march=native -Wno-ignored-attributes)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-O3 -march=native -Wno-ignored-attributes)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(vamp REQUIRED)

add_executable(frida_vamp_bridge_node src/frida_vamp_bridge_node.cpp)


target_include_directories(frida_vamp_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /home/dominguez/roborregos/home_ws/src/manipulation/packages/vamp/src/impl
  /home/dominguez/roborregos/home_ws/src/manipulation/packages/vamp/build/_deps/pdqsort-src
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(frida_vamp_bridge_node Eigen3::Eigen)

ament_target_dependencies(frida_vamp_bridge_node
  rclcpp
  sensor_msgs
  # vamp
)


install(TARGETS
  frida_vamp_bridge_node
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
