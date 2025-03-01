cmake_minimum_required(VERSION 3.8)
project(nav_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(frida_interfaces REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# instalar el json
install(DIRECTORY locations
  DESTINATION share/${PROJECT_NAME})

add_executable(ignore_laser src/ignore_laser.cpp)
ament_target_dependencies(ignore_laser rclcpp sensor_msgs)

install(TARGETS
  ignore_laser
  DESTINATION lib/${PROJECT_NAME})

  
  ament_python_install_package(${PROJECT_NAME})

  file(GLOB PYTHON_SCRIPTS scripts/*.py)

  #Install python scripts
  install(PROGRAMS
    ${PYTHON_SCRIPTS}
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
