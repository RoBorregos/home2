cmake_minimum_required(VERSION 3.8)
project(depth_camera_simulator)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(frida_constants REQUIRED)

#Install directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


######################### Python  #########################
#Python compatability
ament_python_install_package(${PROJECT_NAME})

#Install python scripts
file(GLOB PYTHON_SCRIPTS depth_camera_simulator/*.py)

# Install python scripts
install(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

######################### Testing  #########################
##For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
