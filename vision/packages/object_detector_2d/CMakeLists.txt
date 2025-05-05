cmake_minimum_required(VERSION 3.8)
project(object_detector_2d)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(frida_interfaces REQUIRED)
find_package(frida_constants REQUIRED)

#Install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

######################### CPP #########################
# Include Cpp "include" directory
include_directories(include)

######################### Message Generation #########################
# Generate message and service interfaces
# )

######################### Python  #########################
#Python compatability
ament_python_install_package(${PROJECT_NAME})


file (GLOB PYTHON_SCRIPTS scripts/*.py)

install(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

file (GLOB SCRIPTS_DETECTORS scripts/detectors/*.py)

install(PROGRAMS
  ${SCRIPTS_DETECTORS}
  DESTINATION lib/${PROJECT_NAME}/detectors
)

file (GLOB SCRIPTS_INCLUDE scripts/include/*.py)

install(PROGRAMS
  ${SCRIPTS_INCLUDE}
  DESTINATION lib/${PROJECT_NAME}/include
)

file (GLOB MODELS models/*.pt)
install(PROGRAMS
  ${MODELS}
  DESTINATION lib/${PROJECT_NAME}/models
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
