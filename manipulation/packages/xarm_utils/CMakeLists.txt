cmake_minimum_required(VERSION 3.8)
project(xarm_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(xarm_utils)

# Install examples
set(SCRIPTS_DIR xarm_utils)
  
install(PROGRAMS
    ${SCRIPTS_DIR}/ir_gripper.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
