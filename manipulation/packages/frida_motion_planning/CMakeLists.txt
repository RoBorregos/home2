cmake_minimum_required(VERSION 3.8)
project(frida_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(frida_motion_planning)

# Install examples
set(EXAMPLES_DIR examples)
set(SCRIPTS_DIR frida_motion_planning)
    

file(GLOB_RECURSE example_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${EXAMPLES_DIR}/*.py")

install(PROGRAMS
    ${example_files}
    ${EXAMPLES_DIR}/ds4_demo.py
    ${SCRIPTS_DIR}/motion_planning_server.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
