cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(pick_and_place)

# Install examples
set(SCRIPTS_DIR pick_and_place)
set(TESTS_DIR ${SCRIPTS_DIR}/tests)
install(PROGRAMS
    ${SCRIPTS_DIR}/pick_server.py
    ${TESTS_DIR}/call_pick_action.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()