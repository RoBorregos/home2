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
#Install python scripts
file(GLOB PYTHON_SCRIPTS pick_and_place/*.py)
file(GLOB TEST_SCRIPTS pick_and_place/tests/*.py)

# Install python scripts
install(PROGRAMS
  ${PYTHON_SCRIPTS}
  ${TEST_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()