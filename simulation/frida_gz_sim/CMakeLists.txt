cmake_minimum_required(VERSION 3.8)
project(frida_gz_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

file(GLOB PYTHON_SCRIPTS CONFIGURE_DEPENDS scripts/*.py)
install(PROGRAMS ${PYTHON_SCRIPTS} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch urdf worlds models
  DESTINATION share/${PROJECT_NAME})

ament_package()
