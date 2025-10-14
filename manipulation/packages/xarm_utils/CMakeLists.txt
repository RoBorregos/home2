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

# Verify script exists before installing
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/xarm_utils/ir_gripper.py")
  install(PROGRAMS
      xarm_utils/ir_gripper.py
      DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "ir_gripper.py not found")
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()