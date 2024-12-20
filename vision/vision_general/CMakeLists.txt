cmake_minimum_required(VERSION 3.8)
project(vision_general)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

#Install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})


######################### CPP #########################
# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
# add_executable(cpp_executable src/example_pkg_cpp_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)


# Install Cpp executables
# install(TARGETS
#   cpp_executable
#   DESTINATION lib/${PROJECT_NAME}
# )



######################### Python  #########################
#Python compatability
ament_python_install_package(${PROJECT_NAME})

#Install python scripts
install(PROGRAMS
  scripts/example_pkg_node.py
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
