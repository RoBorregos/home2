cmake_minimum_required(VERSION 3.8)
project(dashgo_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(robot_localization REQUIRED)

#Install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})


#Python compatability
ament_python_install_package(${PROJECT_NAME})

#Install python scripts
install(PROGRAMS
  scripts/dashgo_driver2.py
  DESTINATION lib/${PROJECT_NAME}
)

##For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
