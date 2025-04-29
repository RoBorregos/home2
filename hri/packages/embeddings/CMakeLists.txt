cmake_minimum_required(VERSION 3.8)
project(embeddings)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(frida_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY scripts/
  DESTINATION ${PYTHON_INSTALL_DIR}/embeddings
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY launch config
 DESTINATION share/${PROJECT_NAME})
file(GLOB PYTHON_SCRIPTS scripts/*.py)


install(
  PROGRAMS
    scripts/categorization.py  # Path to your Python script
    scripts/knowledge_base.py
  DESTINATION lib/${PROJECT_NAME}  # Install to the package's lib directory
)
# Install csv files
install(DIRECTORY dataframes
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.csv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
