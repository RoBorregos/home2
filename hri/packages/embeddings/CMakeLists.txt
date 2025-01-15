cmake_minimum_required(VERSION 3.8)
project(embeddings)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(frida_interfaces REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(
  DIRECTORY scripts/
  DESTINATION ${PYTHON_INSTALL_DIR}
  FILES_MATCHING PATTERN "*.py"
)

# Install additional configuration files (launch, config, etc.)


# If you need to install non-Python assets (e.g., ONNX, JSON files), uncomment and configure as needed:
# install(
#   DIRECTORY assets/
#   DESTINATION lib/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.onnx"
# )

# Optional: Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright linter if needed
  set(ament_cmake_cpplint_FOUND TRUE)    # Skip cpplint if no C++ code
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
