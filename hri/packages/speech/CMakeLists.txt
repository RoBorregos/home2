cmake_minimum_required(VERSION 3.8)
project(speech)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(frida_interfaces REQUIRED)

install(DIRECTORY launch config
 DESTINATION share/${PROJECT_NAME})

# Add Cpp executable
# add_executable(ignore_laser src/ignore_laser.cpp)
# ament_target_dependencies(audio_common_msgs)

# Install Cpp executables
# install(TARGETS
#  ignore_laser
#  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

# Install the Python module
install(
  DIRECTORY scripts/
  DESTINATION ${PYTHON_INSTALL_DIR}/speech
  FILES_MATCHING PATTERN "*.py"
)

# Install ONNX and WAV files
install(DIRECTORY assets
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING
  PATTERN "*.onnx"
  PATTERN "*.wav"
)

# Install JSON files
# install(
#   DIRECTORY assets/
#   DESTINATION lib/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.json"
# )

file(GLOB PYTHON_SCRIPTS scripts/*.py)

# Install python scripts
install(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/stt
  DIRECTORY scripts/tts
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
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
