cmake_minimum_required(VERSION 3.8)
project(arm_pkg)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(frida_interfaces REQUIRED)
find_package(frida_constants REQUIRED)

# --------------------- GPD Configuration ---------------------
# Use find_library as in your original CMake
find_library(GPD_LIB NAMES gpd PATHS "$ENV{GPD_INSTALL_DIR}/lib" PATH_SUFFIXES lib NO_DEFAULT_PATH)

# Check if GPD was found
if (NOT GPD_LIB)
  message("ENV Variable: $ENV{GPD_INSTALL_DIR}")
  message(FATAL_ERROR "Library GPD not found. Check ENV variable GPD_INSTALL_DIR.")
else()
  message(STATUS "Library GPD found in ${GPD_LIB}")
endif()

# Include directories
include_directories(include
)

# Create executable
add_executable(gpd_test src/gpd_test.cpp)

ament_target_dependencies(gpd_test
                          frida_interfaces
                          std_srvs 
                          rclcpp 
                          pcl_conversions 
                          pcl_msgs 
                          OpenCV
                          )

# Link libraries
target_link_libraries(gpd_test
  ${OpenCV_LIBS}
  ${GPD_LIB}
  Eigen3::Eigen
)

# Install targets
install(TARGETS gpd_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# Python components
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/test_env.py scripts/grasp_markers.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()