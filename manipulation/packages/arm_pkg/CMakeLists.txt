cmake_minimum_required(VERSION 3.8)
project(arm_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(frida_interfaces REQUIRED)
find_package(frida_constants REQUIRED)

#Install directories
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})


######################### CPP #########################
# Include Cpp "include" directory
include_directories(include)

# file(GLOB_RECURSE srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.srv")

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${srv_files}

#   DEPENDENCIES std_msgs unique_identifier_msgs action_msgs geometry_msgs sensor_msgs std_srvs
# )
#Create Cpp executable
add_executable(gpd_test src/gpd_test.cpp)


ament_target_dependencies(gpd_test
                          frida_interfaces
                          std_srvs 
                          rclcpp 
                          pcl_conversions 
                          pcl_msgs 
                          OpenCV
                          )

# target_link_libraries(gpd_test ${PROJECT_NAME} ${OpenCV_LIBS} ...)
target_link_libraries(gpd_test ${OpenCV_LIBS} ${GPD_LIB})



#Install Cpp executables
install(TARGETS
gpd_test
DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


######################### Python  #########################
#Python compatability
ament_python_install_package(${PROJECT_NAME})

# Install python scripts
install(PROGRAMS
 scripts/test_env.py
 DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)


###################### GPD #################################

find_library(GPD_LIB NAMES gpd PATHS $ENV{GPD_INSTALL_DIR}/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
# target_link_libraries(gpd_test ${GPD_LIB})

if (NOT GPD_LIB)
  message("ENV Variable: $ENV{GPD_INSTALL_DIR}")
  message(FATAL_ERROR "Library GPD not found check ENV variable GPD_INSTALL_DIR")
else()
  message(STATUS "Library GPD found in ${GPD_LIB}")
endif()


# ament_export_dependencies(rosidl_default_runtime)
ament_package()
