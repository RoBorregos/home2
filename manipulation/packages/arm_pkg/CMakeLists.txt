cmake_minimum_required(VERSION 3.8)
project(arm_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)


#Install directories
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME})


######################### CPP #########################
# Include Cpp "include" directory
include_directories(include)

#Create Cpp executable
add_executable(gpd_test src/gpd_test.cpp)
ament_target_dependencies(gpd_test std_srvs rclcpp pcl_conversions pcl_msgs OpenCV)


#Install Cpp executables
install(TARGETS
 gpd_test
 DESTINATION lib/${PROJECT_NAME}
)





######################### Python  #########################
#Python compatability
ament_python_install_package(${PROJECT_NAME})

#Install python scripts
#install(PROGRAMS
#  scripts/example_pkg_node.py
#  DESTINATION lib/${PROJECT_NAME}
#)




###################### GPD #################################

find_library(GPD_LIB NAMES gpd PATHS $ENV{GPD_INSTALL_DIR}/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
target_link_libraries(gpd_test ${GPD_LIB})

if (NOT GPD_LIB)
  message("ENV Variable: $ENV{GPD_INSTALL_DIR}")
  message(FATAL_ERROR "Library GPD not found check ENV variable GPD_INSTALL_DIR")
else()
  message(STATUS "Library GPD found in ${GPD_LIB}")
endif()



ament_package()
