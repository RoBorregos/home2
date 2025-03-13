cmake_minimum_required(VERSION 3.8)
project(frida_constants)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(DIRECTORY map_areas
  DESTINATION share/${PROJECT_NAME})
  
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

file(GLOB_RECURSE headers RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.hpp")

# install(FILES include/frida_constants/manip_3d.hpp DESTINATION include/frida_constants)

install(FILES ${headers} DESTINATION include/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

# Export include directories
ament_export_include_directories(include)

ament_package()
