cmake_minimum_required(VERSION 3.8)
project(map_context)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rviz2 REQUIRED)

#Directory install
install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME})

# #CPP node config
# add_executable(ignore_laser src/ignore_laser.cpp)
# ament_target_dependencies(ignore_laser rclcpp sensor_msgs)

#Added nodes
# install(TARGETS
#   ignore_laser
#   DESTINATION lib/${PROJECT_NAME})
#   ament_python_install_package(${PROJECT_NAME})



file(GLOB PYTHON_SCRIPTS scripts/*.py)
foreach(script ${PYTHON_SCRIPTS})
  execute_process(COMMAND chmod +x ${script})
endforeach()
#Install python scripts
  install(PROGRAMS
    ${PYTHON_SCRIPTS}
    DESTINATION lib/${PROJECT_NAME}
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()


ament_package()
