cmake_minimum_required(VERSION 3.8)
project(vision_general)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(frida_interfaces REQUIRED)
find_package(frida_constants REQUIRED)

#Install directories
install(DIRECTORY launch scripts/utils config
  DESTINATION share/${PROJECT_NAME})


######################### Semantic-Map  #########################
# Its modules have generic names (core, ports, adapters), so they go in their own
# directory and only semantic_map_node.py puts it on the path.
set(SEMANTIC_MAP_LIB ${CMAKE_CURRENT_SOURCE_DIR}/third_party/semantic_map)
if(EXISTS ${SEMANTIC_MAP_LIB}/core/graph.py)
  ament_get_python_install_dir(PYTHON_DIST_DIR)
  install(DIRECTORY ${SEMANTIC_MAP_LIB}/core ${SEMANTIC_MAP_LIB}/adapters
    DESTINATION ${PYTHON_DIST_DIR}/semantic_map_lib
    PATTERN "__pycache__" EXCLUDE)
  install(FILES ${SEMANTIC_MAP_LIB}/ports.py
    DESTINATION ${PYTHON_DIST_DIR}/semantic_map_lib)
else()
  # Only --finals needs it: do not break the other tasks' builds.
  message(WARNING "Semantic-Map is missing, semantic_map_node.py will not start. "
    "Run: git submodule update --init vision/packages/vision_general/third_party/semantic_map")
endif()


######################### Python  #########################
#Python compatability
ament_python_install_package(utils PACKAGE_DIR scripts/utils)

#Install python scripts
file(GLOB PYTHON_SCRIPTS scripts/*.py)

# Install python scripts
install(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# VisionRuntime and the nodes that subclass it.
file(GLOB TASK_NODE_SCRIPTS scripts/task_nodes/*.py)
install(PROGRAMS
  ${TASK_NODE_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install models
file(GLOB MODEL_SCRIPTS scripts/models/*.py)
install(PROGRAMS
  ${MODEL_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}/models
)

# Model weights — installed alongside the Python model modules
file(GLOB MODEL_WEIGHTS models/*.task)
install(FILES
  ${MODEL_WEIGHTS}
  DESTINATION lib/${PROJECT_NAME}/models
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
