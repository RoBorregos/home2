cmake_minimum_required(VERSION 3.8)
project(task_manager)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(frida_interfaces REQUIRED)
find_package(frida_constants REQUIRED)


#Install directories
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME})

######################### Python  #########################
#Python compatability
ament_python_install_package(${PROJECT_NAME})

file (GLOB TASK_MANAGERS scripts/*.py)
file (GLOB SUBTASK_MANAGERS scripts/subtask_managers/*.py)
file (GLOB UTILS scripts/utils/*.py)


# Install python scripts
install(PROGRAMS
${TASK_MANAGERS}
DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
${SUBTASK_MANAGERS}
DESTINATION lib/${PROJECT_NAME}/subtask_managers
)

install(PROGRAMS
${UTILS}
DESTINATION lib/${PROJECT_NAME}/utils
)

install(PROGRAMS
${UTILS}
DESTINATION lib/${PROJECT_NAME}/subtask_managers/utils
)

file (GLOB CONFIG scripts/config/*.py)

# Install python config
install(DIRECTORY scripts/config/ DESTINATION lib/${PROJECT_NAME}/config FILES_MATCHING PATTERN "*.py")

######################### Testing  #########################
##For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

