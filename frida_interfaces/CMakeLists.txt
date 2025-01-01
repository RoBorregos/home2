cmake_minimum_required(VERSION 3.8)
project(frida_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

# Specify the .srv and .msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "vision/srv/NewHost.srv"
  "vision/srv/FindSeat.srv"
  "vision/msg/Person.msg"
  "vision/msg/PersonList.msg"
  "vision/action/DetectPerson.action"
  DEPENDENCIES std_msgs unique_identifier_msgs action_msgs
)

# Export dependencies for other packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
