cmake_minimum_required(VERSION 3.8)
project(frida_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Specify the .srv and .msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NewHost.srv"
  "msg/Person.msg"
  "msg/PersonList.msg"
  DEPENDENCIES std_msgs 
)

# Export dependencies for other packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
