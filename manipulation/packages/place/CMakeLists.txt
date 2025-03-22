# cmake_minimum_required(VERSION 3.0.2)
# project(object_detector_3d)

# ## Compile as C++11, supported in ROS Kinetic and newer
# # add_compile_options(-std=c++11)

# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
# actionlib
# actionlib_msgs
# geometry_msgs
# shape_msgs
# moveit_msgs
# moveit_core
# moveit_ros_planning
# moveit_ros_planning_interface
# moveit_ros_perception
# roscpp
# roslib
# rospy
# pcl_ros
# pcl_conversions
# std_msgs
# std_srvs
# message_generation
# tf
# tf2
# tf2_ros
# tf2_eigen
# tf2_geometry_msgs
# laser_geometry
# gpd_ros
# sensor_msgs
# object_detector_2d
# frida_manipulation_interfaces
# )

# ## System dependencies are found with CMake's conventions
# # find_package(Boost REQUIRED COMPONENTS system)


# ## Uncomment this if the package has a setup.py. This macro ensures
# ## modules and global scripts declared therein get installed
# ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# # catkin_python_setup()

# ################################################
# ## Declare ROS messages, services and actions ##
# ################################################

# ## To declare and build messages, services or actions from within this
# ## package, follow these steps:
# ## * Let MSG_DEP_SET be the set of packages whose message types you use in
# ##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# ## * In the file package.xml:
# ##   * add a build_depend tag for "message_generation"
# ##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# ##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# ##     but can be declared for certainty nonetheless:
# ##     * add a exec_depend tag for "message_runtime"
# ## * In this file (CMakeLists.txt):
# ##   * add "message_generation" and every package in MSG_DEP_SET to
# ##     find_package(catkin REQUIRED COMPONENTS ...)
# ##   * add "message_runtime" and every package in MSG_DEP_SET to
# ##     catkin_package(CATKIN_DEPENDS ...)
# ##   * uncomment the add_*_files sections below as needed
# ##     and list every .msg/.srv/.action file to be processed
# ##   * uncomment the generate_messages entry below
# ##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# ## Generate messages in the 'msg' folder
# # add_message_files(
# #   FILES
# #   Message1.msg
# #   Message2.msg
# # )

# ## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Clustering.srv
# )

# ## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   DetectObjects3D.action
#   GetPlacePosition.action
# )

# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs
#   geometry_msgs
#   shape_msgs
#   std_msgs
#   gpd_ros
#   object_detector_2d
#   object_detector_3d
# )

# ################################################
# ## Declare ROS dynamic reconfigure parameters ##
# ################################################

# ## To declare and build dynamic reconfigure parameters within this
# ## package, follow these steps:
# ## * In the file package.xml:
# ##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# ## * In this file (CMakeLists.txt):
# ##   * add "dynamic_reconfigure" to
# ##     find_package(catkin REQUIRED COMPONENTS ...)
# ##   * uncomment the "generate_dynamic_reconfigure_options" section below
# ##     and list every .cfg file to be processed

# ## Generate dynamic reconfigure parameters in the 'cfg' folder
# # generate_dynamic_reconfigure_options(
# #   cfg/DynReconf1.cfg
# #   cfg/DynReconf2.cfg
# # )

# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if your package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#   LIBRARIES
#   INCLUDE_DIRS
#   CATKIN_DEPENDS
#     roscpp
#     roslib
#     message_runtime
#     moveit_core
#     moveit_ros_planning_interface
# )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
# # include
# ${catkin_INCLUDE_DIRS}
# )

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=shadow")

# ## Declare a C++ library
# # add_library(${PROJECT_NAME}
# #   src/${PROJECT_NAME}/object_detector_3d.cpp
# # )

# ## Add cmake target dependencies of the library
# ## as an example, code may need to be generated before libraries
# ## either from message generation or dynamic reconfigure
# # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Declare a C++ executable
# ## With catkin_make all packages are built within a single CMake context
# ## The recommended prefix ensures that target names across packages don't collide
# add_executable(Detection3D src/Detection3D.cpp)
# add_executable(Detection3DPlace src/Detection3DPlace.cpp)
# add_executable(Clustering_Client src/Clustering_Client.cpp)
# add_executable(SaveOctomap src/SaveOctomap.cpp)
# add_executable(Detection3DKmeans src/Detection3DKmeans.cpp)
# add_executable(Detection3DHeatmap src/Detection3DHeatmap.cpp)


# ## Rename C++ executable without prefix
# ## The above recommended prefix causes long target names, the following renames the
# ## target back to the shorter version for ease of user use
# ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# ## Add cmake target dependencies of the executable
# ## same as for the library above
# # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(Detection3D object_detector_3d_generate_messages_cpp gpd_ros_generate_messages_cpp)
# add_dependencies(Detection3DPlace object_detector_3d_generate_messages_cpp gpd_ros_generate_messages_cpp)
# add_dependencies(Clustering_Client object_detector_3d_generate_messages_cpp gpd_ros_generate_messages_cpp)
# add_dependencies(SaveOctomap object_detector_3d_generate_messages_cpp gpd_ros_generate_messages_cpp)
# add_dependencies(Detection3DKmeans object_detector_3d_generate_messages_cpp gpd_ros_generate_messages_cpp)
# add_dependencies(Detection3DHeatmap object_detector_3d_generate_messages_cpp gpd_ros_generate_messages_cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(Detection3D ${catkin_LIBRARIES})
# target_link_libraries(Detection3DPlace ${catkin_LIBRARIES})
# target_link_libraries(Clustering_Client ${catkin_LIBRARIES})
# target_link_libraries(SaveOctomap ${catkin_LIBRARIES})
# target_link_libraries(Detection3DKmeans ${catkin_LIBRARIES})
# target_link_libraries(Detection3DHeatmap ${catkin_LIBRARIES})


# ## Specify libraries to link a library or executable target against
# # target_link_libraries(${PROJECT_NAME}_node
# #   ${catkin_LIBRARIES}
# # )

# #############
# ## Install ##
# #############

# # all install targets should use catkin DESTINATION variables
# # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# ## Mark executable scripts (Python etc.) for installation
# ## in contrast to setup.py, you can choose the destination
# # catkin_install_python(PROGRAMS
# #   scripts/my_python_script
# #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# # )

# ## Mark executables for installation
# ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# # install(TARGETS ${PROJECT_NAME}_node
# #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# # )

# ## Mark libraries for installation
# ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# # install(TARGETS ${PROJECT_NAME}
# #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# # )

# ## Mark cpp header files for installation
# # install(DIRECTORY include/${PROJECT_NAME}/
# #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# #   FILES_MATCHING PATTERN "*.h"
# #   PATTERN ".svn" EXCLUDE
# # )

# ## Mark other files for installation (e.g. launch and bag files, etc.)
# # install(FILES
# #   # myfile1
# #   # myfile2
# #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# # )

# #############
# ## Testing ##
# #############

# ## Add gtest based cpp test target and link libraries
# # catkin_add_gtest(${PROJECT_NAME}-test test/test_object_detector_3d.cpp)
# # if(TARGET ${PROJECT_NAME}-test)
# #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# # endif()

# ## Add folders to be run by python nosetests
# # catkin_add_nosetests(test)





cmake_minimum_required(VERSION 3.8)
project(place)

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
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(frida_interfaces REQUIRED)
find_package(frida_constants REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)

# Install targets
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Python components
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/heatmapPlace_Server.py DESTINATION lib/${PROJECT_NAME})

ament_package()