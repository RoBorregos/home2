cmake_minimum_required(VERSION 3.8)
project(frida_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Explicitly collect files by type
set(action_files
  "hri/action/Conversate.action"
  "manipulation/action/ManipulationAction.action"
  "manipulation/action/MoveJoints.action"
  "manipulation/action/MoveToPose.action"
  "manipulation/action/PickMotion.action"
  "manipulation/action/PickTask.action"
  "manipulation/action/PlaceMotion.action"
  "navigation/action/Move.action"
  "vision/action/DetectPerson.action"
  "vision/action/Xarmmove.action"
)

set(msg_files
  "hri/msg/AudioData.msg"
  "hri/msg/AudioDataStamped.msg"
  "hri/msg/AudioInfo.msg"
  "hri/msg/CommandList.msg"
  "hri/msg/Command.msg"
  "manipulation/msg/CollisionObject.msg"
  "manipulation/msg/ManipulationTask.msg"
  "manipulation/msg/Constraint.msg"
  "manipulation/msg/PickParams.msg"
  "manipulation/msg/PickResult.msg"
  "manipulation/msg/PlaceParams.msg"
  "vision/msg/ObjectDetectionArray.msg"
  "vision/msg/ObjectDetection.msg"
  "vision/msg/PersonList.msg"
  "vision/msg/Person.msg"
  "vision/msg/ShelfArray.msg"
  "vision/msg/ShelfDetection.msg"
)

set(srv_files
  "hri/srv/AddEntry.srv"
  "hri/srv/AudioText.srv"
  "hri/srv/BuildEmbeddings.srv"
  "hri/srv/CategorizeShelves.srv"
  "hri/srv/CommandInterpreter.srv"
  "hri/srv/CommonInterest.srv"
  "hri/srv/ExtractInfo.srv"
  "hri/srv/Grammar.srv"
  "hri/srv/GuestInfo.srv"
  "hri/srv/IsNegative.srv"
  "hri/srv/IsPositive.srv"
  "hri/srv/LLMWrapper.srv"
  "hri/srv/QueryEntry.srv"
  "hri/srv/Speak.srv"
  "hri/srv/STT.srv"
  "hri/srv/UpdateHotwords.srv"
  "manipulation/srv/AddCollisionObjects.srv"
  "manipulation/srv/AddPickPrimitives.srv"
  "manipulation/srv/AttachCollisionObject.srv"
  "manipulation/srv/ClusterObjectFromPoint.srv"
  "manipulation/srv/GetCollisionObjects.srv"
  "manipulation/srv/GetJoints.srv"
  "manipulation/srv/GetOptimalPositionForPlane.srv"
  "manipulation/srv/GetPlaneBbox.srv"
  "manipulation/srv/GraspDetection.srv"
  "manipulation/srv/HeatmapPlace.srv"
  "manipulation/srv/PickPerceptionService.srv"
  "manipulation/srv/PlacePerceptionService.srv"
  "manipulation/srv/ReadPcdFile.srv"
  "manipulation/srv/RemoveCollisionObject.srv"
  "manipulation/srv/RemovePlane.srv"
  "manipulation/srv/Test.srv"
  "manipulation/srv/ToggleServo.srv"
  "task_manager/srv/FollowFace.srv"
  "task_manager/srv/PointTransformation.srv"
  "task_manager/srv/ReturnAreas.srv"
  "vision/srv/BeverageLocation.srv"
  "vision/srv/CountByColor.srv"
  "vision/srv/CountByPose.srv"
  "vision/srv/CountBy.srv"
  "vision/srv/CropQuery.srv"
  "vision/srv/DetectionHandler.srv"
  "vision/srv/DetectPointingObject.srv"
  "vision/srv/FindSeat.srv"
  "vision/srv/PersonPoseGesture.srv"
  "vision/srv/PersonPosture.srv"
  "vision/srv/Query.srv"
  "vision/srv/SaveName.srv"
  "vision/srv/SetDetectorClasses.srv"
  "vision/srv/SetPointingObjectClasses.srv"
  "vision/srv/ShelfDetectionHandler.srv"
  "vision/srv/TrackBy.srv"
)

# Generate interfaces only keep the necessary runtimes (python, c++)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs unique_identifier_msgs action_msgs geometry_msgs sensor_msgs shape_msgs
)

# Installation configurations
install(
  DIRECTORY hri manipulation navigation task_manager vision
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies for other packages
ament_export_dependencies(rosidl_default_runtime)
ament_package()