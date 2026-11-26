cmake_minimum_required(VERSION 3.8)
#if(POLICY CMP0148)
#  cmake_policy(SET CMP0148 NEW)
#endif()
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

# Comma-separated modules to build (hri,manipulation,navigation,task_manager,vision), or "all".
set(FRIDA_MODULES "all" CACHE STRING "Comma-separated frida_interfaces modules to build, or 'all'")
set(_frida_modules "${FRIDA_MODULES}")
unset(FRIDA_MODULES CACHE)  # Not sticky: a build without -D always builds all

set(hri_files
  "hri/action/SpeechStream.action"
  "hri/msg/AudioData.msg"
  "hri/msg/AudioDataStamped.msg"
  "hri/msg/AudioInfo.msg"
  "hri/msg/CommandList.msg"
  "hri/msg/Command.msg"
  "hri/srv/AnswerQuestion.srv"
  "hri/srv/AddEntry.srv"
  "hri/srv/AudioText.srv"
  "hri/srv/BuildEmbeddings.srv"
  "hri/srv/CommandInterpreter.srv"
  "hri/srv/ExtractInfo.srv"
  "hri/srv/Grammar.srv"
  "hri/srv/GuestInfo.srv"
  "hri/srv/IsNegative.srv"
  "hri/srv/IsPositive.srv"
  "hri/srv/IsCoherent.srv"
  "hri/srv/LLMWrapper.srv"
  "hri/srv/FindClosest.srv"
  "hri/srv/QueryEntry.srv"
  "hri/srv/Speak.srv"
  "hri/srv/STT.srv"
  "hri/srv/UpdateHotwords.srv"
)

set(manipulation_files
  "manipulation/action/ManipulationAction.action"
  "manipulation/action/MoveJoints.action"
  "manipulation/action/MoveToPose.action"
  "manipulation/action/GoToHand.action"
  "manipulation/action/PickTask.action"
  "manipulation/msg/CollisionObject.msg"
  "manipulation/msg/ManipulationTask.msg"
  "manipulation/msg/Constraint.msg"
  "manipulation/msg/PickParams.msg"
  "manipulation/msg/PlaceParams.msg"
  "manipulation/msg/GripperGraspState.msg"
  "manipulation/msg/PourParams.msg"
  "manipulation/msg/ShelfLevel.msg"
  "manipulation/srv/AddCollisionObjects.srv"
  "manipulation/srv/AddPickPrimitives.srv"
  "manipulation/srv/AttachCollisionObject.srv"
  "manipulation/srv/ClusterObjectFromPoint.srv"
  "manipulation/srv/DetectShelf.srv"
  "manipulation/srv/GetCollisionObjects.srv"
  "manipulation/srv/GetJoints.srv"
  "manipulation/srv/GetOptimalPositionForPlane.srv"
  "manipulation/srv/GetOptimalPoseForPlane.srv"
  "manipulation/srv/GetPlaneBbox.srv"
  "manipulation/srv/GraspDetection.srv"
  "manipulation/srv/HeatmapPlace.srv"
  "manipulation/srv/PickPerceptionService.srv"
  "manipulation/srv/PlacePerceptionService.srv"
  "manipulation/srv/ReadPcdFile.srv"
  "manipulation/srv/RemoveCollisionObject.srv"
  "manipulation/srv/RemoveVerticalPlane.srv"
  "manipulation/srv/RemovePlane.srv"
  "manipulation/srv/Test.srv"
  "manipulation/srv/ToggleServo.srv"
  "manipulation/srv/PlayTrayectory.srv"
  "manipulation/srv/GenerateGrasps.srv"
  "manipulation/srv/FixedDistanceMove.srv"
  "manipulation/srv/DockToHandle.srv"
)

set(navigation_files
  "navigation/action/Move.action"
  "navigation/msg/NodeStatus.msg"
  "navigation/msg/MonitorReport.msg"
  "navigation/srv/MapAreas.srv"
  "navigation/srv/MoveLocation.srv"
  "navigation/srv/NavQuery.srv"
  "navigation/srv/DockTable.srv"
  "navigation/srv/GetRobotPose.srv"
  "navigation/srv/GoToPose.srv"
  "navigation/srv/ApproachPoint.srv"
  "navigation/srv/GetAreaForPoint.srv"
  "navigation/srv/PlanPatrol.srv"
)

set(task_manager_files
  "task_manager/srv/FollowFace.srv"
  "task_manager/srv/PointTransformation.srv"
  "task_manager/srv/ReturnLocation.srv"
  "task_manager/srv/CheckDoor.srv"
  "task_manager/srv/HearMultiThread.srv"
  "task_manager/srv/WaitForControllerInput.srv"
  "task_manager/srv/PositionGet.srv"
)

set(vision_files
  "vision/action/ExampleAction.action"
  "vision/action/Xarmmove.action"
  "vision/msg/CustomerTable.msg"
  "vision/msg/ObjectDetectionArray.msg"
  "vision/msg/ObjectDetection.msg"
  "vision/msg/PersonList.msg"
  "vision/msg/Person.msg"
  "vision/msg/Point2D.msg"
  "vision/msg/ShelfArray.msg"
  "vision/msg/ShelfDetection.msg"
  "vision/msg/Detection.msg"
  "vision/msg/PersonPose.msg"
  "vision/srv/BeverageLocation.srv"
  "vision/srv/CountByColor.srv"
  "vision/srv/CountByPose.srv"
  "vision/srv/CountBy.srv"
  "vision/srv/ChairsToRemove.srv"
  "vision/srv/CropQuery.srv"
  "vision/srv/CustomerTables.srv"
  "vision/srv/DetectionHandler.srv"
  "vision/srv/MoondreamDetection.srv"
  "vision/srv/DishwasherDetection.srv"
  "vision/srv/DetectHand.srv"
  "vision/srv/DetectPerson.srv"
  "vision/srv/DetectPointingObject.srv"
  "vision/srv/FindSeat.srv"
  "vision/srv/PersonPoseGesture.srv"
  "vision/srv/PersonPosture.srv"
  "vision/srv/Query.srv"
  "vision/srv/ObjectPoints.srv"
  "vision/srv/SaveName.srv"
  "vision/srv/SetDetectorClasses.srv"
  "vision/srv/SetPointingObjectClasses.srv"
  "vision/srv/ShelfDetectionHandler.srv"
  "vision/srv/TrackBy.srv"
  "vision/srv/ReadQr.srv"
  "vision/srv/ShowDetection.srv"
  "vision/srv/Customer.srv"
  "vision/srv/YoloDetect.srv"
  "vision/srv/TrashcanDetection.srv"
  "vision/srv/SetTrashCategory.srv"
  "vision/srv/PersonInside.srv"
  "vision/srv/PersonInsideReq.srv"
  "vision/srv/IsTalking.srv"
  "vision/srv/EmbedText.srv"
  "vision/srv/EmbedCrops.srv"
  "vision/srv/DetectPose.srv"
)

set(frida_interfaces_valid_modules hri manipulation navigation task_manager vision)

if(_frida_modules STREQUAL "" OR _frida_modules STREQUAL "all")
  set(frida_interfaces_requested_modules ${frida_interfaces_valid_modules})
else()
  string(REPLACE "," ";" frida_interfaces_requested_modules "${_frida_modules}")
endif()

set(interface_files "")
foreach(module ${frida_interfaces_requested_modules})
  list(FIND frida_interfaces_valid_modules "${module}" module_index)
  if(module_index EQUAL -1)
    message(FATAL_ERROR "Unknown frida_interfaces module '${module}'. Valid modules: ${frida_interfaces_valid_modules}")
  endif()
  list(APPEND interface_files ${${module}_files})
  install(DIRECTORY ${module} DESTINATION share/${PROJECT_NAME})
endforeach()

# Generate interfaces only keep the necessary runtimes (python, c++)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES std_msgs unique_identifier_msgs action_msgs geometry_msgs sensor_msgs shape_msgs
)

# Export dependencies for other packages
ament_export_dependencies(rosidl_default_runtime)
ament_package()
