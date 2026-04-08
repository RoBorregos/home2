/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "Roborregos @Home documentation", "index.html", [
    [ "ROS2 @Home Guide", "index.html", "index" ],
    [ "Manipulation", "md_manipulation_README.html", [
      [ "Setup with docker", "md_manipulation_README.html#autotoc_md1", [
        [ "Requirements", "md_manipulation_README.html#autotoc_md2", null ],
        [ "Building the image and container", "md_manipulation_README.html#autotoc_md3", null ]
      ] ],
      [ "Running the project", "md_manipulation_README.html#autotoc_md4", null ]
    ] ],
    [ "Rosbag folder", "md_navigation_packages_map_context_config_rosbag_README.html", [
      [ "Setup", "md_navigation_packages_map_context_config_rosbag_README.html#autotoc_md6", null ],
      [ "Running", "md_navigation_packages_map_context_config_rosbag_README.html#autotoc_md7", null ]
    ] ],
    [ "Home Navigation", "md_navigation_README.html", null ],
    [ "README", "md_vision_README.html", null ],
    [ "README", "md_robot_description_frida_description_README.html", null ],
    [ "Place Params Special Request", "md_frida_interfaces_manipulation_msg_PlaceParamsSpecialRequest.html", [
      [ "Example of a special request", "md_frida_interfaces_manipulation_msg_PlaceParamsSpecialRequest.html#autotoc_md10", null ]
    ] ],
    [ "Names", "md_frida_constants_data_names.html", null ],
    [ "Class drinks (drink)", "md_frida_constants_data_objects.html", [
      [ "Class fruits (fruit)", "md_frida_constants_data_objects.html#autotoc_md13", null ],
      [ "Class snacks (snack)", "md_frida_constants_data_objects.html#autotoc_md14", null ],
      [ "Class foods (food)", "md_frida_constants_data_objects.html#autotoc_md15", null ],
      [ "Class dishes (dish)", "md_frida_constants_data_objects.html#autotoc_md16", null ],
      [ "Class cleaning_supplies (cleaning_supply)", "md_frida_constants_data_objects.html#autotoc_md17", null ]
    ] ],
    [ "CycloneDDS Setup", "md_docs_cyclonedds_setup.html", [
      [ "Architecture", "md_docs_cyclonedds_setup.html#autotoc_md19", null ],
      [ "Shared Memory (SHM) vs UDP", "md_docs_cyclonedds_setup.html#autotoc_md20", [
        [ "SHM Memory Breakdown (when enabled)", "md_docs_cyclonedds_setup.html#autotoc_md21", null ],
        [ "ZED SHM Workarounds", "md_docs_cyclonedds_setup.html#autotoc_md22", null ]
      ] ],
      [ "Files", "md_docs_cyclonedds_setup.html#autotoc_md23", null ],
      [ "Usage", "md_docs_cyclonedds_setup.html#autotoc_md24", [
        [ "Local Development (no SHM, default)", "md_docs_cyclonedds_setup.html#autotoc_md25", null ],
        [ "Orin AGX / High-Memory Machine (with SHM)", "md_docs_cyclonedds_setup.html#autotoc_md26", null ],
        [ "Bare Metal (Orin, direct install)", "md_docs_cyclonedds_setup.html#autotoc_md27", null ],
        [ "Revert to FastDDS", "md_docs_cyclonedds_setup.html#autotoc_md28", null ],
        [ "Override Interface at Runtime", "md_docs_cyclonedds_setup.html#autotoc_md29", null ]
      ] ],
      [ "What It Configures", "md_docs_cyclonedds_setup.html#autotoc_md30", [
        [ "CycloneDDS XML (<tt>/etc/cyclonedds.xml</tt>)", "md_docs_cyclonedds_setup.html#autotoc_md31", null ],
        [ "Kernel Tuning (<tt>/etc/sysctl.d/60-cyclonedds-buffers.conf</tt>)", "md_docs_cyclonedds_setup.html#autotoc_md32", null ],
        [ "Iceoryx / RouDi (when <tt>CYCLONE_SHM=1</tt>)", "md_docs_cyclonedds_setup.html#autotoc_md33", null ]
      ] ],
      [ "Finding Your Network Interface", "md_docs_cyclonedds_setup.html#autotoc_md34", null ],
      [ "Troubleshooting", "md_docs_cyclonedds_setup.html#autotoc_md35", [
        [ "RouDi SIGBUS / fails to start", "md_docs_cyclonedds_setup.html#autotoc_md36", null ],
        [ "Stale iceoryx artifacts", "md_docs_cyclonedds_setup.html#autotoc_md37", null ]
      ] ],
      [ "Reference", "md_docs_cyclonedds_setup.html#autotoc_md38", null ]
    ] ],
    [ "Expo demo Feb 13, 2025", "md_docs_expo_demo.html", [
      [ "Running HRI", "md_docs_expo_demo.html#autotoc_md40", null ]
    ] ],
    [ "frida_interfaces", "md_docs_interfaces.html", [
      [ "MoveJoints.action definition", "md_docs_interfaces.html#autotoc_md42", null ],
      [ "Other Messages", "md_docs_interfaces.html#autotoc_md44", [
        [ "SomeMessage.msg", "md_docs_interfaces.html#autotoc_md45", null ],
        [ "SomeService.srv", "md_docs_interfaces.html#autotoc_md46", null ]
      ] ]
    ] ],
    [ "HRI", "md_docs_Run_Areas_hri.html", [
      [ "Flags", "md_docs_Run_Areas_hri.html#autotoc_md62", null ],
      [ "Running specific containers", "md_docs_Run_Areas_hri.html#autotoc_md63", null ]
    ] ],
    [ "Manipulation", "md_docs_Run_Areas_Manipulation_manipulation.html", [
      [ "Docker setup", "md_docs_Run_Areas_Manipulation_manipulation.html#autotoc_md65", null ],
      [ "Running the vision module", "md_docs_Run_Areas_Manipulation_manipulation.html#autotoc_md66", [
        [ "Additional Information", "md_docs_Run_Areas_Manipulation_manipulation.html#autotoc_md73", null ]
      ] ]
    ] ],
    [ "Running Pick and Place", "md_docs_Run_Areas_Manipulation_pick_and_place.html", [
      [ "Launching the Robot", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md75", [
        [ "Simulation", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md76", null ],
        [ "Real RObot", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md77", [
          [ "Robot interface", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md78", null ],
          [ "3D Camera", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md79", null ]
        ] ]
      ] ],
      [ "Launching utilities", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md80", [
        [ "Object detector", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md81", null ]
      ] ],
      [ "Launch Pick and Place main code", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md82", [
        [ "Pick and Place pipeline", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md83", null ]
      ] ],
      [ "Usage", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md84", null ]
    ] ],
    [ "Navigation Docs", "md_docs_Run_Areas_nav.html", [
      [ "How to Run", "md_docs_Run_Areas_nav.html#autotoc_md86", null ],
      [ "Running Nav Basics", "md_docs_Run_Areas_nav.html#autotoc_md88", [
        [ "Arguments", "md_docs_Run_Areas_nav.html#autotoc_md89", null ],
        [ "Run Command", "md_docs_Run_Areas_nav.html#autotoc_md90", null ]
      ] ],
      [ "Running AMCL Localization", "md_docs_Run_Areas_nav.html#autotoc_md92", [
        [ "Arguments", "md_docs_Run_Areas_nav.html#autotoc_md93", null ],
        [ "Run Command", "md_docs_Run_Areas_nav.html#autotoc_md94", null ]
      ] ],
      [ "Running Navigation Node", "md_docs_Run_Areas_nav.html#autotoc_md96", [
        [ "Arguments", "md_docs_Run_Areas_nav.html#autotoc_md97", null ],
        [ "Run Command", "md_docs_Run_Areas_nav.html#autotoc_md98", null ]
      ] ]
    ] ],
    [ "Vision", "md_docs_Run_Areas_vision.html", [
      [ "Docker setup", "md_docs_Run_Areas_vision.html#autotoc_md100", null ],
      [ "Running the vision module", "md_docs_Run_Areas_vision.html#autotoc_md101", null ],
      [ "Structure", "md_docs_Run_Areas_vision.html#autotoc_md104", null ],
      [ "Camera", "md_docs_Run_Areas_vision.html#autotoc_md105", null ]
    ] ],
    [ "Help me carry", "md_docs_Run_Tasks_help_me_carry.html", [
      [ "Integration", "md_docs_Run_Tasks_help_me_carry.html#autotoc_md107", null ],
      [ "Vision", "md_docs_Run_Tasks_help_me_carry.html#autotoc_md108", null ]
    ] ],
    [ "Receptionist", "md_docs_Run_Tasks_receptionist.html", [
      [ "ZED", "md_docs_Run_Tasks_receptionist.html#autotoc_md110", null ],
      [ "Integration", "md_docs_Run_Tasks_receptionist.html#autotoc_md111", null ],
      [ "Vision", "md_docs_Run_Tasks_receptionist.html#autotoc_md112", null ],
      [ "HRI", "md_docs_Run_Tasks_receptionist.html#autotoc_md113", null ],
      [ "Navigation", "md_docs_Run_Tasks_receptionist.html#autotoc_md114", null ],
      [ "Manipulation", "md_docs_Run_Tasks_receptionist.html#autotoc_md115", [
        [ "Launch", "md_docs_Run_Tasks_receptionist.html#autotoc_md116", null ],
        [ "Arm bringup", "md_docs_Run_Tasks_receptionist.html#autotoc_md117", null ],
        [ "Motion Planning", "md_docs_Run_Tasks_receptionist.html#autotoc_md118", null ],
        [ "Follow face", "md_docs_Run_Tasks_receptionist.html#autotoc_md119", null ]
      ] ]
    ] ],
    [ "Storing Groceries", "md_docs_Run_Tasks_storing_groceries.html", [
      [ "Integration", "md_docs_Run_Tasks_storing_groceries.html#autotoc_md121", null ],
      [ "Vision", "md_docs_Run_Tasks_storing_groceries.html#autotoc_md122", null ]
    ] ],
    [ "Improving your ROS2 workflow with vscode", "md_docs_setup_vscode.html", [
      [ "Prerequisites", "md_docs_setup_vscode.html#autotoc_md124", null ],
      [ "Setup", "md_docs_setup_vscode.html#autotoc_md125", null ],
      [ "Troubleshooting", "md_docs_setup_vscode.html#autotoc_md126", null ]
    ] ],
    [ "Project setup", "md_docs_Setup.html", [
      [ "Pre-commit", "md_docs_Setup.html#autotoc_md128", null ],
      [ "Ruff", "md_docs_Setup.html#autotoc_md129", [
        [ "Vscode integration", "md_docs_Setup.html#autotoc_md130", null ],
        [ "Other integrations", "md_docs_Setup.html#autotoc_md131", null ]
      ] ]
    ] ],
    [ "Decorators", "md_docs_task_manager_decorators.html", [
      [ "Mockable", "md_docs_task_manager_decorators.html#autotoc_md133", [
        [ "Parameters", "md_docs_task_manager_decorators.html#autotoc_md134", null ],
        [ "Implementation", "md_docs_task_manager_decorators.html#autotoc_md135", null ]
      ] ],
      [ "Service Check", "md_docs_task_manager_decorators.html#autotoc_md136", [
        [ "Parameters", "md_docs_task_manager_decorators.html#autotoc_md137", null ],
        [ "Implementation", "md_docs_task_manager_decorators.html#autotoc_md138", null ]
      ] ],
      [ "Example", "md_docs_task_manager_decorators.html#autotoc_md139", null ]
    ] ],
    [ "Namespaces", "namespaces.html", [
      [ "Namespace List", "namespaces.html", "namespaces_dup" ],
      [ "Namespace Members", "namespacemembers.html", [
        [ "All", "namespacemembers.html", "namespacemembers_dup" ],
        [ "Functions", "namespacemembers_func.html", "namespacemembers_func" ],
        [ "Variables", "namespacemembers_vars.html", "namespacemembers_vars" ]
      ] ]
    ] ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", "functions_func" ],
        [ "Variables", "functions_vars.html", "functions_vars" ],
        [ "Typedefs", "functions_type.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", null ],
        [ "Functions", "globals_func.html", null ],
        [ "Variables", "globals_vars.html", null ],
        [ "Typedefs", "globals_type.html", null ],
        [ "Enumerations", "globals_enum.html", null ],
        [ "Enumerator", "globals_eval.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"AddCollisionObjects_8srv.html",
"classIKSolver.html#a67b37120177c48213784f088ced73b2b",
"classPlanner_1_1Planner.html#a2834bbbc877ce26d596d673da9ab5ff1",
"class__storing__groceries_1_1StoringGroceriesTM.html#adf1056ec281c89a1b19035fdac9a1f87",
"classcustomer__node_1_1CustomerNode.html#a2b0a404db6687ba1926f674f8b04f17c",
"classdashgo__driver_1_1dashgo__stm32_1_1Stm32.html#ad8a71c977e61c82b44fb707e79f83c0a",
"classegpsr__task__manager_1_1EGPSRTM.html#afd6df107ceb53bc24f74cbb550c8892c",
"classflat__grasp__estimator_1_1FlatGraspEstimator.html#af9c1d12cb05c1132a219ac1d0db82e95",
"classfollow__person__node_1_1FollowPersonNode.html#a3e121c497d7dce3880d5a09d29f79689",
"classfrida__motion__planning_1_1motion__planning__server_1_1MotionPlanningServer.html#affc6b0285a02cd7886ad966a7dac46f8",
"classhelp__me__carry_1_1HelpMeCarryTM.html#a1478fbe3ae6765800c60935183705c49",
"classikfast_1_1IkSolutionList.html#ae341a33d5aee644cac867e3edd2a9916",
"classmap__publisher_1_1SimpleMapServer.html#a0fc68349714ef68869aeb88cc2e7dabf",
"classmove__to__location_1_1MoveActionServer.html#a0206ecc48a965397ff8088a1a0c7a6c8",
"classnav__ui_1_1NavUI.html#a23d9294ed28444500245fb79f63ec120",
"classobject__detector__node_1_1object__detector__node.html#aef47122bc5e39b92fc02b210e90b0e27",
"classpick__and__place_1_1manipulation__core_1_1ManipulationCore.html#a7f24ca08b5eec6f25faae2d64db79c5b",
"classpointing__detection_1_1DetectPointingObjectServer.html#a881d45334b8888151247cf27152da324",
"classrestaurant__commands_1_1RESTAURANTCommands.html#ae719a43ba6b884d5b7978e885d0d19a2",
"classtask__manager_1_1subtask__managers_1_1gpsr__tasks_1_1GPSRTask.html#a88aede36b7bce4f0903eccd043198982",
"classtask__manager_1_1subtask__managers_1_1manipulation__tasks_1_1ManipulationTasks.html#a79b6afeee37ef3d8043aefe54ff445bd",
"classtask__manager_1_1subtask__managers_1_1vision__tasks_1_1VisionTasks.html#ac8b91583b99f6c7614fcf7f43ca06120",
"classtest__manager_1_1TestTaskManager.html#a1524259ed0bd426610078938e4251af1",
"classtransform__target_1_1PointTransformer.html#a79ad5d3c4ce2c99d630f9aae5eca7c98",
"classzero__shot__object__detector__node_1_1zero__shot__object__detector__node.html#ab2660ef7f1d57db5f7fe6ebcf134fa66",
"example__node_8py.html#a0830ca6b73f60f3404af93f6221efd83",
"gpsr__test__commands_8py.html#af61153ac2e4af84cfaf1fcc0eca7f0e8",
"manipulation__constants__cpp_8hpp.html#a56af7b9da0fb3dfb9f8c337970b4dbd2",
"namespacePickManager.html#afcfec5898dcc59164a20e8562b51f08d",
"namespacefrida__constants_1_1hri__constants.html#a57a8000db92c380cf72a62b5afca2ba8",
"namespacehric__commands.html#a295f86308e2091a1cafb49a1abbadfe9",
"namespacepick__and__place_1_1pour__server.html",
"namespacetemp__follow.html#ad4d7ef6faa21d32ef936bebad810409f",
"pointing__detection_8py.html#a197c3d9ed96b3db5850117c5d78e684a",
"test__vision__manager_8py.html#a0ac83c3be8619e480c385351b9f84b6b",
"xarm__launch_8launch_8py.html#a6120858d912a0dc3d2783725c36b7d04"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';