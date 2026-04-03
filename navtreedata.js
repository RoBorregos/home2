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
        [ "Variables", "functions_vars.html", "functions_vars" ]
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
"classMoveItPlanner_1_1MoveItPlanner.html#a5ac227b43367b5713cede31e7f0a5f7d",
"classServo_1_1Servo.html",
"classarm__pkg_1_1moveit__configs__builder__sim_1_1DualMoveItConfigsBuilder.html#ad7812a5f103e5c32fe9aab8acbf51292",
"classdashgo__driver2_1_1DashgoDriver.html#a29561f314a8846bc7c3c035a03ad1090",
"classdishwasher__node_1_1DishwasherNode.html#a157ad973e7bcb2a9ab9ebc8561193e89",
"classegsr__cut_1_1EGPSRTM.html#aea19fdd19721045677f5f83ee7daa1d0",
"classfollow__face__node__copt_1_1FollowFaceNode.html#a1cf5dbc8c021bd645137a9b529f32e94",
"classfollow__person__node_1_1FollowPersonNode.html#aece6c95a2022996629688fa64b41bee2",
"classgpsr__commands_1_1GPSRCommands.html#a37658fa13c8e5d1f1f7685f1f3a1c27f",
"classhric__commands_1_1HRICCommands.html#a9a01bf9860dcce60e78862f94553fab5",
"classmap__area__tagger_1_1MapAreaTagger.html#afd4d69b1cb14853c97a9efd7810ee063",
"classmoondream__node_1_1MoondreamNode.html#a374a642f47a1121f425b0c5b4a0fda31",
"classnav__ui_1_1NavRosNode.html#a0971e1a7c117b1443ab0bf2848238ef0",
"classobject__detector__node_1_1object__detector__node.html#a1a795b450646f068352181af51719c2a",
"classpick__and__place_1_1keyboard__input_1_1KeyboardInput.html#ac80894cf47fc43e9af204ffa3c74985b",
"classpick__and__place_1_1pour__server_1_1PourMotionServer.html#aa78b9a66db0d521cc6ff7897c7145308",
"classremove__person_1_1RemovePerson.html#a2fd29bbd5d5a0231b815af76cfda6e6c",
"classstoring__groceries__manager_1_1StoringGroceriesManager.html#adb6bbe91c26552f14daeddaf32726875",
"classtask__manager_1_1subtask__managers_1_1hri__tasks_1_1HRITasks.html#adfaba4266e022be25e92766f7024772f",
"classtask__manager_1_1subtask__managers_1_1vision__tasks_1_1VisionTasks.html#a79aacea654115337d5b75d19a857cf95",
"classtemp__follow_1_1FollowPersonNode.html#af02cfa4ca63e59996d97b7dc39c7b47b",
"classtracker__node__fregoso_1_1SingleTracker.html#a81e15f94b15b576ddb68bcf3e776b62e",
"classzero__shot__object__detector__node_1_1zero__shot__object__detector__node.html#a13762850ac22d97d171968661abafe4e",
"downsample__pc_8launch_8py.html",
"gpsr__commands_8py.html#a7dce31766c14809575920f9d6ebe53a2",
"manipulation__constants_8py.html#ac9123d02a104c1579ddaeb0a9cf75238",
"moondream__proto__pb2_8py.html#af44b393f7046597be158d7105696f7f0",
"namespacefollow__person__v2.html#a7fb91f35ad27ea903a93757a83a05e85",
"namespacegpsr__task__manager.html#a3082b3302b813b3c09b9f0d3da26865a",
"namespacepick__and__place.html",
"namespacetask__manager_1_1subtask__managers_1_1vision__tasks.html#acc2ee6161006865e51a5b980585f620b",
"perception__utils_8py.html#adf7d95078b4cecc224d75c0f20109553",
"test__plan__parser_8py.html#a7c320a6064d3055f6213924d2437839a"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';