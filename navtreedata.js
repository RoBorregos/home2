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
"classMoveItPlanner_1_1MoveItPlanner.html#a517710899e89e095f5aafe2c9acfa0c7",
"classPublishNode.html#a76e851048520c14a6779d2e5d755b105",
"classarm__pkg_1_1moveit__configs__builder__sim_1_1DualMoveItConfigsBuilder.html#ac307b1464bd5db0f429a32874ff7ada7",
"classdashgo__driver2_1_1DashgoDriver.html#a23a51b0353c72f32ea0be2ea08eaa72c",
"classdishwasher__node_1_1DishwasherNode.html#a09c4706d32ed666a26a6906955430493",
"classegsr__cut_1_1EGPSRTM.html#ae8b845fe49ed3e58e2a816abd1d4928a",
"classfollow__face__node__copt_1_1FollowFaceNode.html#a12ffb4c8efcb46bff66dbd60eb3c454f",
"classfollow__person__node_1_1FollowPersonNode.html#ae9f8b32e7ceff4e19d0f3b49d1875e72",
"classgpsr__commands_1_1GPSRCommands.html#a350c6cd5e539f98af2f84cc77133b90c",
"classhric__commands_1_1HRICCommands.html#abcabc9b1833c835cbb0ebf3f1bd1e253",
"classmap__area__tagger_1_1MapCanvas.html#a17a0c8251ccc26e8657e7208999a9a1a",
"classmoondream__node_1_1MoondreamNode.html#a45ba0eab06740fdbd8a5b234669bdac4",
"classnav__ui_1_1NavRosNode.html#a2a35a71f71c1faf977ef5f4d0a8a49e7",
"classobject__detector__node_1_1object__detector__node.html#a34b5af89b3f43e805c61c8c123c84f1a",
"classpick__and__place_1_1manipulation__client_1_1ManipulationClient.html",
"classpick__and__place_1_1pour__server_1_1PourMotionServer.html#ae4def19da1fafcde56e203a80a0a5104",
"classremove__person_1_1RemovePerson.html#a5856534b9fedd37a41427733b8438ec8",
"classstoring__groceries__manager_1_1StoringGroceriesManager.html#af5cc83beaeff4b9a40d332d017a68377",
"classtask__manager_1_1subtask__managers_1_1hri__tasks_1_1HRITasks.html#ae9ac27b7c3774a14a2a09e81d1dd6c79",
"classtask__manager_1_1subtask__managers_1_1vision__tasks_1_1VisionTasks.html#a837f910fae575752d6eb302dd5772514",
"classtemp__follow_1_1FollowPersonNode.html#afeca8105ced8e219b6db5d1e901728cd",
"classtracker__node__fregoso_1_1SingleTracker.html#a8e9bdcfcba057538dfb0cf73a401001e",
"classzero__shot__object__detector__node_1_1zero__shot__object__detector__node.html#a2560ee772931f221edc0988b0d8198b4",
"ds4__demo_8py.html#a9ee64277947c3be58860b6df419273bd",
"gpsr__single__tasks_8py.html#a342e4af688797931bd908d3bef0200da",
"manipulation__constants_8py.html#ac9123d02a104c1579ddaeb0a9cf75238",
"moondream__proto__pb2_8py.html#af44b393f7046597be158d7105696f7f0",
"namespacefollow__person__v2.html#a7fb91f35ad27ea903a93757a83a05e85",
"namespacegpsr__task__manager.html#a3082b3302b813b3c09b9f0d3da26865a",
"namespaceperson__in__map.html",
"namespacetask__manager_1_1subtask__managers_1_1vision__tasks.html#a49cfba4f063b64b58df1700773fd7995",
"perception__3d_8launch_8py.html#a0e9c55f0d673d8e39208565acad51d8d",
"test__manipulation__manager_8py.html#aa1d1240f6e1bf9a1472e8c9e914fcfb4"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';