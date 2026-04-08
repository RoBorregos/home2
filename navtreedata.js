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
"classIKSolver.html#a66a0610cba0da00e09deda3c72e3909f",
"classPlanner_1_1Planner.html#a275192f33240079f6d2c906f594c68cb",
"class__storing__groceries_1_1StoringGroceriesTM.html#add47d7edd6f6616ad6552d9717de612e",
"classcustomer__node_1_1CustomerNode.html#a7cb81a4782fcd1393fa353c4ea3b10e3",
"classdashgo__driver_1_1dashgo__stm32_1_1Stm32.html#aed55ad06bc9d7097797c122ef8c1af23",
"classegsr__cut_1_1EGPSRTM.html#a24cf60713c3b163b08e997e0f2b1bbd6",
"classfollow__face__node_1_1FollowFaceNode.html#a4b9557bc0503f7e0462a6cb524492957",
"classfollow__person__node_1_1FollowPersonNode.html#a6d776fd06f1398e140015c3113ca0f54",
"classget__positon__3d_1_1SingleTracker.html#a68b15e189474ff6d2a6fbaa705d0bf89",
"classhric__commands_1_1HRICCommands.html",
"classikfast_1_1IkSolutionListBase.html#a9d862f550472c2fa15189946b12222bf",
"classmodel_1_1PCB.html#a265ee02a5e708aedb25db1f67b11cfd7",
"classnav__central_1_1Nav__Central.html#a26b333e672e7a3f2fd58f729d03d0c26",
"classnav__ui_1_1NavRosNode.html#a50f62b02cbbb850ca72e03bc1eee8b23",
"classobject__detector__node_1_1object__detector__node.html#a0e54ab06ed56e351f5e28cb9b2e8c8bc",
"classpick__and__place_1_1keyboard__input_1_1KeyboardInput.html#a97bdeacbcc8414b61b677987db11fd94",
"classpick__and__place_1_1pour__server_1_1PourMotionServer.html#a8eee0a2e4bc81abcd7c54d019a9a16ee",
"classrestaurant__commands_1_1RESTAURANTCommands.html#a0933c5ddd9c8f08657c37a4550057bf7",
"classtask__manager_1_1subtask__managers_1_1gpsr__single__tasks_1_1GPSRSingleTask.html#abcd3e21f5d1605c471d3fca9b8e571ab",
"classtask__manager_1_1subtask__managers_1_1manipulation__tasks_1_1ManipulationTasks.html#a3b092917d67211db5326fedf365fbfbe",
"classtask__manager_1_1subtask__managers_1_1vision__tasks_1_1VisionTasks.html#ade58c5ad3f192f9259a00aa0a5e8a4a6",
"classtest__navigation__manager_1_1TestNavigationManager.html#ac08ce76f7f834ccdfe608ceb805216b4",
"classutils_1_1CrossEntropyLabelSmooth.html#a986e36cf01db77cfb562757382145172",
"dir_2d3b31f5269641a65ad69fd5e63e7c5f.html",
"format_8py.html#af0893b47c7fbeda9b19ea7006e54f925",
"hric_8launch_8py.html#a6ba4e2587b8b923faea3b8b23fdb037d",
"mapping_8launch_8py.html#ac951e3077b2a262f96039858d7fa2b20",
"namespaceclean__table__task__manager.html",
"namespacefrida__constants_1_1manipulation__constants.html#a9b1b384133b9135e786071e6e8b1b441",
"namespacelaunch__nav.html#ae215736aa067f1294af81ef3dcafe6f9",
"namespacepose__detection.html#a4f1c0192ed6a117c5528d0e790d140d8",
"namespacetest__plan__parser.html",
"reid__model_8py.html",
"vision_2packages_2vision__general_2vision__general_2utils_2models_2swin_2utils_8py.html#a43a1e41e013cee2dc8a50743353feba8"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';