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
    [ "Deep SORT — Multi-Object Tracking", "md_vision_packages_vision_general_vision_general_utils_deep_sort_readme.html", [
      [ "Origin", "md_vision_packages_vision_general_vision_general_utils_deep_sort_readme.html#autotoc_md10", null ],
      [ "Files", "md_vision_packages_vision_general_vision_general_utils_deep_sort_readme.html#autotoc_md11", null ],
      [ "Files NOT included from the original repo", "md_vision_packages_vision_general_vision_general_utils_deep_sort_readme.html#autotoc_md12", null ],
      [ "How tracking works (high-level)", "md_vision_packages_vision_general_vision_general_utils_deep_sort_readme.html#autotoc_md13", null ]
    ] ],
    [ "README", "md_vision_README.html", null ],
    [ "README", "md_robot_description_frida_description_README.html", null ],
    [ "Place Params Special Request", "md_frida_interfaces_manipulation_msg_PlaceParamsSpecialRequest.html", [
      [ "Example of a special request", "md_frida_interfaces_manipulation_msg_PlaceParamsSpecialRequest.html#autotoc_md15", null ]
    ] ],
    [ "Names", "md_frida_constants_data_names.html", null ],
    [ "Class drinks (drink)", "md_frida_constants_data_objects.html", [
      [ "Class fruits (fruit)", "md_frida_constants_data_objects.html#autotoc_md18", null ],
      [ "Class snacks (snack)", "md_frida_constants_data_objects.html#autotoc_md19", null ],
      [ "Class foods (food)", "md_frida_constants_data_objects.html#autotoc_md20", null ],
      [ "Class dishes (dish)", "md_frida_constants_data_objects.html#autotoc_md21", null ],
      [ "Class cleaning_supplies (cleaning_supply)", "md_frida_constants_data_objects.html#autotoc_md22", null ]
    ] ],
    [ "CycloneDDS Setup", "md_docs_cyclonedds_setup.html", [
      [ "Architecture", "md_docs_cyclonedds_setup.html#autotoc_md24", null ],
      [ "Shared Memory (SHM) vs UDP", "md_docs_cyclonedds_setup.html#autotoc_md25", [
        [ "SHM Memory Breakdown (when enabled)", "md_docs_cyclonedds_setup.html#autotoc_md26", null ],
        [ "ZED SHM Workarounds", "md_docs_cyclonedds_setup.html#autotoc_md27", null ]
      ] ],
      [ "Files", "md_docs_cyclonedds_setup.html#autotoc_md28", null ],
      [ "Usage", "md_docs_cyclonedds_setup.html#autotoc_md29", [
        [ "Local Development (no SHM, default)", "md_docs_cyclonedds_setup.html#autotoc_md30", null ],
        [ "Orin AGX / High-Memory Machine (with SHM)", "md_docs_cyclonedds_setup.html#autotoc_md31", null ],
        [ "Bare Metal (Orin, direct install)", "md_docs_cyclonedds_setup.html#autotoc_md32", null ],
        [ "Revert to FastDDS", "md_docs_cyclonedds_setup.html#autotoc_md33", null ],
        [ "Override Interface at Runtime", "md_docs_cyclonedds_setup.html#autotoc_md34", null ]
      ] ],
      [ "What It Configures", "md_docs_cyclonedds_setup.html#autotoc_md35", [
        [ "CycloneDDS XML (<tt>/etc/cyclonedds.xml</tt>)", "md_docs_cyclonedds_setup.html#autotoc_md36", null ],
        [ "Kernel Tuning (<tt>/etc/sysctl.d/60-cyclonedds-buffers.conf</tt>)", "md_docs_cyclonedds_setup.html#autotoc_md37", null ],
        [ "Iceoryx / RouDi (when <tt>CYCLONE_SHM=1</tt>)", "md_docs_cyclonedds_setup.html#autotoc_md38", null ]
      ] ],
      [ "Finding Your Network Interface", "md_docs_cyclonedds_setup.html#autotoc_md39", null ],
      [ "Troubleshooting", "md_docs_cyclonedds_setup.html#autotoc_md40", [
        [ "RouDi SIGBUS / fails to start", "md_docs_cyclonedds_setup.html#autotoc_md41", null ],
        [ "Stale iceoryx artifacts", "md_docs_cyclonedds_setup.html#autotoc_md42", null ]
      ] ],
      [ "Reference", "md_docs_cyclonedds_setup.html#autotoc_md43", null ]
    ] ],
    [ "Expo demo Feb 13, 2025", "md_docs_expo_demo.html", [
      [ "Running HRI", "md_docs_expo_demo.html#autotoc_md45", null ]
    ] ],
    [ "frida_interfaces", "md_docs_interfaces.html", [
      [ "MoveJoints.action definition", "md_docs_interfaces.html#autotoc_md47", null ],
      [ "Other Messages", "md_docs_interfaces.html#autotoc_md49", [
        [ "SomeMessage.msg", "md_docs_interfaces.html#autotoc_md50", null ],
        [ "SomeService.srv", "md_docs_interfaces.html#autotoc_md51", null ]
      ] ]
    ] ],
    [ "HRI", "md_docs_Run_Areas_hri.html", [
      [ "Flags", "md_docs_Run_Areas_hri.html#autotoc_md67", null ],
      [ "Running specific containers", "md_docs_Run_Areas_hri.html#autotoc_md68", null ]
    ] ],
    [ "Manipulation", "md_docs_Run_Areas_Manipulation_manipulation.html", [
      [ "Docker setup", "md_docs_Run_Areas_Manipulation_manipulation.html#autotoc_md70", null ],
      [ "Running the vision module", "md_docs_Run_Areas_Manipulation_manipulation.html#autotoc_md71", [
        [ "Additional Information", "md_docs_Run_Areas_Manipulation_manipulation.html#autotoc_md78", null ]
      ] ]
    ] ],
    [ "Running Pick and Place", "md_docs_Run_Areas_Manipulation_pick_and_place.html", [
      [ "Launching the Robot", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md80", [
        [ "Simulation", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md81", null ],
        [ "Real RObot", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md82", [
          [ "Robot interface", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md83", null ],
          [ "3D Camera", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md84", null ]
        ] ]
      ] ],
      [ "Launching utilities", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md85", [
        [ "Object detector", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md86", null ]
      ] ],
      [ "Launch Pick and Place main code", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md87", [
        [ "Pick and Place pipeline", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md88", null ]
      ] ],
      [ "Usage", "md_docs_Run_Areas_Manipulation_pick_and_place.html#autotoc_md89", null ]
    ] ],
    [ "Navigation Docs", "md_docs_Run_Areas_nav.html", [
      [ "How to Run", "md_docs_Run_Areas_nav.html#autotoc_md91", null ],
      [ "Running Nav Basics", "md_docs_Run_Areas_nav.html#autotoc_md93", [
        [ "Arguments", "md_docs_Run_Areas_nav.html#autotoc_md94", null ],
        [ "Run Command", "md_docs_Run_Areas_nav.html#autotoc_md95", null ]
      ] ],
      [ "Running AMCL Localization", "md_docs_Run_Areas_nav.html#autotoc_md97", [
        [ "Arguments", "md_docs_Run_Areas_nav.html#autotoc_md98", null ],
        [ "Run Command", "md_docs_Run_Areas_nav.html#autotoc_md99", null ]
      ] ],
      [ "Running Navigation Node", "md_docs_Run_Areas_nav.html#autotoc_md101", [
        [ "Arguments", "md_docs_Run_Areas_nav.html#autotoc_md102", null ],
        [ "Run Command", "md_docs_Run_Areas_nav.html#autotoc_md103", null ]
      ] ]
    ] ],
    [ "Vision", "md_docs_Run_Areas_vision.html", [
      [ "Docker setup", "md_docs_Run_Areas_vision.html#autotoc_md105", null ],
      [ "Running the vision module", "md_docs_Run_Areas_vision.html#autotoc_md106", null ],
      [ "Structure", "md_docs_Run_Areas_vision.html#autotoc_md109", null ],
      [ "Camera", "md_docs_Run_Areas_vision.html#autotoc_md110", null ]
    ] ],
    [ "Help me carry", "md_docs_Run_Tasks_help_me_carry.html", [
      [ "Integration", "md_docs_Run_Tasks_help_me_carry.html#autotoc_md112", null ],
      [ "Vision", "md_docs_Run_Tasks_help_me_carry.html#autotoc_md113", null ]
    ] ],
    [ "Receptionist", "md_docs_Run_Tasks_receptionist.html", [
      [ "ZED", "md_docs_Run_Tasks_receptionist.html#autotoc_md115", null ],
      [ "Integration", "md_docs_Run_Tasks_receptionist.html#autotoc_md116", null ],
      [ "Vision", "md_docs_Run_Tasks_receptionist.html#autotoc_md117", null ],
      [ "HRI", "md_docs_Run_Tasks_receptionist.html#autotoc_md118", null ],
      [ "Navigation", "md_docs_Run_Tasks_receptionist.html#autotoc_md119", null ],
      [ "Manipulation", "md_docs_Run_Tasks_receptionist.html#autotoc_md120", [
        [ "Launch", "md_docs_Run_Tasks_receptionist.html#autotoc_md121", null ],
        [ "Arm bringup", "md_docs_Run_Tasks_receptionist.html#autotoc_md122", null ],
        [ "Motion Planning", "md_docs_Run_Tasks_receptionist.html#autotoc_md123", null ],
        [ "Follow face", "md_docs_Run_Tasks_receptionist.html#autotoc_md124", null ]
      ] ]
    ] ],
    [ "Storing Groceries", "md_docs_Run_Tasks_storing_groceries.html", [
      [ "Integration", "md_docs_Run_Tasks_storing_groceries.html#autotoc_md126", null ],
      [ "Vision", "md_docs_Run_Tasks_storing_groceries.html#autotoc_md127", null ]
    ] ],
    [ "Improving your ROS2 workflow with vscode", "md_docs_setup_vscode.html", [
      [ "Prerequisites", "md_docs_setup_vscode.html#autotoc_md129", null ],
      [ "Setup", "md_docs_setup_vscode.html#autotoc_md130", null ],
      [ "Troubleshooting", "md_docs_setup_vscode.html#autotoc_md131", null ]
    ] ],
    [ "Project setup", "md_docs_Setup.html", [
      [ "Pre-commit", "md_docs_Setup.html#autotoc_md133", null ],
      [ "Ruff", "md_docs_Setup.html#autotoc_md134", [
        [ "Vscode integration", "md_docs_Setup.html#autotoc_md135", null ],
        [ "Other integrations", "md_docs_Setup.html#autotoc_md136", null ]
      ] ]
    ] ],
    [ "Decorators", "md_docs_task_manager_decorators.html", [
      [ "Mockable", "md_docs_task_manager_decorators.html#autotoc_md138", [
        [ "Parameters", "md_docs_task_manager_decorators.html#autotoc_md139", null ],
        [ "Implementation", "md_docs_task_manager_decorators.html#autotoc_md140", null ]
      ] ],
      [ "Service Check", "md_docs_task_manager_decorators.html#autotoc_md141", [
        [ "Parameters", "md_docs_task_manager_decorators.html#autotoc_md142", null ],
        [ "Implementation", "md_docs_task_manager_decorators.html#autotoc_md143", null ]
      ] ],
      [ "Example", "md_docs_task_manager_decorators.html#autotoc_md144", null ]
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
"classIKSolver.html#a6906e8cdc998ef02a6794b2ab68a40d8",
"classPlanner_1_1Planner.html#a2834bbbc877ce26d596d673da9ab5ff1",
"class__storing__groceries_1_1StoringGroceriesTM.html#adf1056ec281c89a1b19035fdac9a1f87",
"classcall__pose__goal_1_1MoveToPoseClient.html",
"classdashgo__driver_1_1dashgo__stm32_1_1Stm32.html#a5b7f56770f5bb05cb5e7456953c0dff7",
"classegsr__cut_1_1EGPSRTM.html#a24cf60713c3b163b08e997e0f2b1bbd6",
"classfollow__face__node_1_1FollowFaceNode.html#a4b9557bc0503f7e0462a6cb524492957",
"classfollow__person__node_1_1FollowPersonNode.html#a81ff55fb43f6b7dec58b0825737686f1",
"classget__positon__3d_1_1SingleTracker.html#a87269ddb7a0ebc5841e3fc1bb5228165",
"classhric__commands_1_1HRICCommands.html#a0af5053438a9e8605f7b8b9a694dba23",
"classikfast_1_1IkSolutionListBase.html#a9940fe21cfa0a67c1a21a672696c5cb5",
"classmap__area__tagger_1_1MapCanvas.html#aa64b1ae4116b414b97b3a691f4f215fa",
"classmoondream__proto__pb2__grpc_1_1MoonDreamServiceStub.html#a29d869563ec9ade68e283407fde63863",
"classnav__ui_1_1NavCanvas.html#ae612389c001624be3282491b6b11c834",
"classnew__tracker_1_1SingleTracker.html#af824242cdcaac60ee26669d7525fe5b7",
"classpick__and__place_1_1fix__position__to__plane_1_1FixPositionToPlane.html#a157300fb27199bd9a4b1d6582e706e5e",
"classpick__and__place_1_1place__server_1_1PlaceMotionServer.html",
"classreceptionist__task__manager_1_1ReceptionistTM.html#a983e79d3cff0aa4d10cd4f3949a3a585",
"classstoring__groceries__manager_1_1StoringGroceriesManager.html#a3fe8fd87e732376f4836d2bcc3ab29f8",
"classtask__manager_1_1subtask__managers_1_1hri__tasks_1_1HRITasks.html#a87668b69a8ebede6cf19c280a2e61abd",
"classtask__manager_1_1subtask__managers_1_1vision__tasks_1_1VisionTasks.html#a6dde6d86b849ef211eb07bf8bd632398",
"classtemp__follow_1_1FollowPersonNode.html#aa385c83f41b834151c4d87e439e69d17",
"classtracker__node_1_1SingleTracker.html#a34293ecadfff8078e9520a300998c6ef",
"classwait__for__controller__input__node_1_1WaitForControllerInputNode.html#abc28943a6d0e028abddbdb7188846041",
"dir_7fd0cf9ff5e572a6c00dd1677e1d80bb.html",
"functions_func_t.html",
"kalman__filter_8py.html#a5c1eb7c1bf433282be1265e5e74ccb45",
"md_docs_Run_Areas_vision.html#autotoc_md109",
"namespacedemo__face.html",
"namespacefrida__constants_1_1manipulation__constants.html#afb44d7214e7e343d9f0c7162e2550bc8",
"namespacelook__at__example.html#adc1b4dbd73f98a092d682f946ed57ef3",
"namespacepose__detection.html#a789afef581d6f1cdcc8ea7604d848cee",
"namespacetest__plan__parser.html#a7c320a6064d3055f6213924d2437839a",
"pose__detection_8py.html#ad8f828751d16823638c4ea381064fecb",
"vision_2packages_2object__detector__2d_2object__detector__2d_2Utils_2ros__utils_8py.html"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';