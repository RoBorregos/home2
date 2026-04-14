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
"classIKSolver.html#a791e2437d1fc1b568b213b0388f97d0a",
"classPlanner_1_1Planner.html#a471bc0e65bd0f91717e593fc59e5b248",
"classarm__pkg_1_1moveit__configs__builder_1_1DualMoveItConfigsBuilder.html",
"classcustomer__node_1_1CustomerNode.html#a99632b01300498756596ea2697bb528b",
"classdashgo__driver_1_1dashgo__stm32_1_1Stm32.html#af25a5e0a436e53961e3118e256237711",
"classexample__node_1_1ExampleNode.html#a20b72480091f1d455574afb4818a13d5",
"classfollow__face__node__copt_1_1FollowFaceNode.html#a876df1c7ea1a927bad3ff40a6a6dce42",
"classfollow__person__v2_1_1FollowFaceNode.html#a68c13e3c007dff8edbf40b6c575781a1",
"classgpsr__commands_1_1GPSRCommands.html#ad0a01a65f3853e0eb55897d9c44b0aee",
"classhric__task__manager_1_1Guest.html#a54685526f5ef5784020f4282c264f247",
"classlocal__camera_1_1Camera.html#ac6bbc92436bc9b4c2a56ee3f59088637",
"classmodel_1_1ft__net__convnext.html#a321aa0466e261698aee6ba95670f4b70",
"classnav__central_1_1Nav__Central.html#a813ac5dee3cb6d1347165519fdaf6fb6",
"classnav__ui_1_1NavUI.html#a0b19f4f2b12a747f3ea26f8cb038733c",
"classobject__detector__node_1_1object__detector__node.html#a82aac9fa4e1310b8c9f9e52d03e8064f",
"classpick__and__place_1_1manipulation__client_1_1ManipulationClient.html#ab91e8114592e7aee58c08529e6027b4e",
"classpoint__transformer_1_1PointTransformer.html#a351223e0eeb1b9879f2e733ed027f7cb",
"classrestaurant__commands_1_1RESTAURANTCommands.html#ae38426dee2fa09684e4cedcba0de4db5",
"classtask__manager_1_1subtask__managers_1_1gpsr__tasks_1_1GPSRTask.html#a88aede36b7bce4f0903eccd043198982",
"classtask__manager_1_1subtask__managers_1_1manipulation__tasks_1_1ManipulationTasks.html#a62ddf96c70e22e422fba1bf781cb58a0",
"classtask__manager_1_1subtask__managers_1_1vision__tasks_1_1VisionTasks.html#af2feee7ffb0e49ff71fe244249cdd63f",
"classtest__plan__parser_1_1TestPlanParser.html#a84f6e4ff2ddbc6d185f664520857d448",
"classtracker__node__fregoso_1_1SingleTracker.html#a975c885b9545400ac3b703d0bdac8e50",
"classzero__shot__object__detector__node_1_1zero__shot__object__detector__node.html#a96ccd62786a70880b4ce090739066438",
"face__recognition__node_8py.html",
"grasp__detection__example_8py.html#a40a8024a9f168af4327222571de3ea94",
"manipulation__constants_8py.html#ab75f80813165c558161e6506743bc531",
"moondream__node_8py.html#aa2e1e7b7b490eadcc4fe5414c36631b1",
"namespacefollow__person__v2.html#ae18fc04cf34a4b66e7bb2a60c4308a89",
"namespacefrida__motion__planning_1_1motion__planning__server.html#af7e4f543cce624c9891b51d04ce1ee68",
"namespacemotion__planning__server.html",
"namespacetask__manager_1_1subtask__managers_1_1gpsr__test__commands.html#a3c121f73e0ad4f5793fc7db2ba6ad3d8",
"navigation__constants_8py.html#a0c539f7d23dbcc6c9137ec5379fb528f",
"task__manager_2task__manager_2config_2____init_____8py.html",
"xarm6__ikfast61_8cpp.html#a14ff3dec1d7bf1b6f8bc3a7dcc0ed883"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';