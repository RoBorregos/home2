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
"classIKSolver.html#a66a0610cba0da00e09deda3c72e3909f",
"classPlanner_1_1Planner.html#a275192f33240079f6d2c906f594c68cb",
"class__storing__groceries_1_1StoringGroceriesTM.html#add47d7edd6f6616ad6552d9717de612e",
"classcustomer__node_1_1CustomerNode.html#a7cb81a4782fcd1393fa353c4ea3b10e3",
"classdashgo__driver_1_1dashgo__stm32_1_1Stm32.html#aed55ad06bc9d7097797c122ef8c1af23",
"classexample__node_1_1ExampleNode.html",
"classfollow__face__node__copt_1_1FollowFaceNode.html#a4fb334b43cc465919ca25cc96012ff9d",
"classfollow__person__v2_1_1FollowFaceNode.html#a1d7325ec9fafccdae4f4b0bbc93db65f",
"classgpsr__commands_1_1GPSRCommands.html#a928bee0668cea771ab24a95f6b34ec16",
"classhric__commands_1_1HRICCommands.html#ae71402bd97b55402d87fcaf46eee953e",
"classlocal__camera_1_1Camera.html#ac6bbc92436bc9b4c2a56ee3f59088637",
"classmodel_1_1ft__net__convnext.html#a321aa0466e261698aee6ba95670f4b70",
"classnav__central_1_1Nav__Central.html#a813ac5dee3cb6d1347165519fdaf6fb6",
"classnav__ui_1_1NavUI.html#a0b19f4f2b12a747f3ea26f8cb038733c",
"classobject__detector__node_1_1object__detector__node.html#a94379d5beaf6d663287330a991d956dd",
"classpick__and__place_1_1manipulation__client_1_1ManipulationClient.html#af17d7c3be2d94c561cba0af603846ee8",
"classpoint__transformer_1_1PointTransformer.html#a8004bd3f74182640c3bb273ffcdf4f52",
"classrestaurant__task__manager_1_1RestaurantTaskManager.html",
"classtask__manager_1_1subtask__managers_1_1gpsr__tasks_1_1GPSRTask.html#aabfac22f48b437b68fd474565ddf2a40",
"classtask__manager_1_1subtask__managers_1_1manipulation__tasks_1_1ManipulationTasks.html#a7d960421f2da5e97c3e5aba73b6bae0e",
"classtask__manager_1_1utils_1_1config_1_1TopicConfig.html",
"classtest__vision__manager_1_1TestVisionManager.html#a5d8c1bf8cd97f06cadbe9e0abd831dac",
"classtracker__node__fregoso_1_1SingleTracker.html#ad7efb1b4cd689a8548e71514956c71fe",
"classzero__shot__object__detector__node_1_1zero__shot__object__detector__node.html#aa22c80a7146ec313b9571bfee73c3df3",
"example__node_8py.html#a6eae482e85ac51b66ae90ecccdd460d3",
"gpsr__test__commands_8py.html#af61153ac2e4af84cfaf1fcc0eca7f0e8",
"manipulation__constants_8py.html#ab65c663bc0e5f75a060598e9e0c4a01a",
"moondream__node_8py.html#a10b02c02e345a65f36ac440ca0b90cc6",
"namespacefollow__person__v2.html#ac4ba8b37bda1500e4e6c2d1e6135bf78",
"namespacefrida__moveit__common.html#a4c04134c253d66c6e27201adf558e3b7",
"namespacemoveit.html#acbf7baa79e0c3776c713bd6f15b25ab6",
"namespacetask__manager_1_1subtask__managers_1_1gpsr__test__commands.html#a3c121f73e0ad4f5793fc7db2ba6ad3d8",
"nav__ui_8py.html#ac2d8604148bbc1e3a92e817195c78772",
"structBoxPrimitiveParams.html#a0fb8330765aa15d325fd2e0827d9661d",
"vision__tasks_8py.html#a49cfba4f063b64b58df1700773fd7995"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';