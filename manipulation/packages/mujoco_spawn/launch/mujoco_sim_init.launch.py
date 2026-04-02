from launch import LaunchDescription, LaunchContext
import os
import yaml
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource, load_python_launch_file_as_module
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, LogInfo
from arm_pkg.moveit_configs_builder_sim import MoveItConfigsBuilder

def generate_nodes_for_spawn(context: LaunchContext):
    ##Arguments for xacro and moveit:
    prefix = LaunchConfiguration("prefix", default="")
    hw_ns = LaunchConfiguration("hw_ns", default="xarm")
    mesh_suffix = LaunchConfiguration("mesh_suffix", default="stl")
    limited = LaunchConfiguration("limited", default=False)
    effort_control = LaunchConfiguration("effort_control", default=False)
    velocity_control = LaunchConfiguration("velocity_control", default=False)
    add_gripper = LaunchConfiguration("add_gripper", default=False)
    load_zed = LaunchConfiguration("load_zed", default=False)
    add_vacuum_gripper = LaunchConfiguration("add_vacuum_gripper", default=False)
    add_bio_gripper = LaunchConfiguration("add_bio_gripper", default=False)
    dof = LaunchConfiguration("dof", default=6)
    robot_type = LaunchConfiguration("robot_type", default="xarm")
    ros2_control_plugin = LaunchConfiguration(
        "ros2_control_plugin", default="mujoco_ros2_control/MujocoSystem"
    )
    gripper_yaml = LaunchConfiguration(
        "gripper_yaml",
        default=os.path.join(
            get_package_share_directory("frida_description"),
            "config",
            "custom_gripper_controllers.yaml",
        ),
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    attach_to = LaunchConfiguration("attach_to", default="xarm_base")

    add_realsense_d435i = LaunchConfiguration("add_realsense_d435i", default=False)
    add_d435i_links = LaunchConfiguration("add_d435i_links", default=False)
    model1300 = LaunchConfiguration("model1300", default=False)
    robot_sn = LaunchConfiguration("robot_sn", default="")
    attach_xyz = LaunchConfiguration("attach_xyz", default='"0 0 0"')
    attach_rpy = LaunchConfiguration("attach_rpy", default='"0 0 0"')
    mujoco_plugin = LaunchConfiguration("mujoco_plugin", default=True)

    add_other_geometry = LaunchConfiguration("add_other_geometry", default=False)
    geometry_type = LaunchConfiguration("geometry_type", default="box")
    geometry_mass = LaunchConfiguration("geometry_mass", default=0.1)
    geometry_height = LaunchConfiguration("geometry_height", default=0.1)
    geometry_radius = LaunchConfiguration("geometry_radius", default=0.1)
    geometry_length = LaunchConfiguration("geometry_length", default=0.1)
    geometry_width = LaunchConfiguration("geometry_width", default=0.1)
    geometry_mesh_filename = LaunchConfiguration("geometry_mesh_filename", default="")
    geometry_mesh_origin_xyz = LaunchConfiguration(
        "geometry_mesh_origin_xyz", default='"0 0 0"'
    )
    geometry_mesh_origin_rpy = LaunchConfiguration(
        "geometry_mesh_origin_rpy", default='"0 0 0"'
    )
    geometry_mesh_tcp_xyz = LaunchConfiguration(
        "geometry_mesh_tcp_xyz", default='"0 0 0"'
    )
    geometry_mesh_tcp_rpy = LaunchConfiguration(
        "geometry_mesh_tcp_rpy", default='"0 0 0"'
    )
    kinematics_suffix = LaunchConfiguration("kinematics_suffix", default="")

    load_controller = LaunchConfiguration("load_controller", default=True)
    show_rviz = LaunchConfiguration("show_rviz", default=True)
    no_gui_ctrl = LaunchConfiguration("no_gui_ctrl", default=False)

    xarm_type = "{}{}".format(
        robot_type.perform(context),
        dof.perform(context) if robot_type.perform(context) in ("xarm", "lite") else "",
    )
    print(xarm_type)

    ros_namespace = LaunchConfiguration("ros_namespace", default="").perform(context)

        ##place to save the xml generated file:
    save_xml_folder = "/tmp/mujoco"  # Must be absolute path
    os.makedirs(save_xml_folder, exist_ok=True)
    save_xml_file = os.path.join(save_xml_folder, "main.xml")

    # Start of opperation--
    # ros2 control params 
    # xarm_controller/launch/lib/robot_controller_lib.py
    mod = load_python_launch_file_as_module(
        os.path.join(
            get_package_share_directory("xarm_controller"),
            "launch",
            "lib",
            "robot_controller_lib.py",
        )
    )
    generate_ros2_control_params_temp_file = getattr(
        mod, "generate_ros2_control_params_temp_file"
    )
    ros2_xarm_params_file = generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory("xarm_controller"),
            "config",
            "{}_controllers.yaml".format(xarm_type),
        ),
        prefix=prefix.perform(context),
        add_gripper=add_gripper.perform(context) in ("True", "true"),
        add_bio_gripper=add_bio_gripper.perform(context) in ("True", "true"),
        ros_namespace=ros_namespace,
        update_rate=500,
        robot_type=robot_type.perform(context),
    )

    ##Union of the yaml files for less problems
    xarm_params = yaml_loader(ros2_xarm_params_file)
    gripper_params = yaml_loader(gripper_yaml.perform(context))
    ros2_control_raw_params = deep_update(xarm_params, gripper_params)
    yaml_dump("/tmp/final_frida.yaml", ros2_control_raw_params)

    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    mod = load_python_launch_file_as_module(
        os.path.join(
            get_package_share_directory("xarm_description"),
            "launch",
            "lib",
            "robot_description_lib.py",
        )
    )

    moveit_config = MoveItConfigsBuilder(
        context=context,
        # controllers_name=controllers_name, # Que carajo?
        dof=dof,    
        controllers_name = "fake_controllers",
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300=model1300,
        robot_sn=robot_sn,
        attach_to=attach_to,
        attach_xyz=attach_xyz,
        attach_rpy=attach_rpy,
        mesh_suffix=mesh_suffix,
        kinematics_suffix=kinematics_suffix,
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params="/tmp/final_frida.yaml",
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        add_realsense_d435i=add_realsense_d435i,
        add_d435i_links=add_d435i_links,
        add_other_geometry=add_other_geometry,
        geometry_type=geometry_type,
        geometry_mass=geometry_mass,
        geometry_height=geometry_height,
        geometry_radius=geometry_radius,
        geometry_length=geometry_length,
        geometry_width=geometry_width,
        geometry_mesh_filename=geometry_mesh_filename,
        geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
    ).to_moveit_configs()


    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_pkg"),
                    "launch",
                    "frida_moveit_common.launch.py",
                ]
            )
        ),
        launch_arguments={
            "prefix": prefix,
            "attach_xyz": attach_xyz,
            "attach_rpy": attach_rpy,
            "no_gui_ctrl": no_gui_ctrl,
            "show_rviz": show_rviz,
            "use_sim_time": use_sim_time,
            "moveit_config_dump": yaml.dump(moveit_config.to_dict())
        }.items(),
    )


    additional_files = []
    additional_files.append(
        os.path.join(
            get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml"
        )
    )
    additional_files.append(
        os.path.join(
            get_package_share_directory("frida_description"),
            "urdf",
            "TMR2025",
            "house.xacro",
        )
    )
    robot_description = moveit_config.robot_description
    # Define the xacro2mjcf node, this translates the xacro urdf into MJFL
    # print(robot_description['robot_description'])
    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[
            {
                "robot_descriptions": [robot_description['robot_description']]
            },  # Robot descriptions of actuated robots
            {
                "input_files": additional_files
            },  
            {"output_file": save_xml_file},  # Mujoco output file
            {"mujoco_files_path": save_xml_folder},  # Mujoco project folder
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[use_sim_time, robot_description],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            "/tmp/final_frida.yaml",
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": save_xml_file},
            {"show_gui": True},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    ##Start simulation only after the creation of the xacrofile
    start_mujoco = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro2mjcf,
            on_exit=[
                LogInfo(msg="Created mujoco xml, starting mujoco node..."),
                mujoco,
            ],
        )
    )

    # Load controllers

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "{}/controller_manager".format(ros_namespace),
        ],
    )


    controllers = ["{}{}_traj_controller".format(prefix.perform(context), xarm_type), "xarm_gripper_traj_controller"]
    if (
        add_gripper.perform(context) in ("True", "true")
        and robot_type.perform(context) != "lite"
    ):
        controllers.append(
            "{}{}_gripper_traj_controller".format(
                prefix.perform(context), robot_type.perform(context)
            )
        )
    elif (
        add_bio_gripper.perform(context) in ("True", "true")
        and robot_type.perform(context) != "lite"
    ):
        controllers.append(
            "{}bio_gripper_traj_controller".format(prefix.perform(context))
        )


    # Load controllers
    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    controller,
                    "--controller-manager",
                    "{}/controller_manager".format(ros_namespace),
                ],
            )
        )

    load_gripper_bridge = Node(
        package="mujoco_spawn", executable="Xarm_gripper_mujoco_bridge", output="screen"
    )

    ##load after mujoco start
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting controllers..."),
                *controller_nodes,
                load_gripper_bridge,
                joint_state_broadcaster
                # motion_planning_server
            ],
        )
    )

    return [robot_state_publisher, xacro2mjcf, start_mujoco, load_controllers, joint_state_publisher_node, robot_moveit_common_launch, ros2_control_launch]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(
                function=generate_nodes_for_spawn
            )  # Use OpaqueFunction for node creation. For more information refere to the mujoco_ros2_controll package
        ]
    )


def yaml_loader(filepath):
    # Loads a yaml file
    with open(filepath, "r") as file_descriptor:
        data = yaml.load(file_descriptor, Loader=yaml.SafeLoader)
    return data


def yaml_dump(filepath, data):
    # writtes a yaml file
    with open(filepath, "w") as file_descriptor:
        yaml.dump(data, file_descriptor)


##Joints all their values into a single yaml file
def deep_update(source, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and key in source and isinstance(source[key], dict):
            source[key] = deep_update(source[key], value)
        else:
            source[key] = value
    return source
