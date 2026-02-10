from launch import LaunchDescription, LaunchContext
import os
import yaml
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import OpaqueFunction, RegisterEventHandler, LogInfo

def generate_nodes_for_spawn(context: LaunchContext):
    ##Arguments for xacro:
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=True)
    velocity_control = LaunchConfiguration('velocity_control', default=True)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='mujoco_ros2_control/MujocoSystem')
    
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='base_link')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    
    load_controller = LaunchConfiguration('load_controller', default=False)
    show_rviz = LaunchConfiguration('show_rviz', default=False)
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    
    
    ##place to save the xml generated file:
    save_xml_folder = "/tmp/mujoco"                               # Must be absolute path
    save_xml_file = os.path.join(save_xml_folder, "main.xml") 
    
    #creation of the robot description string 
    # ros2 control params
    # xarm_controller/launch/lib/robot_controller_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lib', 'robot_controller_lib.py'))
    generate_ros2_control_params_temp_file = getattr(mod, 'generate_ros2_control_params_temp_file')
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type)),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        update_rate=1000,
        robot_type=robot_type.perform(context)
    )
    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'robot_description_lib.py'))
    get_xacro_file_content = getattr(mod, 'get_xacro_file_content')
    robot_description = {
        'robot_description': get_xacro_file_content(
            xacro_file=PathJoinSubstitution([FindPackageShare('frida_description'), 'urdf', 'TMR2025','FRIDA.urdf.xacro']), 
            arguments={
                'prefix': prefix,
                'dof': dof,
                'robot_type': robot_type,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'add_bio_gripper': add_bio_gripper,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'ros2_control_plugin': ros2_control_plugin,
                'ros2_control_params': ros2_control_params,
                'add_realsense_d435i': add_realsense_d435i,
                'add_d435i_links': add_d435i_links,
                'model1300': model1300,
                'robot_sn': robot_sn,
                'attach_to': attach_to,
                'attach_xyz': attach_xyz,
                'attach_rpy': attach_rpy,
                'add_other_geometry': add_other_geometry,
                'geometry_type': geometry_type,
                'geometry_mass': geometry_mass,
                'geometry_height': geometry_height,
                'geometry_radius': geometry_radius,
                'geometry_length': geometry_length,
                'geometry_width': geometry_width,
                'geometry_mesh_filename': geometry_mesh_filename,
                'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
                'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
                'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
                'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
                'kinematics_suffix': kinematics_suffix,
            }
        ),
    }
    
    ##Props to be loaded:
    additional_files = []
    additional_files.append(os.path.join(get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml"))
    additional_files.append(os.path.join(get_package_share_directory("task_table_mujoco"), "urdf", "task_table.urdf.xacro"))
    
    # as this node require a string array
    robot_description_string = robot_description['robot_description'].perform(context)
    # Define the xacro2mjcf node, this translates the xacro urdf into MJFL
    xacro2mjcf = Node(
    package="mujoco_ros2_control",
    executable="xacro2mjcf.py",
    parameters=[
            {"robot_descriptions": [robot_description_string]}, # Robot descriptions of actuated robots
            {"input_files": additional_files},        # Files that are added to mujoco but not to ros2_control
            {"output_file": save_xml_file},       # Mujoco output file
            {"mujoco_files_path": save_xml_folder}, # Mujoco project folder
            # Floating base related params:
            {"floating": True},
            {"base_link": "base_link"}, 
            {"initial_position": "0 0 1.05"},
            {"initial_orientation": "0 0 0"}
        ]
    )
    
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            ros2_control_params,
            {"simulation_frequency": 1000.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": save_xml_file},
            {"show_gui": True},
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )
    
    ##Start simulation only after the creation of the xacrofile
    start_mujoco = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro2mjcf,
            on_exit=[
                LogInfo(msg="Created mujoco xml, starting mujoco node..."),
                mujoco
            ],
        )
    )
    
    load_joint_state_broadcaster = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_state_broadcaster",
        "--controller-manager",
        ["/", "controller_manager"],
        ],
    )
    
    ##load after mujoco start
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting joint state broadcaster..."),
                load_joint_state_broadcaster
            ],
        )
    )
    return [
        robot_state_publisher,
        xacro2mjcf,
        start_mujoco,
        load_controllers
    ]
    

def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=generate_nodes_for_spawn)  # Use OpaqueFunction for node creation. For more information refere to the mujoco_ros2_controll package
    ])