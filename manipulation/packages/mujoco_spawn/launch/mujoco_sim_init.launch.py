from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
import os
import xacro
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def generate_nodes_for_spawn(context: LaunchContext):
    ##Arguments
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    prefix = LaunchConfiguration('prefix', default='')
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    dof = LaunchConfiguration('dof', default=7)
    
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    
    
    ##place to save the xml generated file:
    save_xml_folder = "/tmp/mujoco"                               # Must be absolute path
    save_xml_file = os.path.join(save_xml_folder, "main.xml") 
    
    #creation of the robot description string
    robot_xacro_path = os.path.join(get_package_share_directory("frida_description"), "urdf","TMR2025","FRIDA_Real.urdf.xacro")
    robot_description = {
    'robot_description': xacro.process_file(
        robot_xacro_path,
    ).toprettyxml(indent="  ")
    }
    additional_files =[]
    additional_files.append(os.path.join(get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml"))
    
    #define the ros2 control params
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}{}_controllers.yaml'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        update_rate=1000,
        robot_type=robot_type.perform(context)
    )
    
    
    # Define the xacro2mjcf node, this manages all the ros2 control
    xacro2mjcf = Node(
    package="mujoco_ros2_control",
    executable="xacro2mjcf.py",
    parameters=[
            {"robot_descriptions": [robot_description["robot_description"]]}, # Robot descriptions of actuated robots
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