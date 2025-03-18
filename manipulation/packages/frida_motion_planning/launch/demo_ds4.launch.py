from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # command_interpreter_config = os.path.join(
    #     get_package_share_directory(
    #         "nlp"), "config", "command_interpreter.yaml"
    # )

    robot_ip = LaunchConfiguration("robot_ip")

    # robot moveit realmove launch
    # xarm_moveit_config/launch/_robot_moveit_realmove.launch.py
    robot_moveit_realmove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("xarm_moveit_config"),
                    "launch",
                    "_robot_moveit_realmove.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "dof": "6",
            "robot_type": "xarm",
            "hw_ns": "xarm",
            "no_gui_ctrl": "false",
        }.items(),
    )

    motion_planning_launch = Node(
        package="frida_motion_planning",
        executable="motion_planning_server.py",
        name="motion_planning_server",
    )

    demo_ds4_launch = Node(
        package="frida_motion_planning", executable="ds4_demo.py", name="ds4_demo"
    )

    return LaunchDescription(
        [robot_moveit_realmove_launch, motion_planning_launch, demo_ds4_launch]
    )
