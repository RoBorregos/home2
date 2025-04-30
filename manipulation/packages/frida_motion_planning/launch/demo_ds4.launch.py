from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # command_interpreter_config = os.path.join(
    #     get_package_share_directory(
    #         "nlp"), "config", "command_interpreter.yaml"
    # )

    # robot moveit realmove launch
    # xarm_moveit_config/launch/_robot_moveit_realmove.launch.py
    # robot_moveit_realmove_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("arm_pkg"),
    #                 "launch",
    #                 "frida_moveit_config.launch.py",
    #             ]
    #         )
    #     )
    # )

    motion_planning_launch = Node(
        package="frida_motion_planning",
        executable="motion_planning_server.py",
        name="motion_planning_server",
    )

    demo_ds4_launch = Node(
        package="frida_motion_planning", executable="ds4_demo.py", name="ds4_demo"
    )

    return LaunchDescription([motion_planning_launch, demo_ds4_launch])
