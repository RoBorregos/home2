#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "grasp_backend",
                default_value="gpd",
                description="Grasp detection backend: 'gpd' or 'contact_graspnet'.",
            ),
            DeclareLaunchArgument(
                "ckpt_dir",
                default_value="checkpoints/contact_graspnet",
                description="Contact-GraspNet checkpoint dir (used when grasp_backend=contact_graspnet).",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("arm_pkg"),
                            "launch",
                            "frida_moveit_config.launch.py",
                        ]
                    )
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("pick_and_place"),
                            "launch",
                            "pick_and_place.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "grasp_backend": LaunchConfiguration("grasp_backend"),
                    "ckpt_dir": LaunchConfiguration("ckpt_dir"),
                }.items(),
            ),
            Node(
                package="task_manager",
                executable="follow_face_node.py",
                name="follow_face_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
