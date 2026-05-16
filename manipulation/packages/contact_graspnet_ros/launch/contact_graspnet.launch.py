from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ckpt_dir",
                default_value="checkpoints/contact_graspnet",
                description="Path to Contact-GraspNet checkpoint directory.",
            ),
            DeclareLaunchArgument(
                "forward_passes",
                default_value="1",
                description="Number of forward passes for grasp prediction.",
            ),
            Node(
                package="contact_graspnet_ros",
                executable="contact_graspnet_node",
                name="contact_graspnet_node",
                output="screen",
                parameters=[
                    {
                        "ckpt_dir": LaunchConfiguration("ckpt_dir"),
                        "forward_passes": LaunchConfiguration("forward_passes"),
                        "z_range": [0.2, 1.8],
                    }
                ],
            ),
        ]
    )
