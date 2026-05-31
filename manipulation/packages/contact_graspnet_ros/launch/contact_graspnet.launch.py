#!/usr/bin/env python3
"""Standalone launch for the Contact-GraspNet node (for debugging/testing)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def cgn_node(context, *args, **kwargs):
        return [
            Node(
                package="contact_graspnet_ros",
                executable="contact_graspnet_node",
                name="contact_graspnet_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "ckpt_dir": LaunchConfiguration("ckpt_dir").perform(context),
                        "forward_passes": int(
                            LaunchConfiguration("forward_passes").perform(context)
                        ),
                        "z_range": [0.2, 1.8],
                        "pick_min_height": float(
                            LaunchConfiguration("pick_min_height").perform(context)
                        ),
                        "max_input_points": int(
                            LaunchConfiguration("max_input_points").perform(context)
                        ),
                        "use_torch_compile": LaunchConfiguration("use_torch_compile")
                        .perform(context)
                        .lower()
                        == "true",
                    }
                ],
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ckpt_dir",
                default_value="checkpoints/contact_graspnet",
                description="Path to checkpoint dir (absolute or relative to submodule root).",
            ),
            DeclareLaunchArgument(
                "forward_passes",
                default_value="1",
                description="Number of forward passes (more = more grasps, slower).",
            ),
            DeclareLaunchArgument(
                "pick_min_height",
                default_value="0.03",
                description="Min height above cluster bottom a grasp must clear (m).",
            ),
            DeclareLaunchArgument(
                "max_input_points",
                default_value="1024",
                description=(
                    "Cap on input cloud size before CGN inference. "
                    "CGN internally FPS-reduces to ~1024 anyway. "
                    "Must match warm-up size for torch.compile graph cache correctness."
                ),
            ),
            DeclareLaunchArgument(
                "use_torch_compile",
                default_value="true",
                description=(
                    "Enable torch.compile(mode='reduce-overhead') on the GraspNet model. "
                    "Adds ~30-60 s to startup on Jetson Orin (one-time JIT compilation). "
                    "Set to false for faster restarts during development."
                ),
            ),
            OpaqueFunction(function=cgn_node),
        ]
    )
