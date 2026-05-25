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
            OpaqueFunction(function=cgn_node),
        ]
    )
