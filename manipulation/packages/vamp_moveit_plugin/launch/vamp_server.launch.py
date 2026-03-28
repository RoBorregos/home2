
"""
Launch file for VAMP planning server.

Usage:
  ros2 launch vamp_moveit_plugin vamp_server.launch.py

This starts the Python VAMP server node that handles the actual
RRTConnect planning via the VAMP C++ engine. It must be running
before any MoveIt planning requests that use the VAMP pipeline.

Parameters are tunable at launch:
  ros2 launch vamp_moveit_plugin vamp_server.launch.py \
    max_iterations:=100000 \
    range:=0.03 \
    security_margin:=0.03
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments ────────────────────────────────────
        DeclareLaunchArgument("max_iterations", default_value="50000",
            description="Max RRT iterations per attempt"),
        DeclareLaunchArgument("range", default_value="0.05",
            description="RRT extension range in C-space"),
        DeclareLaunchArgument("security_margin", default_value="0.02",
            description="Extra collision margin (meters)"),
        DeclareLaunchArgument("max_retries", default_value="3",
            description="Number of planning retries with increasing params"),
        DeclareLaunchArgument("finger_left_default", default_value="0.8",
            description="Default left finger joint value"),
        DeclareLaunchArgument("finger_right_default", default_value="0.8",
            description="Default right finger joint value"),
        DeclareLaunchArgument("validation_step_size", default_value="0.05",
            description="Interpolation step for path validation (radians)"),
        DeclareLaunchArgument("smoothing_window", default_value="5",
            description="Moving average window for path smoothing"),
        DeclareLaunchArgument("smoothing_passes", default_value="3",
            description="Number of smoothing passes"),
        DeclareLaunchArgument("min_r_point", default_value="0.09",
            description="Minimum collision radius per voxel (compensates sphere model)"),
        DeclareLaunchArgument("self_filter_distance", default_value="0.12",
            description="Distance to filter robot body from pointcloud (meters)"),

        # ── VAMP Server Node ────────────────────────────────────
        Node(
            package="vamp_moveit_plugin",
            executable="vamp_server.py",
            name="vamp_server",
            output="screen",
            parameters=[{
                "max_iterations": LaunchConfiguration("max_iterations"),
                "range": LaunchConfiguration("range"),
                "security_margin": LaunchConfiguration("security_margin"),
                "max_retries": LaunchConfiguration("max_retries"),
                "finger_left_default": LaunchConfiguration("finger_left_default"),
                "finger_right_default": LaunchConfiguration("finger_right_default"),
                "validation_step_size": LaunchConfiguration("validation_step_size"),
                "smoothing_window": LaunchConfiguration("smoothing_window"),
                "smoothing_passes": LaunchConfiguration("smoothing_passes"),
                "min_r_point": LaunchConfiguration("min_r_point"),
                "self_filter_distance": LaunchConfiguration("self_filter_distance"),
            }],
        ),
    ])