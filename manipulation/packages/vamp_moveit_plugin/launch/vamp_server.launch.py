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

Singleton guard:
  The frida moveit configs include this launch file (start_vamp_server:=true),
  so running it again by hand would create a second node named "vamp_server"
  — a name collision that makes the VAMP service ambiguous. To prevent that,
  this launch checks the ROS graph at startup and skips launching if a
  "vamp_server" node is already running. Override with force:=true if you
  really want a second instance.
"""

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# (param name -> default value) for every tunable the server accepts.
_SERVER_PARAMS = {
    "max_iterations": "50000",
    "range": "0.05",
    "security_margin": "0.02",
    "max_retries": "3",
    "finger_left_default": "0.8",
    "finger_right_default": "0.8",
    "validation_step_size": "0.05",
    "smoothing_window": "5",
    "smoothing_passes": "3",
    "min_r_point": "0.09",
    "self_filter_distance": "0.12",
}


def _vamp_server_already_running():
    """True if a 'vamp_server' node is already on the ROS graph (via `ros2 node list`).
    Fails open: any error (timeout, ros2 missing) returns False so the guard never blocks a launch."""
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except Exception:
        return False
    return any(
        line.strip().rsplit("/", 1)[-1] == "vamp_server"
        for line in result.stdout.splitlines()
    )


def launch_setup(context, *args, **kwargs):
    force = LaunchConfiguration("force").perform(context).lower() in ("true", "1")

    if not force and _vamp_server_already_running():
        print(
            "[vamp_server.launch] A 'vamp_server' node is already running — "
            "skipping this instance to avoid a duplicate-name collision. "
            "Pass force:=true to start another anyway."
        )
        return []

    return [
        Node(
            package="vamp_moveit_plugin",
            executable="vamp_server.py",
            name="vamp_server",
            output="screen",
            parameters=[{name: LaunchConfiguration(name) for name in _SERVER_PARAMS}],
        )
    ]


def generate_launch_description():
    arg_descriptions = {
        "max_iterations": "Max RRT iterations per attempt",
        "range": "RRT extension range in C-space",
        "security_margin": "Extra collision margin (meters)",
        "max_retries": "Number of planning retries with increasing params",
        "finger_left_default": "Default left finger joint value",
        "finger_right_default": "Default right finger joint value",
        "validation_step_size": "Interpolation step for path validation (radians)",
        "smoothing_window": "Moving average window for path smoothing",
        "smoothing_passes": "Number of smoothing passes",
        "min_r_point": "Minimum collision radius per voxel (compensates sphere model)",
        "self_filter_distance": "Distance to filter robot body from pointcloud (meters)",
    }

    declared_args = [
        DeclareLaunchArgument(
            name, default_value=default, description=arg_descriptions[name]
        )
        for name, default in _SERVER_PARAMS.items()
    ]
    declared_args.append(
        DeclareLaunchArgument(
            "force",
            default_value="false",
            description="Start vamp_server even if one is already running (bypasses the singleton guard)",
        )
    )

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
