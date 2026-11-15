#!/usr/bin/env python3
"""Harness nodes for one scenario: ground-truth detector, scripted HRI, referee and recorder."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    params = [
        {
            "scenario_file": LaunchConfiguration("scenario_file"),
            "run_dir": LaunchConfiguration("run_dir"),
        }
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_file"),
            DeclareLaunchArgument("run_dir"),
            SetParameter(name="use_sim_time", value=True),
            *[
                Node(
                    package="frida_task_sim",
                    executable=script,
                    output="screen",
                    parameters=params,
                )
                for script in (
                    "gt_object_detector.py",
                    "fake_hri.py",
                    "referee.py",
                    "recorder.py",
                )
            ],
        ]
    )
