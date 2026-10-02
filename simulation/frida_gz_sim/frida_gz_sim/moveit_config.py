"""MoveIt configuration for the simulated FRIDA (same builder as the real robot)."""

import os

from ament_index_python.packages import get_package_share_directory
from arm_pkg.moveit_configs_builder import MoveItConfigsBuilder

from frida_gz_sim.description import GZ_PLUGIN, XARM_ARGS


def build_moveit_config(context):
    """MoveIt configs for the xArm on gz_ros2_control."""
    controllers_file = os.path.join(
        get_package_share_directory("frida_gz_sim"), "config", "gz_controllers.yaml"
    )
    return (
        MoveItConfigsBuilder(
            context=context,
            controllers_name="controllers",
            dof=XARM_ARGS["dof"],
            robot_type=XARM_ARGS["robot_type"],
            prefix=XARM_ARGS["prefix"],
            hw_ns=XARM_ARGS["hw_ns"],
            limited=XARM_ARGS["limited"],
            attach_to=XARM_ARGS["attach_to"],
            attach_xyz='"0 0 0"',
            attach_rpy='"0 0 1.5707963267948966"',
            mesh_suffix=XARM_ARGS["mesh_suffix"],
            ros2_control_plugin=GZ_PLUGIN,
            ros2_control_params=controllers_file,
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
