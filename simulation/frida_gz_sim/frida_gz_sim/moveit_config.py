"""MoveIt configuration for the simulated FRIDA (same builder as the real robot)."""

import os

from ament_index_python.packages import get_package_share_directory
from arm_pkg.moveit_configs_builder import MoveItConfigsBuilder
from arm_pkg.vamp_pipeline import add_vamp_pipeline

from frida_gz_sim.description import GZ_PLUGIN, XARM_ARGS

PLANNERS = ("ompl", "vamp")


def build_moveit_config(context, planner: str = "ompl"):
    """MoveIt configs for the xArm on gz_ros2_control, with `planner` as the default pipeline."""
    if planner not in PLANNERS:
        raise ValueError(f"planner must be one of {PLANNERS}, got '{planner}'")
    controllers_file = os.path.join(
        get_package_share_directory("frida_gz_sim"), "config", "gz_controllers.yaml"
    )
    moveit_config = (
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
    add_vamp_pipeline(moveit_config, default=planner == "vamp")
    return moveit_config
