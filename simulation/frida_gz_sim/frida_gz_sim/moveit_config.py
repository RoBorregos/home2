"""MoveIt configuration for the simulated FRIDA (same builder as the real robot)."""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from arm_pkg.moveit_configs_builder import MoveItConfigsBuilder
from moveit_configs_utils import MoveItConfigs

from frida_gz_sim.description import ARM, GZ_PLUGIN, XARM_ARGS


def build_moveit_config(context):
    """MoveIt configs for the arm selected by FRIDA_ARM on gz_ros2_control."""
    if ARM == "a2":
        return build_a2_moveit_config()
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


def _load(path: str) -> dict:
    with open(path) as file:
        return yaml.safe_load(file)


def build_a2_moveit_config() -> MoveItConfigs:
    """MoveIt configs for the A2 right arm, assembled from config/a2_moveit."""
    config_dir = os.path.join(
        get_package_share_directory("frida_gz_sim"), "config", "a2_moveit"
    )
    # OMPL adapters and planner configs shared with the xArm setup
    ompl_dir = os.path.join(
        get_package_share_directory("xarm_moveit_config"), "config", "moveit_configs"
    )
    urdf = xacro.process_file(
        os.path.join(
            get_package_share_directory("frida_description"),
            "urdf",
            "omnibase",
            "FRIDA_A2.urdf.xacro",
        )
    ).toxml()
    with open(os.path.join(config_dir, "frida_a2.srdf")) as file:
        srdf = file.read()

    ompl = _load(os.path.join(ompl_dir, "ompl_planning.yaml"))
    ompl.update(_load(os.path.join(ompl_dir, "ompl_defaults.yaml")))
    # gz position control settles a hair past the A2's tight limits (j6 +-45 deg); clamp instead of failing
    ompl["fix_start_state"] = True
    planner_ids = list(ompl["planner_configs"].keys())
    for group in ("a2_arm", "a2_left_arm"):
        ompl[group] = {
            "default_planner_config": "RRTConnect",
            "planner_configs": planner_ids,
        }

    config = MoveItConfigs()
    config.robot_description = {"robot_description": urdf}
    config.robot_description_semantic = {"robot_description_semantic": srdf}
    config.robot_description_kinematics = {
        "robot_description_kinematics": _load(
            os.path.join(config_dir, "kinematics.yaml")
        )
    }
    config.joint_limits = {
        "robot_description_planning": _load(
            os.path.join(config_dir, "joint_limits.yaml")
        )
    }
    config.planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl,
    }
    config.trajectory_execution = {
        "moveit_manage_controllers": False,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": _load(
            os.path.join(config_dir, "moveit_controllers.yaml")
        ),
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
        "trajectory_execution.execution_duration_monitoring": False,
        "plan_execution.record_trajectory_state_frequency": 10.0,
    }
    config.planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": False,
        "publish_robot_description_semantic": False,
    }
    return config
