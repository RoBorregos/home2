"""Registers the VAMP planning pipeline on a MoveIt config built by MoveItConfigsBuilder."""

import copy
import os

import yaml
from ament_index_python.packages import get_package_share_directory


def add_vamp_pipeline(moveit_config, default: bool = False) -> None:
    """Adds the "vamp" pipeline on top of the OMPL one, whose planner configs VAMP's OMPL fallback reads."""
    overrides_file = os.path.join(
        get_package_share_directory("vamp_moveit_plugin"),
        "config",
        "vamp_planning.yaml",
    )
    with open(overrides_file) as file:
        overrides = yaml.safe_load(file)

    pipelines = moveit_config.planning_pipelines
    pipelines["vamp"] = {**copy.deepcopy(pipelines["ompl"]), **overrides}
    pipelines["planning_pipelines"] = [*pipelines["planning_pipelines"], "vamp"]
    if default:
        pipelines["default_planning_pipeline"] = "vamp"
