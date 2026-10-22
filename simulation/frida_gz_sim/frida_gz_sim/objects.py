"""Graspable objects spawned in the sim worlds (model name -> (height, radius) in meters)."""

import os

WORLD_NAME = "pnp_table"
ROBOT_MODEL = "frida"
# The gripper body is lumped into the last arm link when the URDF becomes SDF
ATTACH_PARENT_LINK = (
    "openarm_right_link7" if os.environ.get("FRIDA_ARM") == "a2" else "link6"
)

GRASPABLE_OBJECTS = {
    "mustard_bottle": (0.19, 0.05),
    "mug": (0.115, 0.05),
    "medicine_bottle": (0.174, 0.036),
}


def attach_topic(name: str) -> str:
    return f"/sim/grasp/{name}/attach"


def detach_topic(name: str) -> str:
    return f"/sim/grasp/{name}/detach"


def state_topic(name: str) -> str:
    return f"/sim/grasp/{name}/state"
