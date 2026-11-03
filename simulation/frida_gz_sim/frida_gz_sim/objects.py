"""Graspable objects spawned in the sim worlds (model name -> (height, radius) in meters)."""

WORLD_NAME = "pnp_table"
ROBOT_MODEL = "frida"
# The gripper body is lumped into link6 when the URDF becomes SDF
ATTACH_PARENT_LINK = "link6"

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
