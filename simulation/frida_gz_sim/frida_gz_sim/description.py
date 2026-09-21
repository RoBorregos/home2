"""Builds the Gazebo robot description from the real FRIDA xacro."""

import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory

GZ_PLUGIN = "gz_ros2_control/GazeboSimSystem"
FINGER_JOINTS = ("rightfinger", "leftfinger")

# Box collisions (size, center) fitted to the gripper meshes; mesh contacts let objects tunnel into the palm
GRIPPER_BOXES = {
    "gripper": ((0.198, 0.129, 0.081), (0.0, -0.0255, -0.0055)),
    "right_finger": ((0.039, 0.0616, 0.1), (-0.0065, 0.0, 0.05)),
    "left_finger": ((0.039, 0.0616, 0.1), (0.0065, 0.0, 0.05)),
}

# Same xacro arguments the real MoveIt launch passes (arm_pkg MoveItConfigsBuilder)
XARM_ARGS = {
    "ros2_control_plugin": GZ_PLUGIN,
    "dof": "6",
    "robot_type": "xarm",
    "prefix": "",
    "hw_ns": "xarm",
    "limited": "true",
    "attach_to": "xarm_base",
    "attach_xyz": "0 0 0",
    "attach_rpy": "0 0 1.5707963267948966",
    "mesh_suffix": "stl",
}


def build_robot_description(
    finger_effort: float = 150.0,
    image_width: int = 640,
    image_height: int = 360,
    camera_rate: int = 10,
) -> str:
    """Return the sim URDF string with sim-only patches applied."""
    share = get_package_share_directory("frida_gz_sim")
    mappings = dict(XARM_ARGS)
    mappings.update(
        {
            "controllers_file": os.path.join(share, "config", "gz_controllers.yaml"),
            "image_width": str(image_width),
            "image_height": str(image_height),
            "camera_rate": str(camera_rate),
        }
    )
    doc = xacro.process_file(
        os.path.join(share, "urdf", "frida_gz.urdf.xacro"), mappings=mappings
    )
    root = ET.fromstring(doc.toxml())

    # SDF shares one frame namespace between links and joints (FRIDA_Real has link and joint "zed")
    link_names = {link.get("name") for link in root.findall("link")}
    for joint in root.findall("joint"):
        if joint.get("name") in link_names:
            joint.set("name", joint.get("name") + "_joint")

    for joint in root.findall("joint"):
        if joint.get("name") not in FINGER_JOINTS:
            continue
        # Fingers are commanded independently, a URDF mimic would fight the controller
        mimic = joint.find("mimic")
        if mimic is not None:
            joint.remove(mimic)
        limit = joint.find("limit")
        if limit is not None:
            limit.set("effort", str(finger_effort))

    for link in root.findall("link"):
        if link.get("name") in GRIPPER_BOXES:
            _box_collision(link, *GRIPPER_BOXES[link.get("name")])

    return ET.tostring(root, encoding="unicode")


def _box_collision(link: ET.Element, size: tuple, center: tuple) -> None:
    """Replace a link's collision geometry with a single box."""
    for collision in link.findall("collision"):
        link.remove(collision)
    collision = ET.SubElement(link, "collision")
    ET.SubElement(
        collision, "origin", {"xyz": " ".join(map(str, center)), "rpy": "0 0 0"}
    )
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", {"size": " ".join(map(str, size))})
