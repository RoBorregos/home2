#!/usr/bin/env python3
"""Exports the URDF, SRDF and collision meshes of the running move_group, ready for FOAM and Cricket.

ros2 run vamp_moveit_plugin export_robot_model.py <output_dir>
"""

import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

MOVE_GROUP_PARAMETERS = "/move_group/get_parameters"
PLANNING_GROUP = "xarm6"


def resolve_mesh(uri: str) -> Path:
    """Local path of a package:// or file:// mesh URI."""
    parsed = urlparse(uri)
    if parsed.scheme == "package":
        return Path(get_package_share_directory(parsed.netloc)) / parsed.path.lstrip(
            "/"
        )
    return Path(parsed.path if parsed.scheme == "file" else uri)


def group_joints(srdf: ET.Element, group: str) -> set[str]:
    for element in srdf.findall("group"):
        if element.get("name") == group:
            return {joint.get("name") for joint in element.findall("joint")}
    raise ValueError(f"SRDF has no group '{group}'")


def prepare_urdf(urdf: ET.Element, planned_joints: set[str], mesh_dir: Path) -> None:
    """Freezes joints outside the planning group, drops unattached links and visuals, and copies the meshes."""
    for joint in urdf.findall("joint"):
        if joint.get("type") != "fixed" and joint.get("name") not in planned_joints:
            joint.set("type", "fixed")
            for tag in ("limit", "mimic", "axis", "dynamics"):
                for child in joint.findall(tag):
                    joint.remove(child)

    attached = {joint.find("child").get("link") for joint in urdf.findall("joint")}
    attached |= {joint.find("parent").get("link") for joint in urdf.findall("joint")}
    for element in list(urdf):
        if element.tag == "link" and element.get("name") not in attached:
            urdf.remove(element)
        elif element.tag not in ("link", "joint"):
            urdf.remove(element)

    mesh_dir.mkdir(parents=True, exist_ok=True)
    for link in urdf.findall("link"):
        for tag in ("visual", "inertial"):
            for child in link.findall(tag):
                link.remove(child)
        for index, mesh in enumerate(link.iter("mesh")):
            source = resolve_mesh(mesh.get("filename"))
            target = mesh_dir / f"{link.get('name')}_{index}{source.suffix.lower()}"
            shutil.copyfile(source, target)
            mesh.set("filename", f"{mesh_dir.name}/{target.name}")


class RobotModelExporter(Node):
    def __init__(self):
        super().__init__("robot_model_exporter")
        self.client = self.create_client(GetParameters, MOVE_GROUP_PARAMETERS)

    def fetch(self, *names: str) -> list[str]:
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("move_group is not running")
        future = self.client.call_async(GetParameters.Request(names=list(names)))
        rclpy.spin_until_future_complete(self, future)
        return [value.string_value for value in future.result().values]


def main():
    output = Path(sys.argv[1])
    rclpy.init()
    node = RobotModelExporter()
    try:
        urdf_xml, srdf_xml = node.fetch(
            "robot_description", "robot_description_semantic"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    urdf = ET.fromstring(urdf_xml)
    srdf = ET.fromstring(srdf_xml)
    prepare_urdf(urdf, group_joints(srdf, PLANNING_GROUP), output / "meshes")
    ET.ElementTree(urdf).write(output / "robot.urdf", xml_declaration=True)
    ET.ElementTree(srdf).write(output / "robot.srdf", xml_declaration=True)
    print(f"Exported {len(urdf.findall('link'))} links to {output}")


if __name__ == "__main__":
    main()
