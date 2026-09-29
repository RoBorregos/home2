#!/usr/bin/env python3
"""Stand-in for nav_central + Nav2, so semantic_nav_node can be exercised for real.

Serves the areas JSON of the real arena map, publishes the saved .pgm as a
global costmap, and broadcasts map -> base_link at a pose we can move around.
"""
import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster

from frida_interfaces.srv import MapAreas

# Paths are derived from this file, so the suite runs from a source checkout and
# from inside the test container without editing anything.
REPO = os.environ.get("REPO_ROOT") or os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..")
)
NAV_MAIN = os.path.join(REPO, "navigation", "packages", "nav_main")
MAPS = os.path.join(REPO, "navigation", "packages", "map_context", "maps")
if NAV_MAIN not in sys.path:
    sys.path.insert(0, NAV_MAIN)


from nav_main.semantic.viewpoints import grid_from_pgm  # noqa: E402

MAP_NAME = os.environ.get("TEST_MAP", "robocup2026_1")


def read_map_yaml(path):
    out = {}
    for line in open(path):
        line = line.split("#", 1)[0].strip()
        if ":" not in line:
            continue
        key, value = (p.strip() for p in line.split(":", 1))
        if value.startswith("["):
            out[key] = json.loads(value)
        else:
            try:
                out[key] = float(value)
            except ValueError:
                out[key] = value
    return out


class FakeNav(Node):
    def __init__(self):
        super().__init__("fake_nav")
        with open(f"{MAPS}/areas/areas_{MAP_NAME}.json") as handle:
            self.areas_json = handle.read()
        self.create_service(MapAreas, "/navigation/areas_json", self.areas_cb)

        meta = read_map_yaml(f"{MAPS}/{MAP_NAME}.yaml")
        grid = grid_from_pgm(f"{MAPS}/{MAP_NAME}.pgm", meta)
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.info.resolution = grid.resolution
        msg.info.width = grid.width
        msg.info.height = grid.height
        msg.info.origin.position.x = grid.origin_x
        msg.info.origin.position.y = grid.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = list(grid.data)
        self.costmap = msg

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/global_costmap/costmap", latched)
        self.create_timer(1.0, self.publish_costmap)

        # Robot pose, moved by the driver through a parameter.
        self.declare_parameter("robot_x", 4.15)
        self.declare_parameter("robot_y", -11.73)
        self.declare_parameter("robot_yaw", 0.0)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.05, self.publish_tf)
        self.get_logger().info(
            f"fake_nav up: {len(self.areas_json)} bytes of areas, "
            f"costmap {grid.width}x{grid.height}"
        )

    def areas_cb(self, request, response):
        response.areas = self.areas_json
        return response

    def publish_costmap(self):
        self.costmap.header.stamp = self.get_clock().now().to_msg()
        self.costmap_pub.publish(self.costmap)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.get_parameter("robot_x").value
        t.transform.translation.y = self.get_parameter("robot_y").value
        yaw = self.get_parameter("robot_yaw").value
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = FakeNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
