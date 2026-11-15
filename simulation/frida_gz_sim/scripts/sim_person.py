#!/usr/bin/env python3
"""A walking person for the navigation sim.

Spawns a visual-only cylinder (`sim_person`) in the arena and moves it with the
world's `set_pose` service along a route of waypoints, so the gpu_lidar sees it
exactly the way the real lidar sees a person's legs: as a moving obstacle in the
costmaps. No collision geometry, like the arena walls (see the sim README).

Routes arrive on `/sim/person/route` (std_msgs/String, JSON):
    {"waypoints": [[x, y, speed_mps, dwell_s], ...], "teleport": false}
Each waypoint is walked to in a straight line at `speed_mps`, then the person
waits `dwell_s`. A speed of 0 teleports (used to hide the person instantly).
`/sim/person/pose` (PoseStamped, map frame) is the ground truth at 20 Hz, and
`/sim/person/state` (String) is "walking" / "idle" / "dwelling".
"""

import json
import math
import time

import rclpy
from frida_gz_sim.nav import MAP_NAME
from geometry_msgs.msg import PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from std_msgs.msg import String

WORLD = f"arena_{MAP_NAME}"
MODEL = "sim_person"
RADIUS = 0.2
HEIGHT = 1.7
TICK = 0.05


def person_sdf(x, y, z=HEIGHT / 2.0):
    """A standing person as a non-static cylinder with gravity off, so set_pose moves it."""
    return (
        f"<sdf version='1.9'><model name='{MODEL}'><static>false</static>"
        f"<pose>{x} {y} {z} 0 0 0</pose>"
        f"<link name='body'><gravity>false</gravity>"
        f"<inertial><mass>1.0</mass></inertial>"
        f"<visual name='v'><geometry><cylinder><radius>{RADIUS}</radius>"
        f"<length>{HEIGHT}</length></cylinder></geometry>"
        f"<material><ambient>0.2 0.4 0.9 1</ambient><diffuse>0.2 0.4 0.9 1</diffuse>"
        f"</material></visual></link></model></sdf>"
    )


class SimPerson(Node):
    def __init__(self):
        super().__init__("sim_person")
        self.x = self.declare_parameter("x", 0.0).value
        self.y = self.declare_parameter("y", 0.0).value
        self.gz = GzNode()
        self.route = []  # list of (x, y, speed, dwell)
        self.dwell_until = 0.0
        self.state = "idle"
        self.pose_pub = self.create_publisher(PoseStamped, "/sim/person/pose", 10)
        self.state_pub = self.create_publisher(String, "/sim/person/state", 10)
        self.create_subscription(String, "/sim/person/route", self._route_cb, 10)
        self._spawn()
        self.create_timer(TICK, self._tick)
        self.get_logger().info(f"sim_person at ({self.x:.2f}, {self.y:.2f})")

    # ------------------------------------------------------------- gazebo
    def _request(self, service, req, rep_type=Boolean, timeout_ms=2000):
        ok, rep = self.gz.request(
            f"/world/{WORLD}/{service}", req, type(req), rep_type, timeout_ms
        )
        if not ok:
            self.get_logger().warn(f"gz {service} failed", throttle_duration_sec=5.0)
        return ok

    def _spawn(self):
        self._request("remove", Entity(name=MODEL, type=Entity.MODEL))
        time.sleep(0.5)
        req = EntityFactory()
        req.sdf = person_sdf(self.x, self.y)
        req.name = MODEL
        req.allow_renaming = False
        self._request("create", req)

    def _set_pose(self):
        req = Pose()
        req.name = MODEL
        req.position.x = float(self.x)
        req.position.y = float(self.y)
        req.position.z = HEIGHT / 2.0
        req.orientation.w = 1.0
        self._request("set_pose", req, timeout_ms=200)

    # --------------------------------------------------------------- route
    def _route_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            self.get_logger().error(f"bad route: {msg.data}")
            return
        self.route = [tuple(float(v) for v in wp) for wp in data.get("waypoints", [])]
        self.dwell_until = 0.0
        self.get_logger().info(f"route: {self.route}")

    def _tick(self):
        now = time.time()
        if self.route and now >= self.dwell_until:
            tx, ty, speed, dwell = self.route[0]
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            step = speed * TICK
            if speed <= 0.0 or dist <= step:
                self.x, self.y = tx, ty
                self.route.pop(0)
                self.dwell_until = now + dwell
                self.state = "dwelling" if dwell > 0 else "walking"
            else:
                self.x += dx / dist * step
                self.y += dy / dist * step
                self.state = "walking"
            self._set_pose()
        elif not self.route and now >= self.dwell_until:
            self.state = "idle"
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self.state_pub.publish(String(data=self.state))


def main():
    rclpy.init()
    node = SimPerson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._request("remove", Entity(name=MODEL, type=Entity.MODEL))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
