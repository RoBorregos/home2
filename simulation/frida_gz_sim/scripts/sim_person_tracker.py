#!/usr/bin/env python3
"""Stand-in for vision's tracker_node in the navigation sim.

Same interface as `vision_general/scripts/tracker_node.py`, so the real
`VisionTasks.track_person()` / `person_goal_smoother` / task managers run unchanged:

- services  SET_TARGET_TOPIC (SetBool) and IS_TRACKING_TOPIC (Trigger)
- publishes RESULTS_TOPIC (PointStamped in CAMERA_FRAME, 3D person position) and
  CENTROID_TOPIC (Point, normalized horizontal offset for the arm pan)

The "detection" is the simulated person's ground truth (`/sim/person/pose`) with
the camera's limits applied: the person is seen only while inside the field of
view, inside depth range and not occluded by an arena wall (ray cast on the static
map). The FOV is measured from base_link, wide by default: on the robot the ZED
rides on the arm, whose joint1 pans to keep the person centered, so what matters
to navigation is roughly "in front of the base", not the raw camera cone. While
the person is not seen nothing is published, exactly like the real tracker, and
the base loses the goal. When the person comes back a `reacquire_delay` passes
before publishing resumes (stand-in for ByteTrack/ReID re-locking).
"""

import math
import random

import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from frida_constants.vision_constants import (
    CAMERA_FRAME,
    CENTROID_TOPIC,
    IS_TRACKING_TOPIC,
    RESULTS_TOPIC,
    SET_TARGET_TOPIC,
)
from frida_gz_sim import ground_truth as gt
from frida_gz_sim.nav import MAP_NAME
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_geometry_msgs import do_transform_point


class SimPersonTracker(Node):
    def __init__(self):
        super().__init__("sim_person_tracker")
        self.fov_half = math.radians(
            self.declare_parameter("fov_half_deg", 100.0).value
        )
        self.min_range = self.declare_parameter("min_range", 0.4).value
        self.max_range = self.declare_parameter("max_range", 7.0).value
        self.noise = self.declare_parameter("noise_m", 0.03).value
        self.rate = self.declare_parameter("rate_hz", 15.0).value
        self.reacquire_delay = self.declare_parameter("reacquire_delay_s", 1.0).value
        self.camera_frame = self.declare_parameter("camera_frame", CAMERA_FRAME).value

        maps = f"{get_package_share_directory('map_context')}/maps"
        with open(f"{maps}/{MAP_NAME}.yaml") as f:
            meta = yaml.safe_load(f)
        self.grid = gt.read_pgm(f"{maps}/{meta['image']}")
        self.res = meta["resolution"]
        self.origin = meta["origin"]
        self.occupied = self.grid < int((1.0 - meta["occupied_thresh"]) * 255)

        self.target_set = False
        self.person = None  # (x, y) map
        self.visible_since = None
        self.is_tracking = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseStamped, "/sim/person/pose", self._person_cb, 10)
        self.results_pub = self.create_publisher(PointStamped, RESULTS_TOPIC, 10)
        self.centroid_pub = self.create_publisher(Point, CENTROID_TOPIC, 10)
        self.visible_pub = self.create_publisher(String, "/sim/person/visible", 10)
        self.create_service(SetBool, SET_TARGET_TOPIC, self._set_target_cb)
        self.create_service(Trigger, IS_TRACKING_TOPIC, self._is_tracking_cb)
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f"sim tracker ready (fov ±{math.degrees(self.fov_half):.0f} deg, "
            f"range {self.min_range}-{self.max_range} m)"
        )

    # ----------------------------------------------------------- services
    def _set_target_cb(self, request, response):
        if request.data:
            seen = self._observe() is not None
            self.target_set = seen
            response.success = seen
            response.message = "target set" if seen else "no person in view"
            self.get_logger().info(
                "Tracking enabled: target set" if seen else "No person found"
            )
        else:
            self.target_set = False
            self.is_tracking = False
            response.success = True
            response.message = "tracking disabled"
            self.get_logger().info("Tracking disabled")
        return response

    def _is_tracking_cb(self, request, response):
        response.success = self.is_tracking
        return response

    def _person_cb(self, msg):
        self.person = (msg.pose.position.x, msg.pose.position.y)

    # -------------------------------------------------------- perception
    def _robot(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.05)
            )
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _cell(self, x, y):
        mx = int((x - self.origin[0]) / self.res)
        my = self.grid.shape[0] - 1 - int((y - self.origin[1]) / self.res)
        return mx, my

    def _occluded(self, x0, y0, x1, y1):
        """True if a wall cell lies on the segment (Bresenham on the static map)."""
        c0, r0 = self._cell(x0, y0)
        c1, r1 = self._cell(x1, y1)
        n = max(abs(c1 - c0), abs(r1 - r0), 1)
        for i in range(1, n):
            c = c0 + (c1 - c0) * i // n
            r = r0 + (r1 - r0) * i // n
            if 0 <= r < self.occupied.shape[0] and 0 <= c < self.occupied.shape[1]:
                if self.occupied[r, c]:
                    return True
        return False

    def _observe(self):
        """(bearing, distance, robot) if the person is seen right now, else None."""
        robot = self._robot()
        if robot is None or self.person is None:
            return None
        rx, ry, ryaw = robot
        px, py = self.person
        dist = math.hypot(px - rx, py - ry)
        if not (self.min_range <= dist <= self.max_range):
            return None
        bearing = math.atan2(py - ry, px - rx) - ryaw
        bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
        if abs(bearing) > self.fov_half:
            return None
        if self._occluded(rx, ry, px, py):
            return None
        return bearing, dist, robot

    def _tick(self):
        obs = self._observe() if self.target_set else None
        now = self.get_clock().now()
        if obs is None:
            self.visible_since = None
            self.is_tracking = False
            self.visible_pub.publish(
                String(data="hidden" if self.target_set else "off")
            )
            return
        if self.visible_since is None:
            self.visible_since = now
        self.visible_pub.publish(String(data="visible"))
        if (now - self.visible_since).nanoseconds / 1e9 < self.reacquire_delay:
            self.is_tracking = False
            return
        self.is_tracking = True
        bearing, dist, _ = obs

        # Normalized horizontal offset as the real tracker publishes it (-1 left ... +1 right)
        centroid = Point()
        centroid.x = float(max(-1.0, min(1.0, -bearing / self.fov_half)))
        self.centroid_pub.publish(centroid)

        # 3D point expressed in the camera optical frame, through TF, stamped now
        pt = PointStamped()
        pt.header.frame_id = "map"
        pt.header.stamp = now.to_msg()
        pt.point.x = self.person[0] + random.gauss(0.0, self.noise)
        pt.point.y = self.person[1] + random.gauss(0.0, self.noise)
        pt.point.z = 0.9
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame, "map", Time(), timeout=Duration(seconds=0.05)
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF map->{self.camera_frame}: {e}", throttle_duration_sec=5.0
            )
            return
        cam = do_transform_point(pt, t)
        cam.header.frame_id = self.camera_frame
        cam.header.stamp = now.to_msg()
        self.results_pub.publish(cam)


def main():
    rclpy.init()
    node = SimPersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
