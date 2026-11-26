#!/usr/bin/env python3
"""Kinematic office_door: the hinge and lever follow a closed gripper, with the latch and grip emulated.

Contact and welds on a lever deep in a joint chain are not usable in Gazebo's DART, so the
door joints are position-driven from here:
- A gripper closed around the bar holds it firmly: it may slide along the bar, sit up to
  ``depth_slack`` deeper between the pads and turn about the bar, nothing else.
- The lever turns with the grip (spring back to 0 when free); past ``unlatch_angle`` the
  latch lets the leaf go, and the leaf then swings with the grip. A shut door latches again.
- Whatever the grip cannot follow becomes a spring force on the gripper
  (/sim/door/virtual_wrench, base_link), which joint_effort_sim.py adds to the joint efforts;
  past ``lose_distance`` the grip is lost.
- /sim/door/joint_states and /sim/door/latched are the ground truth for scoring.
"""

import math
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import WrenchStamped
from gz.msgs10.double_pb2 import Double
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from gz.transport13 import SubscribeOptions
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

ROBOT_MODEL = "frida"
# Finger pads sit this far along the finger link Z from its origin
PAD_OFFSET = 0.06
# Door geometry in the model frame (see models/office_door/model.sdf)
HINGE = np.array([0.0, 0.403, 0.0])
PIVOT_IN_LEAF = np.array([0.0, -0.733, 1.0])
BAR_X = 0.06  # bar axis off the door face
BAR_SPAN = (-0.01, 0.12)  # bar along the lever, from the pivot
LEVER_LIMITS = (-0.75, 0.0)
HINGE_LIMITS = (0.0, 1.75)


def _rot_x(angle: float) -> Rotation:
    return Rotation.from_euler("x", angle)


def _rot_z(angle: float) -> Rotation:
    return Rotation.from_euler("z", angle)


class DoorSim(Node):
    def __init__(self):
        super().__init__("door_sim")
        self.world = self.declare_parameter("world", "door_house").value
        self.unlatch_angle = self.declare_parameter("unlatch_angle", -0.35).value
        self.relatch_angle = self.declare_parameter("relatch_angle", 0.02).value
        self.max_gap = self.declare_parameter("max_gap", 0.03).value
        self.close_delay = self.declare_parameter("close_delay", 0.8).value
        self.depth_slack = self.declare_parameter("depth_slack", 0.04).value
        self.stiffness = self.declare_parameter("stiffness", 1500.0).value
        # N per m/s of the grip along the swing, like a door closer's damper
        self.swing_damping = self.declare_parameter("swing_damping", 15.0).value
        self.lose_distance = self.declare_parameter("lose_distance", 0.05).value
        door_pose = self.declare_parameter("door_pose", [0.0, 0.0, 0.0]).value
        self.door_origin = np.array([door_pose[0], door_pose[1], 0.0])
        self.door_rotation = _rot_z(door_pose[2])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._lock = threading.Lock()
        self.hinge = 0.0
        self.lever = 0.0
        self.hinge_cmd = 0.0
        self.lever_cmd = 0.0
        self.latched = True
        self.closed = False
        self.grip_depth = (
            None  # grip x in the leaf frame when it closed, None when free
        )
        self.base_position = None
        self.base_rotation = None
        self._close_at = None
        self._shut_since = None
        self._last_hinge = (time.monotonic(), 0.0)

        self.gz = GzNode()
        self.hinge_pub = self.gz.advertise("/sim/door/cmd/hinge", Double)
        self.lever_pub = self.gz.advertise("/sim/door/cmd/lever", Double)
        self.gz.subscribe(Model, "/sim/door/joint_state", self._on_door_joints)
        pose_options = SubscribeOptions()
        pose_options.msgs_per_sec = 50
        self.gz.subscribe(
            Pose_V, f"/world/{self.world}/pose/info", self._on_poses, pose_options
        )

        self.joints_pub = self.create_publisher(
            JointState, "/sim/door/joint_states", 10
        )
        self.latched_pub = self.create_publisher(Bool, "/sim/door/latched", 10)
        self.wrench_pub = self.create_publisher(
            WrenchStamped, "/sim/door/virtual_wrench", 10
        )
        # Same topics as grasp_attach.py, so the gripper bridge reports the grasp and stops squeezing
        self.attached_pub = self.create_publisher(Bool, "/sim/grasp_attached", 10)
        self.object_pub = self.create_publisher(
            String, "/sim/grasp_attached_object", 10
        )
        self.create_subscription(Bool, "/sim/gripper_closed", self._on_gripper, 10)
        self.create_timer(0.02, self._tick)
        self.get_logger().info("Door sim ready")

    def _on_door_joints(self, msg: Model):
        for joint in msg.joint:
            if joint.name == "hinge":
                self.hinge = joint.axis1.position
            elif joint.name == "lever_joint":
                self.lever = joint.axis1.position

    def _on_poses(self, msg: Pose_V):
        for p in msg.pose:
            if p.name == ROBOT_MODEL:
                q = p.orientation
                self.base_position = np.array(
                    [p.position.x, p.position.y, p.position.z]
                )
                self.base_rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
                return

    def _on_gripper(self, msg: Bool):
        with self._lock:
            self.closed = msg.data
            if msg.data:
                self._close_at = time.monotonic() + self.close_delay
            else:
                self._close_at = None
                self._release("gripper opened")

    def _grasp_point(self):
        """Middle of the finger pads in base_link, or None."""
        points = []
        for finger in ("right_finger", "left_finger"):
            try:
                tf = self.tf_buffer.lookup_transform("base_link", finger, Time())
            except tf2_ros.TransformException:
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            pad = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([0.0, 0.0, PAD_OFFSET])
            points.append(np.array([t.x, t.y, t.z]) + pad)
        return (points[0] + points[1]) / 2.0

    def _to_leaf(self, point_door: np.ndarray, hinge: float) -> np.ndarray:
        return _rot_z(-hinge).apply(point_door - HINGE)

    def _nearest_on_bar(self, leaf: np.ndarray, x_range) -> tuple[np.ndarray, float]:
        """Closest admissible grip point (leaf frame) to ``leaf`` and the lever angle it gives."""
        rel = leaf - PIVOT_IN_LEAF
        angle = float(np.clip(math.atan2(rel[2], rel[1]), *LEVER_LIMITS))
        along = float(np.clip(math.hypot(rel[1], rel[2]), *BAR_SPAN))
        x = float(np.clip(leaf[0], *x_range))
        point = PIVOT_IN_LEAF + np.array(
            [x, along * math.cos(angle), along * math.sin(angle)]
        )
        return point, angle

    def _swing_to(self, v: np.ndarray, target_x: float, current: float) -> float:
        """Hinge angle putting the leaf-frame x of ``v`` (from the hinge) at ``target_x``."""
        radius = math.hypot(v[0], v[1])
        if radius < 1e-3 or abs(target_x) > radius:
            return current
        beta = math.atan2(v[1], v[0])
        spread = math.acos(target_x / radius)
        options = [beta + spread, beta - spread]
        best = min(options, key=lambda a: abs(math.remainder(a - current, 2 * math.pi)))
        best = current + math.remainder(best - current, 2 * math.pi)
        return float(np.clip(best, *HINGE_LIMITS))

    def _tick(self):
        with self._lock:
            self._step()

    def _step(self):
        now = time.monotonic()
        force_door = np.zeros(3)
        grasp_base = None
        if self.base_position is not None:
            grasp_base = self._grasp_point()
        grasp_door = None
        if grasp_base is not None:
            world = self.base_position + self.base_rotation.apply(grasp_base)
            grasp_door = self.door_rotation.inv().apply(world - self.door_origin)

        if self._close_at is not None and now >= self._close_at:
            self._close_at = None
            self._try_grip(grasp_door)

        if self.grip_depth is not None and grasp_door is not None:
            x_range = (self.grip_depth - self.depth_slack, self.grip_depth)
            hinge = self.hinge_cmd
            if not self.latched:
                leaf = self._to_leaf(grasp_door, hinge)
                if not x_range[0] <= leaf[0] <= x_range[1]:
                    target = float(np.clip(leaf[0], *x_range))
                    hinge = self._swing_to(grasp_door - HINGE, target, hinge)
            leaf = self._to_leaf(grasp_door, hinge)
            nearest, angle = self._nearest_on_bar(leaf, x_range)
            error_leaf = leaf - nearest
            self.hinge_cmd, self.lever_cmd = hinge, angle
            error = _rot_z(hinge).apply(error_leaf)
            force_door = -self.stiffness * error
            # Resistance to the swing, along the grip's motion about the hinge
            t_prev, h_prev = self._last_hinge
            rate = (self.hinge - h_prev) / max(now - t_prev, 1e-3)
            radius_vec = grasp_door - HINGE
            tangent = np.cross([0.0, 0.0, 1.0], radius_vec)
            force_door = force_door - self.swing_damping * rate * tangent
            if float(np.linalg.norm(error)) > self.lose_distance:
                self._release(
                    f"grip lost: {np.linalg.norm(error) * 1000:.0f} mm off the bar"
                )
                force_door = np.zeros(3)
        elif self.grip_depth is None:
            # Free lever springs back up; a free leaf stays where it was left
            self.lever_cmd = 0.0
            if not self.latched:
                self.hinge_cmd = self.hinge
        self._last_hinge = (now, self.hinge)

        self._update_latch(now)
        if self.latched:
            self.hinge_cmd = 0.0
        self.hinge_pub.publish(Double(data=float(self.hinge_cmd)))
        self.lever_pub.publish(Double(data=float(self.lever_cmd)))
        self._publish(force_door)

    def _update_latch(self, now: float):
        if self.latched and self.lever < self.unlatch_angle:
            self.latched = False
            self.get_logger().info(
                f"Lever at {math.degrees(self.lever):.0f} deg: door unlatched"
            )
        elif not self.latched:
            shut = self.hinge < self.relatch_angle and self.lever > -0.1
            if not shut:
                self._shut_since = None
            elif self._shut_since is None:
                self._shut_since = now
            elif now - self._shut_since > 1.0:
                self.latched = True
                self._shut_since = None
                self.get_logger().info("Door shut: latched again")

    def _try_grip(self, grasp_door):
        if not self.closed or self.grip_depth is not None or grasp_door is None:
            return
        leaf = self._to_leaf(grasp_door, self.hinge)
        bar_x = (BAR_X - 0.02, BAR_X + 0.02)  # the bar's thickness plus the pads
        nearest, _ = self._nearest_on_bar(leaf, bar_x)
        gap = float(np.linalg.norm(leaf - nearest))
        if gap > self.max_gap:
            self.get_logger().info(
                f"Gripper closed {gap * 1000:.0f} mm off the lever bar: no grip"
            )
            return
        self.grip_depth = float(leaf[0])
        self.hinge_cmd = self.hinge
        self.attached_pub.publish(Bool(data=True))
        self.object_pub.publish(String(data="door_lever"))
        self.get_logger().info(
            f"Gripping the lever ({gap * 1000:.0f} mm off the bar axis, {self.grip_depth * 1000:.0f} mm off the door)"
        )

    def _release(self, why: str):
        if self.grip_depth is None:
            return
        self.grip_depth = None
        self.attached_pub.publish(Bool(data=False))
        self.object_pub.publish(String(data=""))
        self.get_logger().info(f"Released the lever ({why})")

    def _publish(self, force_door: np.ndarray):
        stamp = self.get_clock().now().to_msg()
        state = JointState()
        state.header.stamp = stamp
        state.name = ["hinge", "lever_joint"]
        state.position = [float(self.hinge), float(self.lever)]
        self.joints_pub.publish(state)
        self.latched_pub.publish(Bool(data=self.latched))
        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.header.frame_id = "base_link"
        if self.base_rotation is not None:
            force = self.base_rotation.inv().apply(self.door_rotation.apply(force_door))
            wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = map(
                float, force
            )
        self.wrench_pub.publish(wrench)


def main(args=None):
    rclpy.init(args=args)
    node = DoorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
