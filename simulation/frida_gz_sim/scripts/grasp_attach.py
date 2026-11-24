#!/usr/bin/env python3
"""Welds the object between the fingers to the gripper on close and releases it on open.

Physics-only grasps in Gazebo let objects pivot or slip out of a two-finger pinch, so
this mirrors a firm real grasp without touching the manipulation code. Each object gets
a gz DetachableJoint system added to the robot at startup (adding one welds the object
at once, so it is detached right away while the arm is still); grasps then only toggle
attach/detach.
"""

import re
import subprocess
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from frida_gz_sim.objects import (
    ATTACH_PARENT_LINK,
    GRASPABLE_OBJECTS,
    ROBOT_MODEL,
    WORLD_NAME,
    attach_topic,
    detach_topic,
    state_topic,
)
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode
from gz.transport13 import SubscribeOptions
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool

# Finger pads sit this far along the finger link Z from its origin
PAD_OFFSET = 0.06


def _gz(*args: str, timeout: float = 30.0) -> str:
    """Run a gz CLI command (its requests are more reliable than the Python client's)."""
    try:
        result = subprocess.run(
            ["gz", *args], capture_output=True, text=True, timeout=timeout, check=False
        )
    except subprocess.TimeoutExpired:
        return ""
    return result.stdout


class GraspAttach(Node):
    def __init__(self):
        super().__init__("grasp_attach")
        self.max_gap = self.declare_parameter("max_gap", 0.035).value
        self.close_delay = self.declare_parameter("close_delay", 1.0).value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # object name -> (origin, unit up axis) in world frame
        self.poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.states: dict[str, str] = {}
        self.attached: str | None = None
        self.closed = False
        self.ready = False

        self.gz = GzNode()
        self.attach_pubs = {
            n: self.gz.advertise(attach_topic(n), Empty) for n in GRASPABLE_OBJECTS
        }
        self.detach_pubs = {
            n: self.gz.advertise(detach_topic(n), Empty) for n in GRASPABLE_OBJECTS
        }
        # pose/info covers every entity; dynamic_pose/info skips objects that have never moved
        pose_options = SubscribeOptions()
        pose_options.msgs_per_sec = 20
        self.gz.subscribe(
            Pose_V, f"/world/{WORLD_NAME}/pose/info", self._on_poses, pose_options
        )
        for name in GRASPABLE_OBJECTS:
            self.gz.subscribe(
                StringMsg,
                state_topic(name),
                lambda msg, name=name: self.states.__setitem__(name, msg.data),
            )

        self.create_subscription(Bool, "/sim/gripper_closed", self._on_gripper, 10)
        # Tells the gripper bridge to stop squeezing a welded object
        self.attached_pub = self.create_publisher(Bool, "/sim/grasp_attached", 10)
        self._close_timer: threading.Timer | None = None
        self._lock = threading.Lock()
        threading.Thread(target=self._setup, daemon=True).start()

    def _setup(self):
        """Add one DetachableJoint per object, then release every weld it creates."""
        robot_id = None
        while rclpy.ok() and robot_id is None:
            match = re.search(r"Model: \[(\d+)\]", _gz("model", "-m", ROBOT_MODEL))
            robot_id = int(match.group(1)) if match else None
            time.sleep(1.0)

        for name in GRASPABLE_OBJECTS:
            innerxml = (
                f"<parent_link>{ATTACH_PARENT_LINK}</parent_link>"
                f"<child_model>{name}</child_model><child_link>link</child_link>"
                f"<attach_topic>{attach_topic(name)}</attach_topic>"
                f"<detach_topic>{detach_topic(name)}</detach_topic>"
                f"<output_topic>{state_topic(name)}</output_topic>"
                "<suppress_child_warning>true</suppress_child_warning>"
            )
            request = (
                f"entity: {{id: {robot_id}, type: MODEL}}, plugins: [{{"
                'name: "gz::sim::systems::DetachableJoint", '
                'filename: "gz-sim-detachable-joint-system", '
                f'innerxml: "{innerxml}"}}]'
            )
            reply = _gz(
                "service",
                "-s",
                f"/world/{WORLD_NAME}/entity/system/add",
                "--reqtype",
                "gz.msgs.EntityPlugin_V",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "20000",
                "--req",
                request,
            )
            if "true" not in reply:
                self.get_logger().error(f"Could not add a DetachableJoint for {name}")

        for name in GRASPABLE_OBJECTS:
            for _ in range(10):
                _gz("topic", "-t", detach_topic(name), "-m", "gz.msgs.Empty", "-p", "")
                time.sleep(0.5)
                if self.states.get(name) == "detached":
                    break
            else:
                self.get_logger().error(f"{name} may still be welded to the gripper")

        self.ready = True
        self.get_logger().info(
            f"Grasp attach ready for {list(GRASPABLE_OBJECTS)} (robot entity {robot_id})"
        )

    def _on_poses(self, msg: Pose_V):
        poses = {}
        for p in msg.pose:
            if p.name not in GRASPABLE_OBJECTS:
                continue
            q = p.orientation
            up = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([0.0, 0.0, 1.0])
            poses[p.name] = (np.array([p.position.x, p.position.y, p.position.z]), up)
        self.poses.update(poses)

    def _on_gripper(self, msg: Bool):
        if msg.data == self.closed:
            return
        self.closed = msg.data
        if self._close_timer is not None:
            self._close_timer.cancel()
            self._close_timer = None
        if msg.data:
            self._close_timer = threading.Timer(self.close_delay, self._try_attach)
            self._close_timer.start()
        else:
            with self._lock:
                self._release()

    def _grasp_point(self) -> np.ndarray | None:
        points = []
        for finger in ("right_finger", "left_finger"):
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link", finger, Time(), Duration(seconds=0.5)
                )
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f"No TF for {finger}: {e}")
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            axis = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply(
                [0.0, 0.0, PAD_OFFSET]
            )
            points.append(np.array([t.x, t.y, t.z]) + axis)
        return (points[0] + points[1]) / 2.0

    def _closest_object(self, grasp: np.ndarray) -> tuple[str | None, float]:
        best, best_dist = None, self.max_gap
        for name, (origin, up) in dict(self.poses).items():
            height, radius = GRASPABLE_OBJECTS[name]
            # Gap to the object's surface around its axis, so grasps anywhere along it (or on a tipped object) count
            along = float(np.clip(np.dot(grasp - origin, up), 0.0, height))
            dist = float(np.linalg.norm(grasp - (origin + along * up))) - radius
            if dist < best_dist:
                best, best_dist = name, dist
        return best, best_dist

    def _try_attach(self):
        with self._lock:
            self._attach_locked()

    def _attach_locked(self):
        if not self.ready:
            self.get_logger().warn("Gripper closed before grasp attach finished setup")
            return
        if not self.closed or self.attached is not None:
            return
        grasp = self._grasp_point()
        if grasp is None:
            return
        name, dist = self._closest_object(grasp)
        if name is None:
            self.get_logger().info(
                f"Gripper closed on nothing (grasp point {np.round(grasp, 3)})"
            )
            return
        self.attach_pubs[name].publish(Empty())
        self.attached = name
        self.attached_pub.publish(Bool(data=True))
        self.get_logger().info(
            f"Attached {name} ({max(dist, 0.0):.3f} m gap to the fingers)"
        )

    def _release(self):
        if self.attached is None:
            return
        self.detach_pubs[self.attached].publish(Empty())
        self.get_logger().info(f"Released {self.attached}")
        self.attached = None
        self.attached_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = GraspAttach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
