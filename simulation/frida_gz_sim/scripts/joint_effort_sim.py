#!/usr/bin/env python3
"""Republishes /joint_states with arm joint efforts from the simulated wrist force/torque sensor.

The real xArm reports joint torques that the force guards watch; gz_ros2_control has no
measured effort for position-driven joints, so the external wrench at joint 6 is mapped to
joint torques (J^T w) from the TF chain. A force on the finger pads from door_sim.py
(/sim/door/virtual_wrench) is added the same way.
"""

import threading

import numpy as np
import rclpy
import tf2_ros
from gz.msgs10.wrench_pb2 import Wrench
from geometry_msgs.msg import WrenchStamped
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

ARM_BASE_FRAME = "link_base"
JOINTS = [f"joint{i}" for i in range(1, 7)]
JOINT_LINKS = [f"link{i}" for i in range(1, 7)]
SENSOR_LINK = "link6"
FINGERS = ("right_finger", "left_finger")
# Finger pads sit this far along the finger link Z from its origin
PAD_OFFSET = 0.06


class JointEffortSim(Node):
    def __init__(self):
        super().__init__("joint_effort_sim")
        self.input_topic = self.declare_parameter(
            "input_topic", "/joint_state_broadcaster/joint_states"
        ).value
        self.wrench_topic = self.declare_parameter(
            "wrench_topic", "/sim/wrist_ft"
        ).value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._lock = threading.Lock()
        # Force and torque on link 6 in its own frame
        self._wrench = np.zeros(6)
        self._efforts = np.zeros(len(JOINTS))
        # Force on the finger pads (base_link) from the emulated door grip
        self._pad_force = np.zeros(3)
        self.create_subscription(
            WrenchStamped, "/sim/door/virtual_wrench", self._on_pad_force, 10
        )
        self.gz = GzNode()
        self.gz.subscribe(Wrench, self.wrench_topic, self._on_wrench)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, self.input_topic, self._on_joints, 10)
        self.create_timer(0.02, self._update_efforts)
        self.get_logger().info(
            f"Joint efforts from {self.wrench_topic} onto /joint_states"
        )

    def _on_wrench(self, msg: Wrench):
        f, t = msg.force, msg.torque
        with self._lock:
            self._wrench = np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def _on_pad_force(self, msg: WrenchStamped):
        f = msg.wrench.force
        with self._lock:
            self._pad_force = np.array([f.x, f.y, f.z])

    def _transform(self, child: str, parent: str = ARM_BASE_FRAME):
        """(position, rotation) of ``child`` in the arm base frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, Time())
        except tf2_ros.TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])

    def _update_efforts(self):
        frames = [self._transform(link) for link in JOINT_LINKS]
        if any(frame is None for frame in frames):
            return
        with self._lock:
            wrench = self._wrench.copy()
            pad_force = self._pad_force.copy()
        sensor_position, sensor_rotation = frames[-1]
        force = sensor_rotation.apply(wrench[:3])
        torque = sensor_rotation.apply(wrench[3:])
        # Every xArm joint turns about its child link's z axis
        efforts = np.array(
            [
                rotation.apply([0.0, 0.0, 1.0]).dot(
                    torque + np.cross(sensor_position - position, force)
                )
                for position, rotation in frames
            ]
        )
        if pad_force.any():
            pads = self._pad_point()
            base = self._transform("base_link")
            if pads is not None and base is not None:
                pad_force = base[1].apply(pad_force)
                efforts += np.array(
                    [
                        rotation.apply([0.0, 0.0, 1.0]).dot(
                            np.cross(pads - position, pad_force)
                        )
                        for position, rotation in frames
                    ]
                )
        with self._lock:
            self._efforts = efforts

    def _pad_point(self):
        """Middle of the finger pads in the arm base frame, or None."""
        points = []
        for finger in FINGERS:
            frame = self._transform(finger)
            if frame is None:
                return None
            position, rotation = frame
            points.append(position + rotation.apply([0.0, 0.0, PAD_OFFSET]))
        return (points[0] + points[1]) / 2.0

    def _on_joints(self, msg: JointState):
        with self._lock:
            efforts = dict(zip(JOINTS, self._efforts))
        # Arm joints first, in order, as the xArm driver publishes them (the guards index efforts)
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        names = [n for n in JOINTS if n in positions] + [
            n for n in msg.name if n not in JOINTS
        ]
        msg.name = names
        msg.position = [float(positions[n]) for n in names]
        msg.velocity = [float(velocities.get(n, 0.0)) for n in names]
        msg.effort = [float(efforts.get(n, 0.0)) for n in names]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointEffortSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
