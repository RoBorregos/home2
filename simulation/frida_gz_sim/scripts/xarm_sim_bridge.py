#!/usr/bin/env python3
"""Stands in for the xArm tool-GPIO gripper API on top of the gz gripper controller."""

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from frida_constants.manipulation_constants import (
    GRIPPER_GRASP_STATE_TOPIC,
    XARM_SET_DIGITAL_TGPIO_SERVICE,
)
from frida_interfaces.msg import GripperGraspState
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from xarm_msgs.srv import SetDigitalIO

FINGERS = ["rightfinger", "leftfinger"]


class XArmSimBridge(Node):
    def __init__(self):
        super().__init__("xarm_sim_bridge")
        group = ReentrantCallbackGroup()
        self.open_position = self.declare_parameter("open_position", 0.0).value
        self.closed_position = self.declare_parameter("closed_position", 0.056).value
        self.motion_time = self.declare_parameter("motion_time", 0.8).value
        # Finger stopped this far short of fully closed means something is in between
        self.grasp_margin = self.declare_parameter("grasp_margin", 0.006).value

        self.closed = False
        self.finger_pos = None
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/xarm_gripper_traj_controller/follow_joint_trajectory",
            callback_group=group,
        )
        self.create_service(
            SetDigitalIO,
            XARM_SET_DIGITAL_TGPIO_SERVICE,
            self._on_tgpio,
            callback_group=group,
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10, callback_group=group
        )
        self.grasp_pub = self.create_publisher(
            GripperGraspState, GRIPPER_GRASP_STATE_TOPIC, 10
        )
        # Consumed by grasp_attach.py when the attach fallback is enabled
        self.cmd_pub = self.create_publisher(Bool, "/sim/gripper_closed", 10)
        self.attached = False
        self.create_subscription(
            Bool, "/sim/grasp_attached", self._on_attached, 10, callback_group=group
        )
        self.create_timer(0.1, self._publish_grasp_state, callback_group=group)
        self.get_logger().info(f"Serving {XARM_SET_DIGITAL_TGPIO_SERVICE}")

    def _on_joints(self, msg: JointState):
        if FINGERS[0] in msg.name:
            self.finger_pos = msg.position[msg.name.index(FINGERS[0])]

    def _on_attached(self, msg: Bool):
        self.attached = msg.data
        if msg.data and self.finger_pos is not None:
            # Hold the fingers where they stopped; squeezing a welded object destabilizes DART
            self._send_fingers(self.finger_pos)

    def _send_fingers(self, target: float) -> bool:
        if not self._traj_client.wait_for_server(timeout_sec=2.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = FINGERS
        point = JointTrajectoryPoint()
        point.positions = [target, target]
        secs = int(self.motion_time)
        point.time_from_start = Duration(
            sec=secs, nanosec=int((self.motion_time - secs) * 1e9)
        )
        goal.trajectory.points = [point]
        self._traj_client.send_goal_async(goal)
        return True

    def _on_tgpio(self, request, response):
        close = int(request.value) == 1
        target = self.closed_position if close else self.open_position
        self.get_logger().info(
            f"Gripper {'close' if close else 'open'} -> {target:.3f}"
        )
        if close and self.attached:
            # Re-close while holding a welded object: keep the fingers where they are
            response.ret = 0
            response.message = "ok"
            return response
        self.closed = close
        self.cmd_pub.publish(Bool(data=close))
        if not self._send_fingers(target):
            response.ret = 1
            response.message = "gripper controller unavailable"
            return response
        response.ret = 0
        response.message = "ok"
        return response

    def _publish_grasp_state(self):
        detected = self.attached or (
            self.closed
            and self.finger_pos is not None
            and self.finger_pos < self.closed_position - self.grasp_margin
        )
        self.grasp_pub.publish(GripperGraspState(object_detected=bool(detected)))


def main(args=None):
    rclpy.init(args=args)
    node = XArmSimBridge()
    executor = MultiThreadedExecutor(3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
