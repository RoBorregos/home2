#!/usr/bin/env python3
"""Stands in for the xArm driver's mode and velocity API the pick pipeline drives directly.

Serves set_mode / set_state / vc_set_cartesian_velocity and publishes robot_states with
the TCP pose in the arm base frame. A cartesian velocity is integrated into a moving TCP
target, solved with MoveIt IK and streamed to the trajectory controller.
"""

import numpy as np
import rclpy
import tf2_ros
from builtin_interfaces.msg import Duration
from frida_constants.manipulation_constants import (
    XARM_MOVEVELOCITY_SERVICE,
    XARM_ROBOT_STATES_TOPIC,
    XARM_SETMODE_SERVICE,
    XARM_SETSTATE_SERVICE,
)
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveVelocity, SetInt16

JOINTS = [f"joint{i}" for i in range(1, 7)]
ARM_BASE_FRAME = "link_base"
TCP_FRAME = "link_eef"
CARTESIAN_VELOCITY_MODE = 5
IDLE_STATE = 2
RATE = 20.0
# An IK answer further than this from the current joints is a branch flip, not a step
MAX_JOINT_STEP = 0.35


class XArmVelocitySim(Node):
    def __init__(self):
        super().__init__("xarm_velocity_sim")
        group = ReentrantCallbackGroup()
        self.mode = 1
        self.velocity = np.zeros(3)
        self.target = None
        self.joints: dict[str, float] = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.states_pub = self.create_publisher(RobotMsg, XARM_ROBOT_STATES_TOPIC, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/xarm6_traj_controller/joint_trajectory", 10
        )
        self.ik = self.create_client(GetPositionIK, "/compute_ik", callback_group=group)
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10, callback_group=group
        )
        for name, handler in (
            (XARM_SETMODE_SERVICE, self._set_mode),
            (XARM_SETSTATE_SERVICE, self._ok),
            ("/xarm/config_tgpio_reset_when_stop", self._ok),
        ):
            self.create_service(SetInt16, name, handler, callback_group=group)
        self.create_service(
            MoveVelocity,
            "/xarm/vc_set_cartesian_velocity",
            self._set_cartesian_velocity,
            callback_group=group,
        )
        # Joint velocity mode (face following) is accepted but not simulated
        self.create_service(
            MoveVelocity, XARM_MOVEVELOCITY_SERVICE, self._ok, callback_group=group
        )
        # Wall clock on purpose: the pick pipeline times its approach with time.time()
        self.create_timer(
            1.0 / RATE, self._step, callback_group=group, clock=rclpy.clock.Clock()
        )
        self.get_logger().info("xArm velocity-mode emulator ready")

    def _on_joints(self, msg: JointState):
        self.joints.update(zip(msg.name, msg.position))

    def _ok(self, request, response):
        response.ret = 0
        return response

    def _set_mode(self, request, response):
        self.mode = int(request.data)
        if self.mode != CARTESIAN_VELOCITY_MODE:
            self.velocity = np.zeros(3)
            self.target = None
        response.ret = 0
        return response

    def _set_cartesian_velocity(self, request, response):
        if self.mode != CARTESIAN_VELOCITY_MODE:
            response.ret = 1
            response.message = "not in cartesian velocity mode"
            return response
        self.velocity = np.array(request.speeds[:3]) / 1000.0
        if not self.velocity.any():
            self.target = None
        response.ret = 0
        return response

    def tcp(self):
        """TCP pose in the arm base frame, as the xArm controller reports it."""
        try:
            return self.tf_buffer.lookup_transform(
                ARM_BASE_FRAME, TCP_FRAME, Time()
            ).transform
        except tf2_ros.TransformException:
            return None

    def _step(self):
        tcp = self.tcp()
        if tcp is None:
            return
        state = RobotMsg()
        state.header.stamp = self.get_clock().now().to_msg()
        state.mode = self.mode
        state.state = IDLE_STATE
        t = tcp.translation
        state.pose = [t.x * 1000.0, t.y * 1000.0, t.z * 1000.0, 0.0, 0.0, 0.0]
        self.states_pub.publish(state)
        if not self.velocity.any() or len(self.joints) < len(JOINTS):
            return
        if self.target is None:
            self.target = np.array([t.x, t.y, t.z])
        self.target = self.target + self.velocity / RATE
        self._move_to(self.target, tcp.rotation)

    def _move_to(self, position, orientation):
        if not self.ik.service_is_ready():
            return
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "xarm6"
        ik.ik_link_name = TCP_FRAME
        ik.avoid_collisions = False
        ik.robot_state.joint_state.name = JOINTS
        ik.robot_state.joint_state.position = [self.joints[j] for j in JOINTS]
        pose = PoseStamped()
        pose.header.frame_id = ARM_BASE_FRAME
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(
            float, position
        )
        pose.pose.orientation = orientation
        ik.pose_stamped = pose
        self.ik.call_async(request).add_done_callback(self._on_ik)

    def _on_ik(self, future):
        result = future.result()
        if result is None or result.error_code.val != 1 or not self.velocity.any():
            return
        solution = dict(
            zip(result.solution.joint_state.name, result.solution.joint_state.position)
        )
        positions = [solution[j] for j in JOINTS]
        step = max(abs(p - self.joints[j]) for p, j in zip(positions, JOINTS))
        if step > MAX_JOINT_STEP:
            self.get_logger().warn(
                f"IK branch flip ({step:.2f} rad), stopping the move"
            )
            self.velocity = np.zeros(3)
            return
        trajectory = JointTrajectory(joint_names=JOINTS)
        point = JointTrajectoryPoint(positions=positions)
        point.time_from_start = Duration(nanosec=int(2e9 / RATE))
        trajectory.points = [point]
        self.traj_pub.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = XArmVelocitySim()
    executor = MultiThreadedExecutor(4)
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
