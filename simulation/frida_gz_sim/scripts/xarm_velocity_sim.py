#!/usr/bin/env python3
"""Stands in for the xArm driver's mode and velocity API the pick pipeline drives directly.

Serves set_mode / set_state / vc_set_cartesian_velocity and publishes robot_states with
the TCP pose in the arm base frame. A cartesian velocity becomes a straight MoveIt path
along that direction, timed at the commanded speed and sent to the trajectory controller
until the velocity is zeroed or the mode changes.
"""

import time

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
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
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
# Longest straight line planned for one velocity command (m) and its resolution
MAX_TRAVEL = 0.30
PATH_STEP = 0.005


class XArmVelocitySim(Node):
    def __init__(self):
        super().__init__("xarm_velocity_sim")
        group = ReentrantCallbackGroup()
        self.mode = 1
        # Sim seconds per wall second, so commanded speeds hold in wall time
        self.rtf = 1.0
        self._last_tick = None
        self.velocity = np.zeros(3)
        self.joints: dict[str, float] = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.states_pub = self.create_publisher(RobotMsg, XARM_ROBOT_STATES_TOPIC, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/xarm6_traj_controller/joint_trajectory", 10
        )
        self.path = self.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=group
        )
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
        # Wall clock: robot_states must keep flowing whatever the real-time factor
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
        if self.mode != CARTESIAN_VELOCITY_MODE and self.velocity.any():
            self.velocity = np.zeros(3)
            self._hold()
        response.ret = 0
        return response

    def _set_cartesian_velocity(self, request, response):
        if self.mode != CARTESIAN_VELOCITY_MODE:
            self.get_logger().warn(f"Velocity command ignored in mode {self.mode}")
            response.ret = 1
            response.message = "not in cartesian velocity mode"
            return response
        self.velocity = np.array(request.speeds[:3]) / 1000.0
        if self.velocity.any():
            self._start_line()
        else:
            self._hold()
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

    def _update_rtf(self):
        tick = (time.monotonic(), self.get_clock().now().nanoseconds / 1e9)
        if self._last_tick is not None and tick[0] > self._last_tick[0]:
            rtf = (tick[1] - self._last_tick[1]) / (tick[0] - self._last_tick[0])
            if 0.05 < rtf < 2.0:
                self.rtf = 0.9 * self.rtf + 0.1 * rtf
        self._last_tick = tick

    def _step(self):
        self._update_rtf()
        tcp = self.tcp()
        if tcp is None:
            return
        state = RobotMsg()
        state.header.stamp = self.get_clock().now().to_msg()
        state.mode = self.mode
        state.state = IDLE_STATE
        t = tcp.translation
        state.pose = [t.x * 1000.0, t.y * 1000.0, t.z * 1000.0, 0.0, 0.0, 0.0]
        state.angle = [float(self.joints.get(j, 0.0)) for j in JOINTS]
        self.states_pub.publish(state)

    def _start_line(self):
        """Plan a straight TCP line along the commanded velocity and stream it."""
        tcp = self.tcp()
        # The pick pipeline times its moves on the wall clock; the controller runs on sim time
        speed = float(np.linalg.norm(self.velocity)) / self.rtf
        if (
            tcp is None
            or len(self.joints) < len(JOINTS)
            or not self.path.service_is_ready()
        ):
            self.get_logger().warn("Cannot start a cartesian velocity move yet")
            return
        direction = self.velocity / np.linalg.norm(self.velocity)
        request = GetCartesianPath.Request()
        request.header.frame_id = ARM_BASE_FRAME
        request.group_name = "xarm6"
        request.link_name = TCP_FRAME
        request.start_state.joint_state.name = JOINTS
        request.start_state.joint_state.position = [self.joints[j] for j in JOINTS]
        goal = Pose(orientation=tcp.rotation)
        end = np.array([tcp.translation.x, tcp.translation.y, tcp.translation.z])
        end = end + direction * MAX_TRAVEL
        goal.position.x, goal.position.y, goal.position.z = map(float, end)
        request.waypoints = [goal]
        request.max_step = PATH_STEP
        request.avoid_collisions = False
        future = self.path.call_async(request)
        future.add_done_callback(lambda f: self._on_path(f, speed))

    def _on_path(self, future, speed: float):
        result = future.result()
        if result is None or not self.velocity.any():
            return
        points = result.solution.joint_trajectory.points
        if len(points) < 2:
            self.get_logger().warn("No straight-line path for the velocity command")
            return
        self.get_logger().info(
            f"Straight line: {len(points)} points, {result.fraction * MAX_TRAVEL * 1000:.0f} mm "
            f"at {speed * 1000:.0f} mm/s sim (rtf {self.rtf:.2f})"
        )
        names = result.solution.joint_trajectory.joint_names
        order = [names.index(j) for j in JOINTS]
        travelled = result.fraction * MAX_TRAVEL
        trajectory = JointTrajectory(joint_names=JOINTS)
        for i, point in enumerate(points[1:], start=1):
            seconds = travelled * i / (len(points) - 1) / speed
            step = JointTrajectoryPoint(positions=[point.positions[k] for k in order])
            step.time_from_start = Duration(
                sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9)
            )
            trajectory.points.append(step)
        self.traj_pub.publish(trajectory)

    def _hold(self):
        """Stop where the arm is."""
        if len(self.joints) < len(JOINTS):
            return
        point = JointTrajectoryPoint(positions=[self.joints[j] for j in JOINTS])
        point.time_from_start = Duration(nanosec=int(1e8))
        self.traj_pub.publish(JointTrajectory(joint_names=JOINTS, points=[point]))


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
