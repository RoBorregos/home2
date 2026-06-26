#!/usr/bin/env python3
"""Yaw the xArm6 (joint 1 only) to point at the Nav2 destination as the base drives.

While /nav/goal_active is True, computes the bearing from the arm base to the nav
goal and drives joint 1 toward it with direct xArm joint-velocity commands (the
same API follow_face uses), holding all other joints. No MoveIt / motion planning.
When the goal goes inactive, hands the arm back to MoveIt and sends it to the named
NAV_POSE (reliable front-facing pose) instead of velocity-homing joint1 by angle.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose

from frida_constants.manipulation_constants import (
    MOVEIT_MODE,
    JOINT_VELOCITY_MODE,
    XARM_SETMODE_SERVICE,
    XARM_SETSTATE_SERVICE,
    XARM_MOVEVELOCITY_SERVICE,
)
from frida_constants.xarm_configurations import NAV_POSE
from frida_interfaces.action import MoveJoints
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_pymoveit2.robots import xarm6
from xarm_msgs.srv import MoveVelocity, SetInt16

BASE_FRAME = xarm6.base_link_name()  # "link_base"
JOINT1_NAME = xarm6.joint_names()[0]  # "joint1"
JOINT1_LIMIT = math.pi * 0.99  # rad, matches xarm6 joint1 soft limit

CONTROL_PERIOD = 0.1  # s (10 Hz)
KP = 1.0  # rad/s per rad of error
MAX_JOINT1_VEL = 0.5  # rad/s cap
ANGLE_TOL = 0.02  # rad deadband (~1.1 deg) -> hold still
VEL_DURATION = 0.3  # s; xArm auto-stops if we stop sending (safety if node dies)

# ponytail: tracking calibration. JOINT1_SIGN flips rotation direction;
# JOINT1_OFFSET aligns joint1's zero with the arm-base +x. 0.0 = tracking points
# correctly. (The post-nav front pose is no longer tuned here -- see NAV_POSE.)
JOINT1_SIGN = 1.0
JOINT1_OFFSET = 0.0

# When the goal ends, instead of velocity-homing joint1 by angle (imprecise), hand
# the arm to MoveIt and send it to the named NAV_POSE so it reliably faces front.
MOVE_JOINTS_ACTION = "/manipulation/move_joints_action_server"
NAV_POSE_VELOCITY = 0.75  # matches move_to_position default
NAV_POSE_NAMES = list(NAV_POSE["joints"].keys())
NAV_POSE_RADS = [
    math.radians(v) if NAV_POSE.get("degrees") else float(v)
    for v in NAV_POSE["joints"].values()
]


def wrap_angle(a: float) -> float:
    """Wrap to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def joint1_velocity(target: float, current: float) -> float:
    """P controller for joint 1: returns clamped velocity, 0 inside the deadband."""
    err = wrap_angle(target - current)
    if abs(err) < ANGLE_TOL:
        return 0.0
    return max(-MAX_JOINT1_VEL, min(MAX_JOINT1_VEL, KP * err))


class NavGoalArmPointer(Node):
    def __init__(self):
        super().__init__("nav_goal_arm_pointer")
        cbg = ReentrantCallbackGroup()

        self.goal = None  # PoseStamped (map)
        self.goal_active = False
        self.joint1_pos = None  # rad, from /joint_states
        self.vel_mode_on = False  # whether the arm is currently in velocity mode
        self.returning = False  # sending the arm to NAV_POSE after the goal ends
        self.point_attempted = False  # tried to grab velocity mode for this goal

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # True = arm idle / back to its normal pose; False = pointing or returning.
        # Latched so a late subscriber (nav_central) reads the current value.
        self.ready_pub = self.create_publisher(Bool, "/nav/arm_ready", latched)
        self.ready_pub.publish(Bool(data=True))
        self.create_subscription(
            PoseStamped, "/nav/current_goal", self._goal_cb, latched, callback_group=cbg
        )
        self.create_subscription(
            Bool, "/nav/goal_active", self._active_cb, latched, callback_group=cbg
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_cb, 10, callback_group=cbg
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.mode_client = self.create_client(
            SetInt16, XARM_SETMODE_SERVICE, callback_group=cbg
        )
        self.state_client = self.create_client(
            SetInt16, XARM_SETSTATE_SERVICE, callback_group=cbg
        )
        self.vel_client = self.create_client(
            MoveVelocity, XARM_MOVEVELOCITY_SERVICE, callback_group=cbg
        )
        self.move_joints_client = ActionClient(
            self, MoveJoints, MOVE_JOINTS_ACTION, callback_group=cbg
        )

        for c, n in (
            (self.mode_client, "set_mode"),
            (self.state_client, "set_state"),
            (self.vel_client, "vc_set_joint_velocity"),
        ):
            if not c.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f"xArm service {n} not available")

        self.create_timer(CONTROL_PERIOD, self._control, callback_group=cbg)
        self.get_logger().info("nav_goal_arm_pointer ready (joint1 velocity control)")

    def _goal_cb(self, msg):
        self.goal = msg

    def _active_cb(self, msg):
        self.goal_active = msg.data

    def _joint_cb(self, msg):
        if JOINT1_NAME in msg.name:
            self.joint1_pos = msg.position[msg.name.index(JOINT1_NAME)]

    # -- xArm mode --

    def _set_mode(self, mode: int) -> bool:
        """Set xArm mode then state 0 (active). Blocking, best-effort."""
        ok = wait_for_future(self.mode_client.call_async(SetInt16.Request(data=mode)))
        ok = ok and wait_for_future(
            self.state_client.call_async(SetInt16.Request(data=0))
        )
        if not ok:
            self.get_logger().error(f"Failed to set xArm mode {mode}")
        return bool(ok)

    def _send_joint1_vel(self, vel: float):
        req = MoveVelocity.Request()
        req.is_sync = True
        req.duration = VEL_DURATION
        req.speeds = [vel, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vel_client.call_async(req)

    # -- return to NAV_POSE (MoveIt) when the goal ends --

    def _arm_ready(self):
        """Mark the arm back to its normal pose so nav can finish. Idempotent."""
        if self.returning:
            self.returning = False
            self.ready_pub.publish(Bool(data=True))

    def _send_nav_pose(self):
        """Plan + move to the named NAV_POSE via MoveIt. Non-blocking."""
        if not self.move_joints_client.server_is_ready():
            self.get_logger().warn(
                "move_joints action server unavailable; skipping NAV_POSE"
            )
            self._arm_ready()  # don't block nav waiting for an arm that can't move
            return
        goal = MoveJoints.Goal()
        goal.joint_names = NAV_POSE_NAMES
        goal.joint_positions = NAV_POSE_RADS
        goal.velocity = NAV_POSE_VELOCITY
        self.move_joints_client.send_goal_async(goal).add_done_callback(
            self._nav_pose_sent
        )

    def _nav_pose_sent(self, future):
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn("NAV_POSE goal rejected")
            self._arm_ready()
            return
        gh.get_result_async().add_done_callback(lambda _f: self._nav_pose_done())

    def _nav_pose_done(self):
        self.get_logger().info("arm returned to NAV_POSE")
        self._arm_ready()

    # -- control loop --

    def _control(self):
        # Activation / deactivation edges.
        if self.goal_active:
            # Try to take velocity control ONCE per goal. Retrying every tick
            # fights ros2_control for /xarm/set_mode and storms "Failed to set
            # xArm mode 4" when manipulation owns the arm. This assumes
            # manipulation doesn't move the arm during navigation; real fix is
            # node coordination (see _send_nav_pose / module TODO).
            if not self.vel_mode_on and not self.point_attempted:
                self.point_attempted = True
                if self._set_mode(JOINT_VELOCITY_MODE):
                    self.vel_mode_on = True
                    self.ready_pub.publish(Bool(data=False))  # busy: pointing
                else:
                    self.get_logger().warn(
                        "could not take velocity mode (arm busy?); not pointing this goal"
                    )
            self.returning = False  # a new goal preempts an in-flight NAV_POSE return
        else:
            self.point_attempted = False  # reset for the next goal
            if self.vel_mode_on:
                # Goal ended: stop pointing, hand to MoveIt, send arm to NAV_POSE.
                self._send_joint1_vel(0.0)
                self._set_mode(MOVEIT_MODE)
                self.vel_mode_on = False
                self.returning = True
                self._send_nav_pose()

        if not self.vel_mode_on or self.joint1_pos is None:
            return

        if self.goal is None:
            return

        # Goal (map) -> arm base frame, then bearing in the base plane.
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                self.goal.header.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF {BASE_FRAME}->{self.goal.header.frame_id} unavailable "
                f"(localization not bridged to the arm?): {e}"
            )
            return
        g = do_transform_pose(self.goal.pose, tf)
        bearing = math.atan2(g.position.y, g.position.x)

        target = JOINT1_SIGN * bearing + JOINT1_OFFSET
        target = max(-JOINT1_LIMIT, min(JOINT1_LIMIT, wrap_angle(target)))
        self._send_joint1_vel(joint1_velocity(target, self.joint1_pos))


def _selftest():
    assert abs(wrap_angle(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-6
    assert joint1_velocity(0.0, 0.0) == 0.0  # on target -> stop
    assert joint1_velocity(1.0, 0.0) > 0.0  # need +rotation
    assert joint1_velocity(-1.0, 0.0) < 0.0  # need -rotation
    assert abs(joint1_velocity(10.0, 0.0)) == MAX_JOINT1_VEL  # clamped
    # shortest path across the wrap: from near -pi, target near +pi -> go negative
    assert joint1_velocity(math.pi - 0.05, -math.pi + 0.05) < 0.0
    print("selftest OK")


def main(args=None):
    if "--selftest" in sys.argv:
        _selftest()
        return
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(3)
    node = NavGoalArmPointer()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
