#!/usr/bin/env python3
"""Aim the xArm6 camera at the Nav2 destination while the base drives there.

Listens to the nav goal published by nav_central (/nav/current_goal, frame map)
and, while /nav/goal_active is True, continuously re-orients the camera to look
at that destination via the MoveToPose action. Stops (keeps last pose) when the
goal becomes inactive. Adapted from examples/look_at_example.py.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frida_interfaces.action import MoveToPose
from frida_motion_planning.utils.tf_utils import look_at
from frida_pymoveit2.robots import xarm6

MAP_FRAME = "map"
TRACK_PERIOD = 1.0 / 1.5  # ~1.5 Hz
MIN_DISTANCE = 0.05  # m; below this look_at would normalize a ~0 vector -> NaN
VELOCITY = 0.3
BUSY_TIMEOUT = 5.0  # s; if a MoveToPose never returns, recover and re-send


class NavGoalArmPointer(Node):
    def __init__(self):
        super().__init__("nav_goal_arm_pointer")
        self.callback_group = ReentrantCallbackGroup()

        self.goal = None  # PoseStamped in map frame
        self.goal_active = False
        self.busy = False  # an arm MoveToPose is in flight
        self.busy_since = 0.0  # monotonic time the in-flight goal started

        # transient_local to match nav_central's latched publishers, so we get
        # the last goal even if we start after it was published.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseStamped,
            "/nav/current_goal",
            self._goal_cb,
            latched,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            "/nav/goal_active",
            self._active_cb,
            latched,
            callback_group=self.callback_group,
        )

        self.debug_pub = self.create_publisher(
            PoseStamped, "/nav_goal_arm_pointer/debug_pose", 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_client = ActionClient(
            self,
            MoveToPose,
            "/manipulation/move_to_pose_action_server",
            callback_group=self.callback_group,
        )

        self.create_timer(TRACK_PERIOD, self._track, callback_group=self.callback_group)
        self.get_logger().info("nav_goal_arm_pointer ready")

    def _goal_cb(self, msg: PoseStamped):
        self.goal = msg

    def _active_cb(self, msg: Bool):
        self.goal_active = msg.data

    def _track(self):
        if not self.goal_active or self.goal is None:
            return
        if self.busy:
            # Recover from a goal whose result never came back, else tracking
            # would stall for the rest of the run.
            if time.monotonic() - self.busy_since < BUSY_TIMEOUT:
                return
            self.get_logger().warn("arm goal timed out, re-sending")
            self.busy = False

        # Camera pose in map frame.
        camera_frame = xarm6.camera_frame_name()
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                camera_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5),
            )
        except TransformException as e:
            self.get_logger().warn(f"TF {MAP_FRAME}->{camera_frame} unavailable: {e}")
            return

        cam_x = tf.transform.translation.x
        cam_y = tf.transform.translation.y
        cam_z = tf.transform.translation.z

        camera_pose = PoseStamped()
        camera_pose.header.frame_id = MAP_FRAME
        camera_pose.header.stamp = self.get_clock().now().to_msg()
        camera_pose.pose.position.x = cam_x
        camera_pose.pose.position.y = cam_y
        camera_pose.pose.position.z = cam_z

        # Target = goal x,y at camera height (aim horizontally, not at the floor).
        target_pose = PoseStamped()
        target_pose.header.frame_id = MAP_FRAME
        target_pose.header.stamp = camera_pose.header.stamp
        target_pose.pose.position.x = self.goal.pose.position.x
        target_pose.pose.position.y = self.goal.pose.position.y
        target_pose.pose.position.z = cam_z

        # Skip if camera is basically on top of the destination (avoids NaN).
        if (
            math.hypot(
                target_pose.pose.position.x - cam_x, target_pose.pose.position.y - cam_y
            )
            < MIN_DISTANCE
        ):
            return

        try:
            aimed = look_at(camera_pose, target_pose)
        except Exception as e:
            self.get_logger().warn(f"look_at failed: {e}")
            return

        self.debug_pub.publish(aimed)

        goal = MoveToPose.Goal()
        goal.pose = aimed
        goal.velocity = VELOCITY
        goal.target_link = camera_frame

        if not self._action_client.server_is_ready():
            self.get_logger().warn("move_to_pose action server not ready")
            return

        self.busy = True
        self.busy_since = time.monotonic()
        self._action_client.send_goal_async(goal).add_done_callback(self._sent_cb)

    def _sent_cb(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"send goal failed: {e}")
            self.busy = False
            return
        if not handle.accepted:
            self.get_logger().warn("arm goal rejected")
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().warn(f"arm goal errored: {e}")
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(2)
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
