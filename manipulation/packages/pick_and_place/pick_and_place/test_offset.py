#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion
from frida_interfaces.action import OffsetMove
from frida_constants.manipulation_constants import OFFSET_MOVE_ACTION_SERVER

# Default offset distance in metres
DEFAULT_OFFSET = 0.3
DEFAULT_DIRECTION = OffsetMove.Goal.HORIZONTAL


def _unit(v: np.ndarray) -> np.ndarray:
    """Return a normalised copy of vector *v*."""
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def compute_grasp_orientation(from_xyz: np.ndarray, to_xyz: np.ndarray) -> Quaternion:
    """Return a quaternion that aligns the gripper Z-axis from *from_xyz* toward *to_xyz*.

    For xArm the ``gripper_grasp_frame`` Z-axis is the approach direction.
    We build an orthonormal frame (right, up, approach) and convert to
    quaternion using the standard rotation-matrix → quaternion formula.

    Parameters
    ----------
    from_xyz:
        The position the gripper will be placed at (the offset position).
    to_xyz:
        The target object position (the original clicked point).
    """
    approach = _unit(to_xyz - from_xyz)          # Z-axis of EEF → toward object

    # Choose a reference "up" that is not collinear with approach.
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(approach, world_up)) > 0.95:   # nearly vertical approach
        world_up = np.array([1.0, 0.0, 0.0])

    right = _unit(np.cross(world_up, approach))  # X-axis of EEF
    up    = np.cross(approach, right)             # Y-axis of EEF (recomputed for ortho)

    # 3×3 rotation matrix: columns are right, up, approach
    R = np.column_stack([right, up, approach])

    # Rotation matrix → quaternion (Shepperd method)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = Quaternion()
    q.x, q.y, q.z, q.w = float(x), float(y), float(z), float(w)
    return q


def offset_position(target: np.ndarray, direction: int, offset: float) -> np.ndarray:
    """Return the gripper approach position given the OffsetMove *direction*.

    Mirrors ``pick_server.compute_offset`` logic to know which side the
    server will push the pose toward, so we can compute the orientation
    *before* sending the goal (the server modifies position but keeps
    our orientation unchanged).
    """
    pos = target.copy()
    if direction == OffsetMove.Goal.UP:
        pos[2] += offset
    elif direction == OffsetMove.Goal.DOWN:
        pos[2] -= offset
    elif direction == OffsetMove.Goal.LEFT:
        pos[1] += offset
    elif direction == OffsetMove.Goal.RIGHT:
        pos[1] -= offset
    elif direction == OffsetMove.Goal.VERTICAL:
        if abs(pos[2] - offset) < abs(pos[2] + offset):
            pos[2] -= offset
        else:
            pos[2] += offset
    elif direction == OffsetMove.Goal.HORIZONTAL:
        if abs(pos[1] - offset) < abs(pos[1] + offset):
            pos[1] -= offset
        else:
            pos[1] += offset
    return pos


class TestOffsetMoveClient(Node):

    def __init__(self):
        super().__init__("test_offset_move_client")

        self._client = ActionClient(
            self,
            OffsetMove,
            OFFSET_MOVE_ACTION_SERVER
        )

        self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            10
        )

        self.get_logger().info(
            f"Waiting for clicked points in RViz… "
            f"(offset={DEFAULT_OFFSET} m, direction={DEFAULT_DIRECTION})"
        )

    def clicked_point_callback(self, msg: PointStamped):

        self.get_logger().info(
            f"Clicked point received: "
            f"x={msg.point.x:.3f}, "
            f"y={msg.point.y:.3f}, "
            f"z={msg.point.z:.3f}"
        )

        target = np.array([msg.point.x, msg.point.y, msg.point.z])

        # Predict where the server will place the gripper after applying offset
        approach_pos = offset_position(target, DEFAULT_DIRECTION, DEFAULT_OFFSET)

        self.get_logger().info(
            f"Computed approach position: "
            f"x={approach_pos[0]:.3f}, "
            f"y={approach_pos[1]:.3f}, "
            f"z={approach_pos[2]:.3f}"
        )

        # Orientation: gripper Z-axis points from approach_pos toward target
        orientation = compute_grasp_orientation(approach_pos, target)

        self.get_logger().info(
            f"Computed orientation (xyzw): "
            f"{orientation.x:.3f}, {orientation.y:.3f}, "
            f"{orientation.z:.3f}, {orientation.w:.3f}"
        )

        # Build PoseStamped with the TARGET position (server will add the offset)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = msg.point.z
        pose.pose.orientation = orientation

        self.send_goal(pose)

    def send_goal(self, pose: PoseStamped):

        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        goal = OffsetMove.Goal()
        goal.pose = pose
        goal.offset = DEFAULT_OFFSET
        goal.direction = DEFAULT_DIRECTION

        self.get_logger().warning("Sending OffsetMove goal...")

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("OffsetMove succeeded ✓")
        else:
            self.get_logger().error("OffsetMove FAILED ✗")

    def feedback_callback(self, feedback_msg):
        self.get_logger().info("Feedback received")


def main(args=None):
    rclpy.init(args=args)

    node = TestOffsetMoveClient()

    try:
        rclpy.spin(node)   # 🔥 IMPORTANTE
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()