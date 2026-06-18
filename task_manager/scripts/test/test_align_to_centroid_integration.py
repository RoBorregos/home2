#!/usr/bin/env python3

"""Integration test for `ManipulationTasks.align_to_centroid_height` that
does NOT require the moondream vision pipeline.

It injects a synthetic centroid `PointStamped` in `base_link` at several
heights and verifies, via TF readback of `gripper_grasp_frame`, that:
  1. wrist height matches the target z within ~2 cm,
  2. the gripper approach axis stays approximately horizontal,
  3. the locked joints (1, 3, 4, 6) didn't move from the pre-pose values.

Run inside the docker container with the xArm bringup active so
`/xarm/get_joints`, the `move_joints` action and TF are available.
"""

import math
import sys

import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

PRE_POSE = "front_low_stare"
WRIST_FRAME = "gripper_grasp_frame"
TEST_HEIGHTS_M = [0.35, 0.55, 0.85]
Z_TOL_M = 0.03
HORIZONTAL_TOL = 0.08
LOCKED_JOINT_TOL_RAD = 5e-3


class TestAlignToCentroidIntegration(Node):
    def __init__(self):
        super().__init__("test_align_to_centroid_integration")
        self.manipulation = ManipulationTasks(self, task=Task.DEBUG)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        rclpy.spin_once(self, timeout_sec=1.0)

    def _make_point(self, z: float) -> PointStamped:
        p = PointStamped()
        p.header.frame_id = "base_link"
        p.header.stamp = self.get_clock().now().to_msg()
        # x/y don't drive the solver (joint1 is locked), they're informational.
        p.point.x = 0.6
        p.point.y = 0.0
        p.point.z = float(z)
        return p

    def _run_case(self, z: float) -> bool:
        Logger.info(self, f"--- case: target z = {z:.3f} m ---")

        # Capture locked-joint reference at the pre-pose
        if (
            self.manipulation.move_joint_positions(named_position=PRE_POSE, velocity=0.3)
            != Status.EXECUTION_SUCCESS
        ):
            Logger.error(self, "Failed to reach pre-pose")
            return False
        rclpy.spin_once(self, timeout_sec=0.5)
        ref = self.manipulation.get_joint_positions()
        if not isinstance(ref, dict) or "joint1" not in ref:
            Logger.error(self, f"Bad joints at pre-pose: {ref}")
            return False
        ref_locked = {k: ref[k] for k in ("joint1", "joint3", "joint4", "joint6")}

        if (
            self.manipulation.align_to_centroid_height(self._make_point(z))
            != Status.EXECUTION_SUCCESS
        ):
            Logger.error(self, "align_to_centroid_height did not succeed")
            return False

        rclpy.spin_once(self, timeout_sec=0.5)

        # Check wrist height
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link",
                WRIST_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except Exception as e:
            Logger.error(self, f"TF base_link->{WRIST_FRAME} failed: {e}")
            return False
        wz = tf.transform.translation.z
        z_err = wz - z
        Logger.info(self, f"wrist z={wz:.3f}, err={z_err * 100:+.1f} cm")

        # Check horizontality: z component of gripper +Z expressed in base_link.
        # That's R[2,2] = 1 - 2*(qx^2 + qy^2) of the standard quat->matrix.
        q = tf.transform.rotation
        approach_z_in_base = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        Logger.info(self, f"gripper approach·base_z = {approach_z_in_base:+.3f}")

        # Check locked joints unchanged
        post = self.manipulation.get_joint_positions()
        locked_ok = True
        for k, v in ref_locked.items():
            delta = post.get(k, math.nan) - v
            if abs(delta) > LOCKED_JOINT_TOL_RAD:
                Logger.error(
                    self,
                    f"locked joint {k} moved by {delta:+.4f} rad (tol " f"{LOCKED_JOINT_TOL_RAD})",
                )
                locked_ok = False

        height_ok = abs(z_err) <= Z_TOL_M
        horiz_ok = abs(approach_z_in_base) <= HORIZONTAL_TOL
        ok = height_ok and horiz_ok and locked_ok
        Logger.info(
            self,
            f"case z={z:.3f} -> "
            f"height_ok={height_ok} horiz_ok={horiz_ok} locked_ok={locked_ok}",
        )
        return ok


def main():
    rclpy.init()
    node = TestAlignToCentroidIntegration()
    try:
        results = [node._run_case(z) for z in TEST_HEIGHTS_M]
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    passed = sum(results)
    print(f"\nINTEGRATION TEST: {passed}/{len(results)} cases passed")
    sys.exit(0 if passed == len(results) else 1)


if __name__ == "__main__":
    main()
