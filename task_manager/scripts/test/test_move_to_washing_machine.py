#!/usr/bin/env python3

"""End-to-end test: locate the washing-machine drum opening with moondream
from a LOOK pose where the ZED actually sees it, then move to a separate
REACH pose where the arm can physically extend down to the drum height.

Strategy (split look + reach):
  1. Pre-pose to `LOOK_POSE` — gripper high, ZED looks at the drum cleanly.
  2. Call moondream → bbox → ROI depth → 3D centroid in the camera frame.
  3. TF the centroid into `base_link` IMMEDIATELY (world-stable). The
     ZED moves with the gripper, so once we move the arm again the camera's
     view of the drum is gone — but the cached world point is invariant.
  4. Hand the cached `base_link` point to `align_to_centroid_height(...,
     pre_pose=REACH_POSE)`. That call moves the arm to the reach pose, runs
     FK calibration vs TF, solves j2/j5 (locking j3/j4/j6 at the reach
     pose values), and executes. The camera is irrelevant during the reach.

joints 1, 3, 4, 6 are LOCKED at the reach pose values; joints 2 and 5 are
the only ones the solver moves. The math lives in `task_manager.utils.xarm_fk`.
"""

import sys

import rclpy
import tf2_ros
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point

from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

CAMERA_FLIP = False
# Pose where the ZED can actually see the drum cleanly enough for moondream
# to return a meaningful bbox. (front_low_stare keeps the camera looking +X
# at the scene with the gripper roughly horizontal.)
LOOK_POSE = "front_low_stare"
# Pose used as the LOCK reference for the reach. j3 is opened up so the
# wrist-z range covers low targets; gripper ends up pointing ~horizontal +X.
REACH_POSE = "washing_machine_reach_pose"
WRIST_FRAME = "gripper_grasp_frame"
SUBJECT = (
    "exact geometric center of the circular washing machine drum opening "
    "(the round hole in the front door where clothes go in); point at the "
    "middle of the circle, not the door, rim, glass, or surrounding frame"
)


class TestMoveToWashingMachine(Node):
    def __init__(self):
        super().__init__("test_move_to_washing_machine")
        self.vision = VisionTasks(self, task=Task.DEBUG)
        self.manipulation = ManipulationTasks(self, task=Task.DEBUG)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.vision.camera_upside_down(CAMERA_FLIP)

    def run(self) -> int:
        # 1. Look pose: where the camera can see the drum.
        Logger.info(self, f"Moving to LOOK pose '{LOOK_POSE}'...")
        if (
            self.manipulation.move_joint_positions(
                named_position=LOOK_POSE, velocity=0.3
            )
            != Status.EXECUTION_SUCCESS
        ):
            Logger.error(self, "Failed to reach LOOK pose. Aborting.")
            return 3

        # 2. Centroid via moondream (bbox → ROI → mean of pointcloud).
        Logger.info(self, "Requesting washing-machine centroid from moondream...")
        point = self.vision.get_moondream_point_3d(SUBJECT)
        if point is None:
            Logger.error(self, "No centroid returned. Aborting.")
            return 1
        Logger.success(
            self,
            f"Centroid in {point.header.frame_id}: "
            f"({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f})",
        )

        # 3. TF the centroid to base_link RIGHT NOW (camera still in look pose).
        # Once we move to REACH_POSE the ZED's view is gone, so we can't
        # re-derive the world point — we use this cached one for the solver
        # AND for the final verification.
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except Exception as e:
            Logger.error(self, f"TF {point.header.frame_id} -> base_link failed: {e}")
            return 4
        point_in_base = do_transform_point(point, transform)
        # Re-stamp the frame so align_to_centroid_height's internal TF lookup
        # is just identity (base_link -> base_link) and doesn't pick up the
        # post-move ZED transform.
        point_in_base.header.frame_id = "base_link"
        cached_cz = point_in_base.point.z
        Logger.info(
            self,
            f"Cached centroid in base_link: "
            f"({point_in_base.point.x:.3f}, {point_in_base.point.y:.3f}, "
            f"{cached_cz:.3f})",
        )

        # 4. Reach: move to REACH_POSE and solve j2/j5 to hit the cached z.
        Logger.info(
            self, f"Aligning to centroid via REACH pose '{REACH_POSE}'..."
        )
        status = self.manipulation.align_to_centroid_height(
            point_in_base, pre_pose=REACH_POSE
        )
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"align_to_centroid_height failed: {status}")
            return 2

        # 5. Verify final wrist height against the cached target.
        rclpy.spin_once(self, timeout_sec=0.5)
        try:
            tf_w = self._tf_buffer.lookup_transform(
                "base_link",
                WRIST_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            wz = tf_w.transform.translation.z
            Logger.info(
                self,
                f"Final wrist z={wz:.3f}, target z={cached_cz:.3f} (cached), "
                f"error={(wz - cached_cz) * 100:+.1f} cm",
            )
        except Exception as e:
            Logger.warn(self, f"Final TF verification failed: {e}")

        Logger.success(self, "Done.")
        return 0


def main():
    rclpy.init()
    node = TestMoveToWashingMachine()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
