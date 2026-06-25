#!/usr/bin/env python3

"""End-to-end test for the arrow-shape washing-machine alignment + insert.

Look pose -> moondream bbox+ROI centroid -> snapshot in base_link -> arrow align.
The ZED travels with the arm, so the centroid is snapshotted in `base_link`
BEFORE moving — that frame is the only one that's still valid after the move.

Pass `--insert` to also exercise the base insert sequence in isolation:
compute the forward push from the aligned gripper TF, drive the base in
(odom-measured RelativeMove), rotate the wrist down, close, rotate up, and
drive the base back out the same distance.
"""

import sys

import rclpy
import tf2_ros
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point

from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.nav_tasks import NavigationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

LOOK_POSE = "front_low_stare"
ARROW_POSE = "washing_machine_arrow_pose"
WRIST_FRAME = "gripper_grasp_frame"
SUBJECT = "round container entrance of color orange"

# Insert-sequence parameters (keep in sync with doing_laundry_task_manager.py).
INSERT_DEPTH = 0.10
MAX_INSERT = 0.40
LOOKDOWN_DEG = 90.0


class TestMoveToWashingMachine(Node):
    def __init__(self, do_insert: bool = False):
        super().__init__("test_move_to_washing_machine")
        self.do_insert = do_insert
        self.vision = VisionTasks(self, task=Task.DEBUG)
        self.manipulation = ManipulationTasks(self, task=Task.DEBUG)
        self.nav = NavigationTasks(self, task=Task.DEBUG) if do_insert else None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.vision.camera_upside_down(False)

    def insert_sequence(self, opening) -> int:
        """Drive the base into the drum, grab, and back out. `opening` is the
        cached PointStamped in base_link. Returns 0 on success, nonzero on error."""
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        try:
            g = self._tf_buffer.lookup_transform(
                "base_link",
                WRIST_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            ).transform.translation
        except Exception as e:
            Logger.error(self, f"TF base_link <- {WRIST_FRAME} failed: {e}")
            return 5
        forward = (opening.point.x - g.x) + INSERT_DEPTH
        forward = max(0.0, min(forward, MAX_INSERT))
        Logger.info(
            self,
            f"gripper x={g.x:.3f}, opening x={opening.point.x:.3f} -> push {forward:.3f} m",
        )

        self.manipulation.open_gripper()
        status, error = self.nav.move_relative(forward, backward=False)
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"forward insert failed: {error}")
            return 6

        self.manipulation.rotate_wrist_pitch(LOOKDOWN_DEG)
        self.manipulation.close_gripper()
        self.manipulation.rotate_wrist_pitch(-LOOKDOWN_DEG)

        status, error = self.nav.move_relative(forward, backward=True)
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"retreat failed: {error}")
            return 7

        Logger.success(self, "Insert sequence complete.")
        return 0

    def run(self) -> int:
        Logger.info(self, f"Moving to LOOK pose '{LOOK_POSE}'...")
        if (
            self.manipulation.move_joint_positions(named_position=LOOK_POSE, velocity=0.3)
            != Status.EXECUTION_SUCCESS
        ):
            Logger.error(self, "Failed to reach LOOK pose")
            return 3

        Logger.info(self, "Requesting centroid from moondream...")
        point = self.vision.get_moondream_point_3d(SUBJECT)
        if point is None:
            Logger.error(self, "No centroid returned")
            return 1
        p = point.point
        Logger.success(
            self, f"Centroid in {point.header.frame_id}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        )

        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link",
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except Exception as e:
            Logger.error(self, f"TF {point.header.frame_id} -> base_link failed: {e}")
            return 4
        point_in_base = do_transform_point(point, tf)
        point_in_base.header.frame_id = "base_link"
        pb = point_in_base.point
        Logger.info(self, f"Cached centroid in base_link: ({pb.x:.3f}, {pb.y:.3f}, {pb.z:.3f})")

        Logger.info(self, f"Pointing arrow at centroid via '{ARROW_POSE}'...")
        if (
            self.manipulation.align_arm_toward_centroid(point_in_base, pre_pose=ARROW_POSE)
            != Status.EXECUTION_SUCCESS
        ):
            Logger.error(self, "align_arm_toward_centroid failed")
            return 2

        rclpy.spin_once(self, timeout_sec=0.5)
        try:
            w = self._tf_buffer.lookup_transform(
                "base_link",
                WRIST_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            ).transform.translation
            Logger.info(
                self,
                f"Final wrist=({w.x:.3f}, {w.y:.3f}, {w.z:.3f}); centroid z={pb.z:.3f}",
            )
        except Exception as e:
            Logger.warn(self, f"Final TF verification failed: {e}")

        if self.do_insert:
            Logger.info(self, "Running base insert sequence...")
            rc = self.insert_sequence(point_in_base)
            if rc != 0:
                return rc

        Logger.success(self, "Done.")
        return 0


def main():
    do_insert = "--insert" in sys.argv[1:]
    rclpy.init()
    node = TestMoveToWashingMachine(do_insert=do_insert)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
