#!/usr/bin/env python3

"""End-to-end test: get the washing-machine drum centroid via moondream and
align the gripper to that height with joints 2 and 5.

Strategy (locked-joint, FK-based):
  * Pre-pose: `front_low_stare` (gripper already horizontal toward +X).
  * joints 1, 3, 4, 6: LOCKED at their pre-pose values.
  * joint2: solved so gripper_grasp_frame.z == centroid.z.
  * joint5: solved so the gripper approach axis stays horizontal.

The math lives in `task_manager.utils.xarm_fk` and is invoked through
`ManipulationTasks.align_to_centroid_height`.
"""

import sys

import rclpy
import tf2_ros
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point  # noqa: F401

from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.utils.xarm_fk import fk_grasp_frame

CAMERA_FLIP = False
PRE_POSE = "front_low_stare"
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
        # 1. Pre-pose
        Logger.info(self, f"Pre-positioning to {PRE_POSE}...")
        if (
            self.manipulation.move_joint_positions(named_position=PRE_POSE, velocity=0.3)
            != Status.EXECUTION_SUCCESS
        ):
            return 3

        # 2. FK vs TF sanity check at the pre-pose
        rclpy.spin_once(self, timeout_sec=0.5)
        current = self.manipulation.get_joint_positions()
        if isinstance(current, dict) and "joint1" in current:
            T = fk_grasp_frame(
                current["joint1"],
                current["joint2"],
                current["joint3"],
                current["joint4"],
                current["joint5"],
                current["joint6"],
            )
            fk_z = T[2, 3]
            try:
                tf = self._tf_buffer.lookup_transform(
                    "base_link",
                    WRIST_FRAME,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0),
                )
                tf_z = tf.transform.translation.z
            except Exception as e:
                tf_z = float("nan")
                Logger.warn(self, f"TF lookup failed at pre-pose: {e}")
            Logger.info(
                self,
                f"Pre-pose FK z={fk_z:.3f}, TF z={tf_z:.3f}, "
                f"gripper-z·base-z={T[2, 2]:+.3f} (should be ~0 if horizontal)",
            )

        # 3. Get the centroid from vision
        Logger.info(self, "Requesting washing-machine centroid from moondream...")
        point = self.vision.get_moondream_point_3d(SUBJECT)
        if point is None:
            Logger.error(self, "No centroid. Aborting.")
            return 1
        Logger.success(
            self,
            f"Centroid in {point.header.frame_id}: "
            f"({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f})",
        )

        # 4. Align using joints 2 and 5 only
        status = self.manipulation.align_to_centroid_height(point, pre_pose=PRE_POSE)
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"align_to_centroid_height failed: {status}")
            return 2

        # 5. Verify final wrist height vs target via TF
        rclpy.spin_once(self, timeout_sec=0.5)
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            point_base = do_transform_point(point, transform)
            cz = point_base.point.z
            tf_w = self._tf_buffer.lookup_transform(
                "base_link",
                WRIST_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            wz = tf_w.transform.translation.z
            Logger.info(
                self,
                f"Final wrist z={wz:.3f}, target z={cz:.3f}, " f"error={(wz - cz) * 100:+.1f} cm",
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
