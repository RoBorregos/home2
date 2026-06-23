#!/usr/bin/env python3

"""End-to-end test for the arrow-shape washing-machine alignment.

Flow:
  1. Move to LOOK_POSE so the ZED can see the drum.
  2. Get the moondream-bbox + depth-ROI centroid in the camera frame.
  3. Snapshot it in `base_link` BEFORE moving (the ZED travels with the arm).
  4. Call `align_arm_toward_centroid` to point j1/j2/j5 at that cached point.
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

LOOK_POSE = "front_low_stare"
ARROW_POSE = "washing_machine_arrow_pose"
WRIST_FRAME = "gripper_grasp_frame"
SUBJECT = "round container entrance of color orange"  # moondream bbox prompt


class TestMoveToWashingMachine(Node):
    def __init__(self):
        super().__init__("test_move_to_washing_machine")
        self.vision = VisionTasks(self, task=Task.DEBUG)
        self.manipulation = ManipulationTasks(self, task=Task.DEBUG)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.vision.camera_upside_down(False)

    def run(self) -> int:
        Logger.info(self, f"Moving to LOOK pose '{LOOK_POSE}'...")
        if (
            self.manipulation.move_joint_positions(named_position=LOOK_POSE, velocity=0.3)
            != Status.EXECUTION_SUCCESS
        ):
            Logger.error(self, "Failed to reach LOOK pose. Aborting.")
            return 3

        Logger.info(self, "Requesting centroid from moondream...")
        point = self.vision.get_moondream_point_3d(SUBJECT)
        if point is None:
            Logger.error(self, "No centroid returned. Aborting.")
            return 1
        Logger.success(
            self,
            f"Centroid in {point.header.frame_id}: "
            f"({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f})",
        )

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
        point_in_base.header.frame_id = "base_link"
        Logger.info(
            self,
            f"Cached centroid in base_link: "
            f"({point_in_base.point.x:.3f}, {point_in_base.point.y:.3f}, "
            f"{point_in_base.point.z:.3f})",
        )

        Logger.info(self, f"Pointing arrow at centroid via '{ARROW_POSE}'...")
        status = self.manipulation.align_arm_toward_centroid(point_in_base, pre_pose=ARROW_POSE)
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"align_arm_toward_centroid failed: {status}")
            return 2

        rclpy.spin_once(self, timeout_sec=0.5)
        try:
            tf_w = self._tf_buffer.lookup_transform(
                "base_link",
                WRIST_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            Logger.info(
                self,
                f"Final wrist (base_link): "
                f"({tf_w.transform.translation.x:.3f}, "
                f"{tf_w.transform.translation.y:.3f}, "
                f"{tf_w.transform.translation.z:.3f}); "
                f"cached centroid z = {point_in_base.point.z:.3f}",
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
