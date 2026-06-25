#!/usr/bin/env python3

"""End-to-end test for the washing-machine insert-and-pick.

Look pose -> moondream centroid -> snapshot in base_link -> arrow align ->
drive base in (odom RelativeMove) -> wrist down -> close -> wrist up -> base out.
The ZED rides the arm, so the centroid is snapshotted in `base_link` BEFORE
moving — that frame is the only one still valid after the arm moves.
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
GRIPPER_FRAME = "gripper_grasp_frame"
SUBJECT = "round container entrance of color orange"
INSERT_DEPTH = 0.10
LOOKDOWN_DEG = 90.0


class TestMoveToWashingMachine(Node):
    def __init__(self):
        super().__init__("test_move_to_washing_machine")
        self.vision = VisionTasks(self, task=Task.DEBUG)
        self.manipulation = ManipulationTasks(self, task=Task.DEBUG)
        self.nav = NavigationTasks(self, task=Task.DEBUG)
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
            Logger.error(self, "Failed to reach LOOK pose")
            return 3

        Logger.info(self, "Requesting centroid from moondream...")
        point = self.vision.get_moondream_point_3d(SUBJECT)
        if point is None or point.header.frame_id == "":
            Logger.error(self, "No centroid returned")
            return 1

        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link",
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            opening = do_transform_point(point, tf)
        except Exception as e:
            Logger.error(self, f"TF {point.header.frame_id} -> base_link failed: {e}")
            return 4
        pb = opening.point
        Logger.info(self, f"Opening in base_link: ({pb.x:.3f}, {pb.y:.3f}, {pb.z:.3f})")

        Logger.info(self, "Aligning arm toward opening...")
        if self.manipulation.align_arm_toward_centroid(opening) != Status.EXECUTION_SUCCESS:
            Logger.error(self, "align_arm_toward_centroid failed")
            return 2

        # Forward push = remaining gap to the opening + margin, from the aligned pose.
        rclpy.spin_once(self, timeout_sec=0.5)
        try:
            g = self._tf_buffer.lookup_transform(
                "base_link",
                GRIPPER_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            ).transform.translation
        except Exception as e:
            Logger.error(self, f"TF base_link <- {GRIPPER_FRAME} failed: {e}")
            return 5
        forward = max(0.0, (pb.x - g.x) + INSERT_DEPTH)
        Logger.info(self, f"Pushing base forward {forward:.3f} m")

        self.manipulation.open_gripper()
        if self.nav.move_relative(forward, backward=False)[0] != Status.EXECUTION_SUCCESS:
            Logger.error(self, "Forward insert failed")
            return 6

        self.manipulation.rotate_wrist_pitch(LOOKDOWN_DEG)
        self.manipulation.close_gripper()
        self.manipulation.rotate_wrist_pitch(-LOOKDOWN_DEG)

        if self.nav.move_relative(forward, backward=True)[0] != Status.EXECUTION_SUCCESS:
            Logger.error(self, "Retreat failed")
            return 7

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
