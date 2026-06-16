#!/usr/bin/env python3

"""
End-to-end test: get washing-machine drum centroid via moondream, then move
the xArm to a pose at a fixed distance in front of the robot base, at the
height of the detected centroid. The robot does NOT try to reach the centroid
in 3D — it only matches its height.
"""

import sys

import rclpy
from rclpy.node import Node

from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

FORWARD_DISTANCE = 0.50  # meters in front of base_link the EE is sent to
LATERAL = 0.0  # meters left/right of base_link
VELOCITY = 0.2
CAMERA_FLIP = False
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
        rclpy.spin_once(self, timeout_sec=1.0)
        self.vision.camera_upside_down(CAMERA_FLIP)

    def run(self) -> int:
        Logger.info(self, "Requesting washing machine hole centroid...")
        point = self.vision.get_moondream_point_3d(SUBJECT)
        if point is None:
            Logger.error(self, "No centroid returned from moondream. Aborting.")
            return 1

        p = point.point
        Logger.success(
            self,
            f"Centroid @ {point.header.frame_id}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})",
        )

        Logger.info(
            self,
            f"Commanding MoveToPose at (x={FORWARD_DISTANCE:.2f}, "
            f"y={LATERAL:.2f}, z=<centroid height>) in base_link",
        )
        status = self.manipulation.move_to_height_in_front(
            reference_point=point,
            forward_distance=FORWARD_DISTANCE,
            lateral=LATERAL,
            velocity=VELOCITY,
        )

        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, "Move to washing machine approach pose: SUCCESS")
            return 0
        Logger.error(self, f"Move failed with status={status}")
        return 2


def main(args=None):
    rclpy.init(args=args)
    node = TestMoveToWashingMachine()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        try:
            node.vision.camera_upside_down(False)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
