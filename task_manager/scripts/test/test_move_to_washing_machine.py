#!/usr/bin/env python3

"""
End-to-end test: get washing-machine drum centroid via moondream, then extend
the xArm forward to that height so the gripper points into the drum opening.

Single-shot. Tune `FORWARD_EXTENSION` between runs to adjust how far past the
rim the EE is pushed.
"""

import sys

import rclpy
from rclpy.node import Node

from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

FORWARD_EXTENSION = 0.20  # meters past the drum centroid along base_link +x
VELOCITY = 0.2
CAMERA_FLIP = False


class TestMoveToWashingMachine(Node):
    def __init__(self):
        super().__init__("test_move_to_washing_machine")
        self.vision = VisionTasks(self, task=Task.DEBUG)
        self.manipulation = ManipulationTasks(self, task=Task.DEBUG)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.vision.camera_upside_down(CAMERA_FLIP)

    def run(self) -> int:
        Logger.info(self, "Requesting washing machine hole centroid...")
        point = self.vision.get_washing_machine_hole_point()
        if point is None:
            Logger.error(self, "No centroid returned from moondream. Aborting.")
            return 1

        p = point.point
        Logger.success(
            self,
            f"Centroid @ {point.header.frame_id}: " f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})",
        )

        Logger.info(
            self,
            f"Commanding MoveToPose with forward_extension={FORWARD_EXTENSION:.2f} m",
        )
        status = self.manipulation.move_to_washing_machine_hole(
            hole_point=point,
            forward_extension=FORWARD_EXTENSION,
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
