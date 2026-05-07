#!/usr/bin/env python3

"""
Test: pick basket from floor behind the robot.

Sequence
--------
1. Move xArm to 'look_at_floor_back' — arm rotated to face the back of the
   robot and tilted down so the camera sees the floor.
2. Detect objects; find a laundry basket and select the side of its bounding
   box nearest to the robot (highest y2 = bottom edge in image).
3. Move the gripper to the basket's 3D centroid point via go_to_hand.
4. Close gripper.
5. Open gripper (test release).
6. Return arm to nav_pose.

Run
---
    ros2 run task_manager test_pick_basket
or
    python3 scripts/test/test_pick_basket.py
"""

import rclpy
from rclpy.node import Node

from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

# ── tunables ──────────────────────────────────────────────────────────────────
LOOK_POSE = "look_at_floor_back"  # xArm config that faces back & looks down
RETURN_POSE = "nav_pose"  # safe travel pose after the test
DETECTION_TIMEOUT = 15.0  # seconds to wait for basket detection
PICK_VELOCITY = 0.3  # m/s for MoveToPose approach
MOVE_VELOCITY = 0.3
# ──────────────────────────────────────────────────────────────────────────────


class TestPickBasket(Node):
    def __init__(self):
        super().__init__("test_pick_basket")

        self.manipulation = ManipulationTasks(self, task=Task.DEBUG, mock_data=False)
        self.vision = VisionTasks(self, task=Task.DEBUG, mock_data=False)

        rclpy.spin_once(self, timeout_sec=1.0)
        Logger.info(self, "TestPickBasket ready — starting sequence")
        self.run()

    # ------------------------------------------------------------------
    def run(self):
        # 1. Look at floor behind the robot
        Logger.info(self, f"[1/5] Moving to '{LOOK_POSE}'")
        result = self.manipulation.move_to_position(LOOK_POSE, velocity=MOVE_VELOCITY)
        if result != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"Failed to move to '{LOOK_POSE}'. Aborting.")
            return

        # 2. Detect basket
        Logger.info(self, "[2/5] Detecting basket on the floor")
        status, basket_bbox = self.vision.detect_basket(timeout=DETECTION_TIMEOUT)

        if status != Status.EXECUTION_SUCCESS or basket_bbox is None:
            Logger.error(self, "No basket detected. Returning to nav pose.")
            self.manipulation.move_to_position(RETURN_POSE, velocity=MOVE_VELOCITY)
            return

        Logger.success(
            self,
            f"Basket '{basket_bbox.classname}' found — "
            f"bottom_y={basket_bbox.y2:.3f}, "
            f"3d=({basket_bbox.px:.3f}, {basket_bbox.py:.3f}, {basket_bbox.pz:.3f}), "
            f"frame={basket_bbox.point3d.header.frame_id}",
        )

        # 3-5. Approach, grasp, release
        Logger.info(self, "[3/5] Picking basket (approach + grasp + release)")
        pick_result = self.manipulation.pick_basket(basket_bbox.point3d, velocity=PICK_VELOCITY)

        if pick_result == Status.EXECUTION_SUCCESS:
            Logger.success(self, "[3/5] pick_basket completed successfully")
        else:
            Logger.error(self, f"[3/5] pick_basket failed with status {pick_result}")

        # 6. Return arm to a safe pose regardless of pick result
        Logger.info(self, f"[4/5] Returning to '{RETURN_POSE}'")
        self.manipulation.move_to_position(RETURN_POSE, velocity=MOVE_VELOCITY)

        Logger.info(self, "[5/5] Test finished")


# ──────────────────────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = TestPickBasket()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
