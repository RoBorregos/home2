#!/usr/bin/env python3

"""Manual test for picking clothes from INSIDE the laundry basket.

Drives the full pipeline end-to-end through the manipulation action server:
  look_back_stare -> enable grasp estimator -> detect the basket ("laundry_basket")
  -> estimator publishes the bbox-centroid clothes pose on
  /manipulation/clothes_grasp_pose -> PickManager builds a PickMotion goal ->
  pick_server does MoveIt pre-grasp + fixed-distance descent -> close gripper ->
  lift the clothes out -> PickManager returns to table_stare.

Requirements before running:
  - manipulation stack up (manipulation_core / pick_server) and the
    flat_grasp_estimator node (it also handles baskets/clothes).
  - vision detector publishing on /vision/detections with a "laundry_basket"
    class, and a basket with clothes on the floor within view of look_back_stare.

The clothes are located via the BASKET detection (the camera sees the basket,
not the clothes); the request label "laundry_clothes" selects the clothes mode.
PickManager enables/disables the estimator on its own, so this test only needs
to send the pick request — same as a normal pick_object call.
"""

import rclpy
from rclpy.node import Node
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks

# Must match an entry in CLOTHES_NAMES (frida_constants.manipulation_constants).
OBJECT_NAME = "laundry_clothes"


class TestClothesPick(Node):
    def __init__(self):
        super().__init__("test_clothes_pick")

        self.manipulation_manager = ManipulationTasks(self, task=Task.DEBUG, mock_data=False)

        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("Test Clothes Pick started.")
        self.run()

    def run(self):
        self.get_logger().info(f"--- STARTING CLOTHES PICK TEST ({OBJECT_NAME}) ---")

        result = self.manipulation_manager.pick_object(OBJECT_NAME)

        if result == Status.EXECUTION_SUCCESS:
            self.get_logger().info("SUCCESS: Clothes picked.")
        else:
            self.get_logger().error(f"FAILED: Result status {result}")


def main(args=None):
    rclpy.init(args=args)
    node = TestClothesPick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
