#!/usr/bin/env python3

"""Manual test for the rim pick feature.

Drives the full pipeline end-to-end through the manipulation action server:
  look_side_stare -> enable grasp estimator -> detect "laundry_basket" ->
  rim_grasp_estimator publishes the near-rim pose -> PickManager builds a
  PickMotion goal -> pick_server does MoveIt pre-grasp + fixed-distance descent
  -> close gripper -> lift.

Requirements before running:
  - manipulation stack up (manipulation_core / pick_server) and the
    flat_grasp_estimator node (it also handles rim objects).
  - vision detector publishing on /vision/detections with a "laundry_basket"
    class, and an object on the floor within view of look_side_stare.

PickManager enables/disables the estimator on its own, so this test only needs
to send the pick request — same as a normal pick_object call.
"""

import rclpy
from rclpy.node import Node
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks

# Must match an entry in RIM_NAMES (frida_constants.manipulation_constants)
# and the label the detector publishes for the object.
OBJECT_NAME = "aundry_basket"


class TestRimPick(Node):
    def __init__(self):
        super().__init__("test_rim_pick")

        self.manipulation_manager = ManipulationTasks(self, task=Task.DEBUG, mock_data=False)

        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("Test Rim Pick started.")
        self.run()

    def run(self):
        self.get_logger().info(f"--- STARTING RIM PICK TEST ({OBJECT_NAME}) ---")

        result = self.manipulation_manager.pick_object(OBJECT_NAME)

        if result == Status.EXECUTION_SUCCESS:
            self.get_logger().info("SUCCESS: Rim pick succeeded.")
        else:
            self.get_logger().error(f"FAILED: Result status {result}")


def main(args=None):
    rclpy.init(args=args)
    node = TestRimPick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
