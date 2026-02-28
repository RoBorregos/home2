#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from utils.status import Status
from utils.task import Task
from subtask_managers.manipulation_tasks import ManipulationTasks


class TestManipulationManager(Node):
    def __init__(self):
        super().__init__("test_manipulation_task_manager")

        self.manipulation_manager = ManipulationTasks(self, task=Task.DEBUG, mock_data=False)

        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("Test Manipulation Manager started.")
        self.run()

    def run(self):
        # self.get_logger().info("--- STARTING PLACE ON FLOOR TEST ---")

        result = self.manipulation_manager.place_on_floor()
        # result = self.manipulation_manager.move_to_position("front_stare",velocity=0.1)
        # result = self.manipulation_manager.move_to_position("place_floor_right", velocity=0.1)
        # result = self.manipulation_manager.move_to_position("place_floor_left", velocity=0.1)

        # result = self.manipulation_manager.move_joint_positions(named_position="place_floor_right", velocity=0.1, degrees=True)

        if result == Status.EXECUTION_SUCCESS:
            self.get_logger().info("SUCCESS: Robot moved to floor position.")
        else:
            self.get_logger().error(f"FAILED: Result status {result}")


def main(args=None):
    rclpy.init(args=args)
    node = TestManipulationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
