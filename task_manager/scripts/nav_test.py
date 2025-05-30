#!/usr/bin/env python3
"""
Task Manager for testing the nav subtask manager
"""

import rclpy
from rclpy.node import Node
from utils.subtask_manager import SubtaskManager, Task

# from utils.task import Task
from utils.logger import Logger


class TestTaskManager(Node):
    def __init__(self):
        super().__init__("nav_test_task_manager")
        self.subtask_manager = SubtaskManager(
            self, task=Task.DEBUG, mock_areas=["manipulation", "vision", "hri"]
        )
        Logger.info(self, "TestNavTaskManager has started.")
        self.run()

    def run(self):
        Logger.info(self, "Running test task manager")
        # status, res = self.subtask_manager.nav.check_door()
        # if status == Status.EXECUTION_SUCCESS:
        #     Logger.info(self, f"Door status: {res}")
        # else:
        #     Logger.error(self, "Failed to check door status")

        data = self.subtask_manager.nav.ReturnLocation_callback()
        print(data[0])
        print(data[1].location)
        print(data[1].nearest_locations)

        Logger.info(self, f"data: {data}")


def main(args=None):
    print("Starting test task manager")
    rclpy.init(args=args)
    node = TestTaskManager()

    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
