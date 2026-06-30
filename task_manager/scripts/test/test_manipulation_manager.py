#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.utils.logger import Logger
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks


class TestManipulationManager(Node):
    def __init__(self):
        super().__init__("ManipulationTaskManager")
        self.logs = self.declare_parameter("clear_logs", True).value
        self.mocked = self.declare_parameter("mocked", False).value
        self.task_to_test = Task[self.declare_parameter("task", Task.DEBUG.name).value]

        print(f"\n{Logger.BOLD}Starting Manipulation Subtask \n")

        self.manipulation_manager = ManipulationTasks(
            self, task=self.task_to_test, mock_data=self.mocked
        )

        self.tests_funcs = {
            "Follow Person Start (arm)": {
                "func": self.manipulation_manager.follow_person,
                "follow": True,
            },
            "Follow Person Stop (arm)": {
                "func": self.manipulation_manager.follow_person,
                "follow": False,
            },
            "Open Gripper": {"func": self.manipulation_manager.open_gripper},
            "Close Gripper": {"func": self.manipulation_manager.close_gripper},
        }

        print(f"\n{Logger.BOLD}Testing {len(self.tests_funcs)} available subtasks..... \n")
        self.run()

    def check_manip_task(self, func, *args, **kwargs):
        result = func(**kwargs)
        assert result == Status.EXECUTION_SUCCESS, f"Task returned {result}"

    def run(self):
        passed = 0
        failed = 0
        for command in self.tests_funcs:
            if Logger.run_test(
                f"Test - {command}",
                self.check_manip_task,
                **self.tests_funcs[command],
                clear_logs=self.logs,
            ):
                passed += 1
            else:
                failed += 1
        print()
        if failed == 0:
            print(f"  {Logger.GREEN}{Logger.BOLD}All {passed} tests passed!{Logger.RESET}\n")
        else:
            print(
                f"  {Logger.GREEN}{passed} passed{Logger.RESET}, "
                f"{Logger.RED}{failed} failed{Logger.RESET}\n"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TestManipulationManager()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
