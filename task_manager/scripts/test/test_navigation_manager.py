#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.utils.logger import Logger
from task_manager.subtask_managers.nav_tasks import NavigationTasks


def _front_point(x: float = 1.2) -> PointStamped:
    """A point `x` meters straight ahead of the robot (base_link frame)."""
    p = PointStamped()
    p.header.frame_id = "base_link"
    p.point.x = x
    return p


# TUPLA DE REGRESO DE IDA ARGS
class TestNavigationManager(Node):
    def __init__(self):
        super().__init__("NavigationTaskManager")
        self.logs = self.declare_parameter("clear_logs", True).value
        self.mocked = self.declare_parameter("mocked", False).value
        self.task_to_test = Task[self.declare_parameter("task", Task.DEBUG.name).value]

        print(f"\n{Logger.BOLD}Starting Navigation Subtask \n")

        self.navigation_manager = NavigationTasks(
            self, task=self.task_to_test, mock_data=self.mocked
        )

        self.tests_funcs = {
            "Check Door": {"func": self.navigation_manager.check_door},
            "Retrieve Areas": {"func": self.navigation_manager.retrieve_areas},
            "Move to Location": {
                "func": self.navigation_manager.move_to_location,
                "location": "entrance",
                "sublocation": "",
            },
            "Follow Person Start": {
                "func": self.navigation_manager.follow_person,
                "follow": True,
            },
            "Follow Person Stop": {
                "func": self.navigation_manager.follow_person,
                "follow": False,
            },
            "Get Path Info": {
                "func": self.navigation_manager.get_path_info,
                "location_b": "entrance",
                "sublocation_b": "",
            },
            # Approaches a virtual target 1.2 m ahead: with standoff 0.5 the
            # robot should advance ~0.7 m and stop facing the point (safe to
            # run in an open area; nav_central snaps to a costmap-free pose).
            "Approach Point (1.2m ahead)": {
                "func": self.navigation_manager.approach_point,
                "point": _front_point(1.2),
                "standoff": 0.5,
            },
        }

        print(f"\n{Logger.BOLD}Testing {len(self.tests_funcs)} available subtaks..... \n")
        self.run()

    def check_nav_task(self, func, *args, **kwargs):
        result = func(**kwargs)
        # Check for map_service case
        if (
            result[0] == Status.EXECUTION_ERROR
            and result[1] == self.navigation_manager.areas_backup
        ):
            assert False, "Service not started or Service return empty"

        assert result[0] == Status.EXECUTION_SUCCESS, result[1]

    def run(self):
        passed = 0
        failed = 0
        for command in self.tests_funcs:
            if Logger.run_test(
                f"Test - {command}",
                self.check_nav_task,
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
                f"  {Logger.GREEN}{passed} passed{Logger.RESET}, {Logger.RED}{failed} failed{Logger.RESET}\n"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TestNavigationManager()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
