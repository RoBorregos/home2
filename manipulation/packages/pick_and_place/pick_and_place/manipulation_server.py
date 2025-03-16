#!/usr/bin/env python3

# This node is the interface between manipulation tasks and the task managers
# This can receive tasks from ManipulationTask.action (pick, place, move, etc.) and call ManipulationCore accordingly
# It can also handle debugging tasks, such as using the rviz point publisher to try picking tasks

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_constants.manipulation_constants import (
    PICK_ACTION_SERVER,
)
from frida_interfaces.action import PickTask


class ManipulationServer(Node):
    def __init__(self):
        super().__init__("manipulation_server")
        self.callback_group = ReentrantCallbackGroup()

        # This is on manipulation core node
        self._pick_action_client = ActionClient(
            self,
            PickTask,
            PICK_ACTION_SERVER,
        )

        # Server here, which is the interface between manipulation tasks and the task managers

        # Point Subscriber for debug here


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = ManipulationServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
