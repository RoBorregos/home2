#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.action import PickMotion
from frida_constants.manipulation_constants import PICK_MOTION_ACTION_SERVER


class ManipulationCore(Node):
    def __init__(self):
        super().__init__("manipulation_server")
        self.callback_group = ReentrantCallbackGroup()

        self._pick_action_client = ActionClient(
            self,
            PickMotion,
            PICK_MOTION_ACTION_SERVER,
        )


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = ManipulationCore()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
