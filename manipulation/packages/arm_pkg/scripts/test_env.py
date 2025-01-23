#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import os


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.get_logger().info("*Starting Hear Node*")
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        route = os.environ["SRC_HOME_PATH"] + "/hri/packages/nlp"
        print(os.path.dirname(route))
        self.get_logger().info(route)


def main(args=None):
    rclpy.init(args=args)
    if os.environ["SRC_HOME_PATH"]:
        try:
            rclpy.spin(TestNode())
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        finally:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
