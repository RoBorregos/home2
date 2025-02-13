#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for receptionist
commands.
"""

import rclpy
from rclpy.node import Node
from utils.logger import Logger
from xarm_msgs.srv import MoveVelocity
from rclpy.action import ActionServer
from frida_interfaces.action import Xarm_move


XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"

TIMEOUT = 5.0


class MiddleXarm:
    def __init__(self) -> None:
        self.action_server = ActionServer(
            self, Xarm_move, "xarm_move_action", self.execute_callback
        )
        self.move_client = self.node.create_client(MoveVelocity, XARM_MOVEVELOCITY_SERVICE)

    def execute_callback(self, goal_handle):
        motion_msg = MoveVelocity.Request()
        motion_msg.is_sync = True
        motion_msg.speeds = goal_handle.request.speeds

        future_move = self.move_client.call_async(motion_msg)
        rclpy.spin_until_future_complete(self.node, future_move, timeout_sec=TIMEOUT)
        result = Xarm_move.Result()
        result.success = True
        return result


if __name__ == "__main__":
    rclpy.init()
    node = Node("middleware_xarm")
    Manipulation_tasks = MiddleXarm(node)

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
