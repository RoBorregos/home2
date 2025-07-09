#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory
from frida_interfaces.action import Move
import frida_constants
import importlib.resources as pkg_resources

MOVE_TOPIC = "/navigation/move"

class MoveActionServer(Node):  
    def __init__(self):
        super().__init__('move_action_server')

        self.person_detection_action_server = ActionServer(
        self,Move, MOVE_TOPIC, self.move_callback
    )
    
        try:
            with pkg_resources.files(frida_constants).joinpath("areas.json").open("r") as f:
                self.locations = json.load(f)
        except Exception as e:
            print(f"[ERROR] Failed to load JSON file: {e}")
            self.locations = {}

    def move_callback(self, goal_handle):
        """Callback to return a response until the robot move
        to a place."""

        target_location = goal_handle.request.location

        if not self.is_valid_location(target_location):
            self.get_logger().info(f"Invalid location: {target_location}")
            goal_handle.succeed()
            result = Move.Result()
            result.success = False
            return result
        
        self.get_logger().info(f'Moving to {target_location}...')
        goal_handle.succeed()

        result = Move.Result()
        result.success = True
        self.success(f"Successfully moved to {target_location}")
        return result
    
    def success(self, message):
        """Log a success message."""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def is_valid_location(self, location):
        for area, objects in self.locations.items():
            if location == area:
                return True
            if location in objects:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)

    move_action_server = MoveActionServer()

    rclpy.spin(move_action_server)


if __name__ == '__main__':
    main()