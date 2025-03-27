#!/usr/bin/env python3

"""
Node to hold generic routines
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from utils.decorators import mockable, service_check
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
import os
import json
from rclpy.task import Future

class Generic_Tasks:
    """Class to manage the generic tasks"""

    def __init__(self, task_manager, mock_data=False) -> None:
        if not self.mock_data:
            self.setup_services()
            
    def test_call(self):
        pass

if __name__ == "__main__":
    rclpy.init()
    node = Node("generic_tasks")
    generic_tasks = Generic_Tasks(node)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
