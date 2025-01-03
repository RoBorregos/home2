#!/usr/bin/env python3

"""
Task Manager for Restaurant task of Robocup @Home 2025
"""

import rclpy
from rclpy.node import Node

from subtask_managers.vision_tasks import VisionTasks

AREAS = ["nav", "manipulation", "hri", "vision"]

NAV_ENABLED = True
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = True
VISION_ENABLED = True

AREA_ENABLED = {
    "nav": NAV_ENABLED,
    "manipulation": MANIPULATION_ENABLED,
    "hri": CONVERSATION_ENABLED,
    "vision": VISION_ENABLED
}

class RestaurantTaskManager(Node):
    def __init__(self):
        super().__init__('restaurant_task_manager')
        self.subtask_manager = {}

        if VISION_ENABLED:
            self.subtask_manager["vision"] = VisionTasks(self)

        self.get_logger().info('RestaurantTaskManager has started.')
        self.run()
    
    def run(self):
        """testing vision tasks"""
        self.subtask_manager["vision"].save_face_name("John Doe")
        angle = self.subtask_manager["vision"].find_seat()
        self.subtask_manager["vision"].detect_person()

def main(args=None):
    rclpy.init(args=args)
    node = RestaurantTaskManager()

    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
