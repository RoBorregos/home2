#!/usr/bin/env python3

"""
Task Manager for Restaurant task of Robocup @Home 2025
"""

import rclpy
from rclpy.node import Node

from subtask_managers.vision_tasks import VisionTasks

from utils.logger import Logger

MOCK_NAV = False
MOCK_MANIPULATION = False
MOCK_HRI = False
MOCK_VISION = False

ATTEMPT_LIMIT = 3
CUSTOMERS_LIMIT = 2


class RestaurantTaskManager(Node):
    """Class to manage the restaurant task"""

    TASK_STATES = {
        "START": 0,
        "DETECT_CUSTOMERS": 1,
        "NAVIGATE_TO_CUSTOMER": 2,
        "TAKE_ORDER": 3,
        "NAVIGATE_TO_KITCHENBAR": 4,
        "PICK_ITEMS": 5,
        "DELIVER_ORDER": 6,
        "DEUX_GUIDE_TO_TABLE": 7,
        "FINISH_TASK": 8,
    }

    def __init__(self):
        """Initialize the node"""
        super().__init__("restaurant_task_manager")
        self.subtask_manager = {}

        self.subtask_manager["vision"] = VisionTasks(self, task="RESTAURANT", mock_data=MOCK_VISION)
        self.subtask_manager["nav"] = None
        self.subtask_manager["manipulation"] = None
        self.subtask_manager["hri"] = None

        self.detection_attempts = 0
        self.customers_served = 0
        self.pan_direction = 1
        self.coordinates = None
        self.order = None

        self.current_state = RestaurantTaskManager.TASK_STATES["START"]

        self.get_logger().info("RestaurantTaskManager has started.")
        self.run()

    def run(self):
        """Running main loop"""

        while rclpy.ok():
            if self.current_state == RestaurantTaskManager.TASK_STATES["START"]:
                Logger.state(self, "Starting task...")
                self.subtask_manager["hri"].say("I am ready to start the task.")
                self.current_state += 1

            if self.current_state == RestaurantTaskManager.TASK_STATES["DETECT_CUSTOMERS"]:
                Logger.state(self, "Detecting customers...")
                status, self.coordinates = self.subtask_manager["vision"].detect_waving_customer()
                if status == VisionTasks.STATE["EXECUTION_SUCCESS"]:
                    self.subtask_manager["hri"].say(
                        "I have detected a customer. I will navigate to them."
                    )
                    self.current_state = RestaurantTaskManager.TASK_STATES["NAVIGATE_TO_CUSTOMER"]
                elif (
                    status == VisionTasks.STATE["TARGET_NOT_FOUND"]
                    and self.detection_attempts < ATTEMPT_LIMIT
                ):
                    self.subtask_manager["manipulation"].pan_camera(self.pan_direction)
                    self.pan_direction *= -1
                else:
                    self.current_state = RestaurantTaskManager.TASK_STATES["DEUX_GUIDE_TO_TABLE"]
                self.detection_attempts += 1

            if self.current_state == RestaurantTaskManager.TASK_STATES["NAVIGATE_TO_CUSTOMER"]:
                Logger.state(self, "Navigating to customer...")
                self.detection_attempts = 0
                self.subtask_manager["nav"].navigate_to_target(self.coordinates)
                if self.order is None:
                    self.current_state = RestaurantTaskManager.TASK_STATES["TAKE_ORDER"]
                else:
                    self.current_state = RestaurantTaskManager.TASK_STATES["DELIVER_ORDER"]

            if self.current_state == RestaurantTaskManager.TASK_STATES["TAKE_ORDER"]:
                Logger.state(self, "Taking order...")
                self.subtask_manager["manipulation"].follow_face(True)
                self.subtask_manager["hri"].say("Hello! What would you like to order?")
                self.order = self.subtask_manager["hri"].get_order()
                if self.order != "error":
                    self.subtask_manager["hri"].say(f"I'll bring your order: {self.order}.")
                    self.subtask_manager["manipulation"].follow_face(False)
                    self.current_state = RestaurantTaskManager.TASK_STATES["NAVIGATE_TO_KITCHENBAR"]
                else:
                    self.subtask_manager["hri"].say(
                        "I'm sorry, I didn't understand. Can you repeat that?"
                    )

            if self.current_state == RestaurantTaskManager.TASK_STATES["NAVIGATE_TO_KITCHENBAR"]:
                Logger.state(self, "Navigating to kitchen/bar...")
                self.subtask_manager["nav"].navigate_to_origin()
                if self.order is None:
                    self.current_state = RestaurantTaskManager.TASK_STATES["DETECT_CUSTOMERS"]
                else:
                    self.current_state = RestaurantTaskManager.TASK_STATES["PICK_ITEMS"]

            if self.current_state == RestaurantTaskManager.TASK_STATES["PICK_ITEMS"]:
                Logger.state(self, "Picking items...")
                # TODO: Choose to pick items one by one or attempt using tray
                self.subtask_manager["manipulation"].pick_items()
                self.current_state = RestaurantTaskManager.TASK_STATES["NAVIGATE_TO_CUSTOMER"]

            if self.current_state == RestaurantTaskManager.TASK_STATES["DELIVER_ORDER"]:
                Logger.state(self, "Delivering order...")
                self.subtask_manager["hri"].say(
                    f"Here is your order: {self.order}. Please pick the items"
                )
                # TODO: If items are picked one at the time, then perform place
                self.customers_served += 1
                self.order = None
                if self.customers_served < CUSTOMERS_LIMIT:
                    self.current_state = RestaurantTaskManager.TASK_STATES["NAVIGATE_TO_KITCHENBAR"]
                else:
                    self.current_state = RestaurantTaskManager.TASK_STATES["FINISH_TASK"]

            if self.current_state == RestaurantTaskManager.TASK_STATES["DEUX_GUIDE_TO_TABLE"]:
                Logger.state(self, "Following to table...")
                self.subtask_manager["hri"].say(
                    "I couldn't find any customers. Please stand in front of me and guide me to the customer's table."
                )
                self.subtask_manager["vision"].follow_person()
                self.subtask_manager["nav"].follow_person()
                # TODO: Stop when reaching customer
                self.current_state = RestaurantTaskManager.TASK_STATES["TAKE_ORDER"]

            if self.current_state == RestaurantTaskManager.TASK_STATES["FINISH_TASK"]:
                Logger.state(self, "Finishing task...")
                self.subtask_manager["hri"].say("I have served all customers. I will rest now.")
                self.get_logger().info("Task completed.")
                self.timer.cancel()


def main(args=None):  #
    rclpy.init(args=args)
    node = RestaurantTaskManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
