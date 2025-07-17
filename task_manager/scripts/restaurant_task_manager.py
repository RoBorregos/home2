#!/usr/bin/env python3

"""
Task Manager for Restaurant task of Robocup @Home 2025
"""

import rclpy
from rclpy.node import Node

from utils.subtask_manager import SubtaskManager, Task

from utils.logger import Logger
from utils.status import Status
from frida_constants.vision_enums import Gestures
from frida_constants.vision_constants import (
    TRACKER_IMAGE_TOPIC
)

MOCK_NAV = False
MOCK_MANIPULATION = False
MOCK_HRI = False
MOCK_VISION = False

ATTEMPT_LIMIT = 3
CUSTOMERS_LIMIT = 2


class RestaurantTaskManager(Node):
    """Class to manage the restaurant task"""

    class TaskStates:
        START = "START"
        DETECT_CUSTOMERS = "DETECT_CUSTOMERS"
        NAVIGATE_TO_CUSTOMER = "NAVIGATE_TO_CUSTOMER"
        TAKE_ORDER = "TAKE_ORDER"
        NAVIGATE_TO_KITCHENBAR = "NAVIGATE_TO_KITCHENBAR"
        PICK_ITEMS = "PICK_ITEMS"
        DELIVER_ORDER = "DELIVER_ORDER"
        DEUX_GUIDE_TO_TABLE = "DEUX_GUIDE_TO_TABLE"
        END = "END"

    def __init__(self):
        """Initialize the node"""
        super().__init__("restaurant_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.RESTAURANT, mock_areas=[])

        self.running_task = True
        self.detection_attempts = 0
        self.customers_served = 0
        self.pan_direction = 1
        self.coordinates = None
        self.order = None
        self.pan_angles = [0, -45, 90]
        self.pick_object = None
        self.gestures = [Gestures.WAVING.value, Gestures.RAISING_LEFT_ARM.value, Gestures.RAISING_RIGHT_ARM.value]
        self.subtask_manager.vision.track_person(False)
        self.subtask_manager.manipulation.follow_person(False)
        self.subtask_manager.nav.follow_person(False)

        self.current_state = RestaurantTaskManager.TaskStates.START

        self.get_logger().info("RestaurantTaskManager has started.")
        # self.run()

    def run(self):
        """Running main loop"""

        while rclpy.ok():
            if self.current_state == RestaurantTaskManager.TaskStates.START:
                Logger.state(self, "Starting task...")
                self.subtask_manager.hri.say("I am ready to start the task.")
                self.subtask_manager.hri.publish_display_topic(TRACKER_IMAGE_TOPIC)

            if self.current_state == RestaurantTaskManager.TaskStates.DETECT_CUSTOMERS:
                Logger.state(self, "Detecting customers...")

                person_found = False
                for pan_angle in self.pan_angles:
                    self.subtask_manager.manipulation.pan_to(pan_angle)
                    if person_found:
                        break

                    for gesture in self.gestures:
                        status = self.subtask_manager.vision.track_person_by("gestures", gesture)
                        if status == Status.EXECUTION_SUCCESS:
                            person_found = True

                if person_found:
                    self.subtask_manager.hri.say("I have detected a customer. You can check my display to see the person. I will go towards them now.")
                    self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_CUSTOMER            
                # elif (
                #     status == Status.TARGET_NOT_FOUND
                #     and self.detection_attempts < ATTEMPT_LIMIT
                # ):
                #     self.subtask_manager["manipulation"].pan_camera(self.pan_direction)
                #     self.pan_direction *= -1
                # else:
                    # self.current_state = RestaurantTaskManager.TASK_STATES["DEUX_GUIDE_TO_TABLE"]
                # self.detection_attempts += 1

            if self.current_state == RestaurantTaskManager.TaskStates.NAVIGATE_TO_CUSTOMER:
                Logger.state(self, "Navigating to customer...")
                self.subtask_manager.manipulation.follow_person(True)
                self.subtask_manager.nav.follow_person(True)
                if self.order is None:
                    self.current_state = RestaurantTaskManager.TaskStates.TAKE_ORDER
                else:
                    self.current_state = RestaurantTaskManager.TaskStates.DELIVER_ORDER

            if self.current_state == RestaurantTaskManager.TaskStates.TAKE_ORDER:
                Logger.state(self, "Taking order...")
                self.subtask_manager.manipulation.follow_person(False)
                self.subtask_manager.nav.follow_person(False)
                self.subtask_manager.manipulation.follow_face(True)
                self.subtask_manager.hri.say("Hello! What would you like to order?")
                status, self.order = self.subtask_manager.hri.ask_and_confirm(
                    question="What is your order?", query="object", use_hotwords=False
                )
                if status == Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say(f"I'll bring your order: {self.order}.")
                    self.subtask_manager.manipulation.follow_face(False)
                    self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_KITCHENBAR
                else:
                    self.subtask_manager.hri.say(
                        "I'm sorry, I didn't understand. Can you repeat that?"
                    )

            if self.current_state == RestaurantTaskManager.TaskStates.NAVIGATE_TO_KITCHENBAR:
                Logger.state(self, "Navigating to kitchen/bar...")
                self.subtask_manager.nav.navigate_to_origin()
                if self.order is None:
                    self.current_state = RestaurantTaskManager.TaskStates.DETECT_CUSTOMERS
                else:
                    self.current_state = RestaurantTaskManager.TaskStates.PICK_ITEMS

            if self.current_state == RestaurantTaskManager.TaskStates.PICK_ITEMS:
                Logger.state(self, "Picking items...")
                # TODO: Choose to pick items one by one or attempt using tray
                self.pick_object = self.order[]
                self.subtask_manager.manipulation.pick_object(self.order[0])
                
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_CUSTOMER

            if self.current_state == RestaurantTaskManager.TaskStates.DELIVER_ORDER:
                Logger.state(self, "Delivering order...")
                self.subtask_manager.hri.say(
                    f"Here is the firsrt item of your order: {self.pick_object}. Please grab the object and I will open my gripper"
                )
                self.subtask_manager.manipulation.open_gripper()
                self.customers_served += 1
                self.order = None
                if self.customers_served < CUSTOMERS_LIMIT:
                    self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_KITCHENBAR
                else:
                    self.current_state = RestaurantTaskManager.TaskStates.END

            if self.current_state == RestaurantTaskManager.TaskStates.END:
                Logger.state(self, "Finishing task...")
                self.subtask_manager.hri.say("I have served all customers. I will rest now.")
                self.get_logger().info("Task completed.")
                self.running_task = False


def main(args=None):  #
    rclpy.init(args=args)
    node = RestaurantTaskManager()

    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
