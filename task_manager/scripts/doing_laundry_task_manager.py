#!/usr/bin/env python3
"""
Task Manager for Doing Laundry Task
"""

import rclpy
from rclpy.node import Node
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3


class DoingLaundryTM(Node):
    """Task Manager for Doing Laundry"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        NAVIGATE_TO_BASKET = "NAVIGATE_TO_BASKET"
        DETECT_BASKET = "DETECT_BASKET"
        PICK_LAUNDRY = "PICK_LAUNDRY"
        END = "END"

    def __init__(self):
        super().__init__("doing_laundry_task_manager")
        self.subtask_manager = SubtaskManager(
            self, task=Task.DOING_LAUNDRY, mock_areas=["manipulation", "vision"]
        )
        self.current_state = DoingLaundryTM.TaskStates.WAIT_FOR_BUTTON
        self.running_task = True
        self.state_start_time = None
        self.state_times = {}
        self.previous_state = None
        self.basket_detection = None

        # Initial pose
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        Logger.info(self, "DoingLaundryTM has started.")

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location following the pattern from HRIC"""
        self.subtask_manager.vision.deactivate_face_recognition()
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        if say:
            Logger.info(self, f"Moving to {location}")
            self.subtask_manager.hri.say(f"Navigating to {location}.", wait=False)
        return self.subtask_manager.nav.move_to_location(location, sublocation)

    def run(self):
        if self.current_state == DoingLaundryTM.TaskStates.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.", wait=False)

            # Wait for the start button to be pressed
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, Doing Laundry task will begin now")
            self.current_state = DoingLaundryTM.TaskStates.START

        elif self.current_state == DoingLaundryTM.TaskStates.START:
            Logger.state(self, "Starting Doing Laundry Task")
            self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET:
            Logger.info(self, "Navigating to basket area")
            status, error = self.navigate_to("kitchen", "dishwasher")

            if status == Status.EXECUTION_SUCCESS:
                self.current_state = DoingLaundryTM.TaskStates.DETECT_BASKET
            else:
                Logger.error(self, f"Navigation failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.DETECT_BASKET:
            Logger.info(self, "Detecting laundry basket from vision detections.")
            # Use detect_objects from vision_tasks.py
            status, detections = self.subtask_manager.vision.detect_objects()
            basket_detection = None
            if status == Status.EXECUTION_SUCCESS and detections:
                for det in detections:
                    if det.classname.lower() == "basket":
                        basket_detection = det
                        break
            if basket_detection:
                Logger.info(self, f"Basket detected: {basket_detection.classname}")
                self.basket_detection = basket_detection
                self.current_state = DoingLaundryTM.TaskStates.PICK_LAUNDRY
            else:
                Logger.warn(self, "Could not detect basket in vision detections. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.PICK_LAUNDRY:
            Logger.info(self, "Attempting to pick the basket using pick_object.")
            # Use pick_object from manipulation
            result = self.subtask_manager.manipulation.pick_object("laundry_basket")
            if result == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Successfully picked the basket.")
            else:
                Logger.error(self, "Failed to pick the basket.")
            self.current_state = DoingLaundryTM.TaskStates.END

        elif self.current_state == DoingLaundryTM.TaskStates.END:
            Logger.state(self, "Ending task")
            self.subtask_manager.hri.say("Laundry task finished. I will rest now.")
            self.running_task = False


def main(args=None):
    rclpy.init(args=args)
    node = DoingLaundryTM()
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
