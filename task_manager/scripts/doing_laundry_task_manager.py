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
        self.subtask_manager = SubtaskManager(self, task=Task.DOING_LAUNDRY, mock_areas=[])
        self.current_state = DoingLaundryTM.TaskStates.WAIT_FOR_BUTTON
        self.running_task = True
        self.state_start_time = None
        self.state_times = {}
        self.previous_state = None
        Logger.info(self, "DoingLaundryTM has started.")

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
            self.subtask_manager.hri.say("Navigating to the basket area.")
            self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET:
            Logger.info(self, "Navigating to basket area.")
            self.subtask_manager.nav.resume_nav()
            result = Status.EXECUTION_ERROR
            retry = 0
            while result == Status.EXECUTION_ERROR and retry < ATTEMPT_LIMIT:
                future = self.subtask_manager.nav.move_to_location("basket_area")
                if "navigation" not in self.subtask_manager.get_mocked_areas():
                    rclpy.spin_until_future_complete(self, future)
                    result = future.result()
                retry += 1
            self.subtask_manager.nav.pause_nav()
            self.current_state = DoingLaundryTM.TaskStates.DETECT_BASKET

        elif self.current_state == DoingLaundryTM.TaskStates.DETECT_BASKET:
            Logger.info(self, "Detecting laundry basket.")
            status, basket = self.subtask_manager.vision.get_laundry_basket()
            if status == Status.EXECUTION_SUCCESS:
                Logger.info(self, f"Basket found at {basket}")
                self.basket_centroid = basket
                self.current_state = DoingLaundryTM.TaskStates.PICK_LAUNDRY
            else:
                Logger.warn(self, "Could not detect basket. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.PICK_LAUNDRY:
            Logger.info(self, "Attempting to pick laundry from basket.")
            result = self.subtask_manager.manipulation.pick_from_basket(self.basket_centroid)
            if result == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Successfully picked laundry item.")
            else:
                Logger.error(self, "Failed to pick laundry item.")
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
