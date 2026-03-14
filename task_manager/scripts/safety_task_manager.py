#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task


class SafetyTaskManager(Node):
    class TaskStates:
        WAIT_DOOR_OPEN = "WAIT_DOOR_OPEN"
        GO_TO_SAFE_PLACE = "GO_TO_SAFE_PLACE"
        PRESENTATION = "PRESENTATION"
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        GO_TO_EXIT = "GO_TO_EXIT"
        END = "END"
        DEBUG = "DEBUG"

    def __init__(self):
        super().__init__("safety_task_node")
        Logger.info(self, "Initiating safety task manager")
        self.subtask_manager = SubtaskManager(
            self, task=Task.SAFETY_CHECK,
        )
        self.current_state = SafetyTaskManager.TaskStates.WAIT_DOOR_OPEN
        self.running_task = True
        Logger.info(self, "Safety Task Started")

    def navigate_to(self, location: str, sub_location: str = "", say: bool = True) -> Status:
        Logger.info(self, f"Navigating to {location} {sub_location} ")
        try:
            if say:
                self.subtask_manager.hri.say(text=f"Going to {location} {sub_location}", wait=False)
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            result = Status.EXECUTION_ERROR
            retry = 0
            while result == Status.EXECUTION_ERROR and retry < 5:
                future = self.subtask_manager.nav.move_to_location(location, sub_location)
                if "navigation" not in self.subtask_manager.get_mocked_areas():
                    rclpy.spin_until_future_complete(self, future)
                    result = future.result()
                else:
                    rclpy.spin_until_future_complete(self, future)
                    result = future.result()
                    if result.success:
                        result = Status.EXECUTION_SUCCESS
                retry += 1
            return result
        except Exception as e:
            Logger.error(self, f"Error navigating to {location}: {e}")
            return Status.EXECUTION_ERROR

    def run(self):
        if self.current_state == SafetyTaskManager.TaskStates.WAIT_DOOR_OPEN:
            Logger.state(self, "Waiting for open_door")
            self.subtask_manager.hri.say("Task started, Open the door to start")
            res = "closed"
            while res == "closed":
                time.sleep(1)
                status, res = self.subtask_manager.nav.check_door()
            self.current_state = SafetyTaskManager.TaskStates.GO_TO_SAFE_PLACE

        if self.current_state == SafetyTaskManager.TaskStates.GO_TO_SAFE_PLACE:
            Logger.state(self, "Going to safe place")
            self.subtask_manager.hri.say("Navigating to safe place")
            self.navigate_to("inspection_area")
            self.current_state = SafetyTaskManager.TaskStates.PRESENTATION

        if self.current_state == SafetyTaskManager.TaskStates.PRESENTATION:
            Logger.state(self, "Presenting the robot")
            self.subtask_manager.hri.say(
                "Hello, My name is Frida, I'm a Friendly Robotic Interactive Domestic Asisstant. I'm here to help you!"
            )
            self.current_state = SafetyTaskManager.TaskStates.WAIT_FOR_BUTTON

        if self.current_state == SafetyTaskManager.TaskStates.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for button to continue")
            self.subtask_manager.hri.say("Waiting for button to exit arena")
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Button Clicked")
            self.current_state = SafetyTaskManager.TaskStates.GO_TO_EXIT

        if self.current_state == SafetyTaskManager.TaskStates.GO_TO_EXIT:
            self.subtask_manager.hri.say("I will exit now")
            self.navigate_to("end_area")
            self.current_state = SafetyTaskManager.TaskStates.END

        if self.current_state == SafetyTaskManager.TaskStates.END:
            Logger.state(self, "Task Completed")
            self.subtask_manager.hri.say("I Finished my task")
            self.running_task = False


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = SafetyTaskManager()
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
