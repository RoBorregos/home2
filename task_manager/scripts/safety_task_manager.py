#!/usr/bin/env python3
from enum import Enum
from rclpy.node import Node

import rclpy
from utils.subtask_manager import SubtaskManager, Task
from utils.logger import Logger
from utils.status import Status
import time


class ExecutionStates(Enum):
    START = 0
    WAIT_DOOR_OPEN = 5
    GO_TO_SAFE_PLACE = 10
    SAY_ARRIVED = 20
    GO_BACK = 30


class Retries(Enum):
    NAVIGATION = 5


for i in ExecutionStates:
    for j in ExecutionStates:
        if i.value == j.value and i.name != j.name:
            raise ValueError(f"Duplicate value found: {i.value} for {i.name} and {j.name}")


class SafetyTaskManager(Node):
    def __init__(self):
        super().__init__("safety_task_node")
        Logger.info(self, "Initiating safety task manager")
        self.subtask_manager = SubtaskManager(self, task=Task.STORING_GROCERIES, mock_areas=[])
        self.state = ExecutionStates.START
        # return self

    def nav_to(self, location: str, sub_location: str = "", say: bool = True) -> Status:
        Logger.info(self, f"Navigating to {location} {sub_location} ")
        try:
            if say:
                self.subtask_manager.hri.say(text=f"Going to {location} {sub_location}", wait=False)
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            result = Status.EXECUTION_ERROR
            retry = 0
            while result == Status.EXECUTION_ERROR and retry < Retries.NAVIGATION.value:
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
        Logger.info(self, "starting safety")
        Logger.info(self, "Ready for open_door")
        self.state = ExecutionStates.WAIT_DOOR_OPEN
        res = "closed"
        while res == "closed":
            time.sleep(1)
            status, res = self.subtask_manager.nav.check_door()
            if status == Status.EXECUTION_SUCCESS:
                Logger.info(self, f"Door status: {res}")
            else:
                Logger.error(self, "Failed to check door status")

        Logger.info(self, "Door OPENED GOING TO NEXT STAT")
        hres: Status = self.nav_to("living_room")

        Logger.info(self, f"hres: {hres}")

        self.state = ExecutionStates.SAY_ARRIVED

        Logger.info(self, "Saying")
        self.subtask_manager.hri.say(
            "Hello, My name is Frida, I'm a Friendly Robotic Interactive Domestic Asisstant. I'm here yo help you!"
        )

        self.state = ExecutionStates.GO_TO_SAFE_PLACE

        Logger.info(self, "Going back")

        self.nav_to("kitchen")

        Logger.info(self, "Arrived")

        self.subtask_manager.hri.say("I've arrived.")


def main(args=None):
    """Main function"""
    # print("Starting Storing Groceries Manager...")
    rclpy.init(args=args)
    node = SafetyTaskManager()

    try:
        node.run()
        # while rclpy.ok():
        #     rclpy.spin_once(node, timeout_sec=0.1)
        #     if node.run() == ExecutionStates.END:
        #         break
        node.subtask_manager.hri.say(text="Ending Safety Task...", wait=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
