#!/usr/bin/env python3

"""
Task Manager for Carry my luggage task of Robocup @Home 2025
"""

import rclpy

from rclpy.node import Node
from utils.logger import Logger
from utils.subtask_manager import SubtaskManager, Task
from subtask_managers.vision_tasks import VisionTasks

ATTEMPT_LIMIT = 3


class HelpMeCarryTM(Node):
    """Class to manage the Help Me Carry task"""

    TASK_STATES = {
        "START": 0,
        "FIND_GUEST": 1,
        "CONFIRM_FOLLOWING": 2,
        "FOLLOWING_TO_DESTINATION": 3,
        "ASK_TO_POINT_THE_BAG": 4,
        "GRASP_THE_BAG": 5,
        "RETURN_TO_STARTING_LOCATION": 6,
        "PLACE_THE_BAG": 7,
        "END": 8,
    }

    def __init__(self):
        """Initialize the node"""
        super().__init__("Help_me_carry_task_manager")
        self.subtask_manager = SubtaskManager(
            self, task=Task.HELP_ME_CARRY, mock_areas=["manipulation", "navigation"]
        )
        self.current_state = HelpMeCarryTM.TASK_STATES["START"]
        self.current_attempts = 0
        self.running_task = True

        Logger.info(self, "HelpMeCarryTaskManager has started.")

    def navigate_to(self, location: str, sublocation: str = ""):
        self.subtask_manager.hri.say(f"I will follow you now{location}")
        return
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.move_to_position("navigation")
        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        rclpy.spin_until_future_complete(self, future)

    def confirm(self, statement: str) -> bool:
        """Confirm the name is correct"""
        self.subtask_manager.hri.say(f"I heard {statement}, is that correct?")
        response = self.subtask_manager.hri.hear()
        return True
        return self.subtask_manager.hri.is_positive(response)

    def hear_word(self, word: str) -> bool:
        """Check if the word is heard"""
        statement = self.subtask_manager.hri.hear()
        return statement
        return self.subtask_manager.hri.extract_data(word, statement)

    def run(self):
        """State machine"""

        if self.current_state == HelpMeCarryTM.TASK_STATES["START"]:
            Logger.state(self, "Starting task")
            self.subtask_manager.hri.say("I am ready to start my task.")
            self.current_state = HelpMeCarryTM.TASK_STATES["FIND_GUEST"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["FIND_GUEST"]:
            Logger.state(self, "Finding guest")
            self.subtask_manager.hri.say(
                "Please stand in front of me and wait for following confirmation", wait=True
            )
            result = self.subtask_manager.vision.detect_person(timeout=10)
            if result == VisionTasks.STATE["EXECUTION_SUCCESS"]:
                # self.subtask_manager.manipulation.move_to_position("gaze")
                # self.subtask_manager.manipulation.follow_face(True)
                self.subtask_manager.hri.say(
                    "Thank you, I can see you now, wait for following confirmation"
                )
                # MOCK: self.subtask_manager.vision.save_person()
                # if slef.subtask_manager.vision.save_person() == VisionTasks.STATE["EXECUTION_SUCCESS"]:
                self.current_state = HelpMeCarryTM.TASK_STATES["CONFIRM_FOLLOWING"]
            else:
                self.subtask_manager.hri.say("Please stand in front of me")

        if self.current_state == HelpMeCarryTM.TASK_STATES["CONFIRM_FOLLOWING"]:
            Logger.state(self, "Confirming following to guest")
            self.subtask_manager.hri.say("I will follow you now")
            self.current_state = HelpMeCarryTM.TASK_STATES["FOLLOWING_TO_DESTINATION"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["FOLLOWING_TO_DESTINATION"]:
            Logger.state(self, "Following to destination")
            while not self.subtask_manager.hri.hear("STOP"):
                # MOCK: self.subtask_manager.vision.follow_person() /It must return the person pose and the person position towards the center of the camera
                # to enable the arm to rotate towards the person
                # MOCK: self.subtask_manager.manipulation.pan_to(person_pose)
                # MOCK: self.subtask_manager.navigation.follow_pose_slam(person_pose)
                pass

            self.subtask_manager.hri.say("I heard STOP, stopping now")
            self.subtask_manager.hri.say("Please hand me the bag and confirm saying BAG PLACED")
            while not self.subtask_manager.hri.hear("BAG PLACED"):
                pass

            self.subtask_manager.hri.say("Thank you for handing me the bag")

            # TODO: Only if the bag picking will be done: self.current_state = HelpMeCarryTM.TASK_STATES["ASK_TO_POINT_THE_BAG"]
            self.current_state = HelpMeCarryTM.TASK_STATES["RETURN_TO_STARTING_LOCATION"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["ASK_TO_POINT_THE_BAG"]:
            Logger.state(self, "Asking to point the bag")
            self.subtask_manager.hri.say("Please point to the bag you want me to carry")
            # MOCK: self.subtask_manager.vision.detect_pointed_bag() /Must return the approximated pose of the bag
            # MOCK: self.subtask_mananger.nav.approach_table(pose) /aPPPPROACHIN
            self.current_state = HelpMeCarryTM.TASK_STATES["GRASP_THE_BAG"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["GRASP_THE_BAG"]:
            Logger.state(self, "Grasping the bag")
            # MOCK: Self.subtask_manager.manipulation.pick(pose)

        if self.current_state == HelpMeCarryTM.TASK_STATES["RETURN_TO_STARTING_LOCATION"]:
            Logger.state(self, "Returning to starting location")

            # TODO Define the starting location as the 0,0 coordinate for SLAM mode
            self.subtask_manager.nav.move_to_location("starting_location,starting_location")
            # Wait until the nav module returns true
            self.current_state = HelpMeCarryTM.TASK_STATES["PLACE_THE_BAG"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["PLACE_THE_BAG"]:
            Logger.state(self, "Placing the bag")
            # MOCK: self.subtask_manager.manipulation.place(pose)
            self.current_state = HelpMeCarryTM.TASK_STATES["END"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["END"]:
            Logger.state(self, "Ending task")
            self.subtask_manager.hri.say("I have finished my task, I will rest now.")
            self.running_task = False


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = HelpMeCarryTM()

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
