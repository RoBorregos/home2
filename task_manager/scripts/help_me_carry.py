#!/usr/bin/env python3

"""
Task Manager for Carry my luggage task of Robocup @Home 2025
"""

import rclpy
from rclpy.node import Node
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

# from subtask_managers.vision_tasks import VisionTasks

ATTEMPT_LIMIT = 3


class HelpMeCarryTM(Node):
    """Class to manage the Help Me Carry task"""

    TASK_STATES = {
        "START": 0,
        "FIND_GUEST": 1,
        "CONFIRM_FOLLOWING": 2,
        "FOLLOWING_TO_DESTINATION": 3,
        "ASK_TO_POINT_THE_BAG": 4,
        "DETECT_THE_BAG": 5,
        "RECEIVE_THE_BAG": 6,
        "GRASP_THE_BAG": 7,
        "RETURN_TO_STARTING_LOCATION": 8,
        "PLACE_THE_BAG": 9,
        "END": 10,
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
        self.subtask_manager.hri.say(f"I will follow you now to {location}")
        return
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.move_to_position("navigation")
        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        rclpy.spin_until_future_complete(self, future)

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
            if result == Status.EXECUTION_SUCCESS:
                # self.subtask_manager.manipulation.move_to_position("gaze")
                # self.subtask_manager.manipulation.follow_face(True)
                self.subtask_manager.hri.say(
                    "Thank you, I can see you now, wait for following confirmation"
                )
                self.subtask_manager.vision.track_person()
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
            while True:
                s, result = self.subtask_manager.hri.interpret_keyword(["stop"])
                if result == "stop":
                    self.subtask_manager.hri.say("I will stop following you now")
                    break
                else:
                    self.subtask_manager.hri.say(
                        "I will keep following you. Please say stop when you want me to stop"
                    )

            # while not self.subtask_manager.hri.interpret_keyword("STOP"):
            #     # TODO: person_pose = self.subtask_manager.vision.follow_person() /It must return the person pose and the person position towards the center of the camera
            #     # to enable the arm to rotate towards the person
            #     # TODO: self.subtask_manager.manipulation.pan_to(person_pose)
            #     # TODO: self.subtask_manager.navigation.follow_pose_slam(person_pose)
            #     pass

            self.subtask_manager.hri.say("I heard STOP, stopping now")
            #            self.current_state = HelpMeCarryTM.TASK_STATES["RETURN_TO_STARTING_LOCATION"]
            self.current_state = HelpMeCarryTM.TASK_STATES["ASK_TO_POINT_THE_BAG"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["ASK_TO_POINT_THE_BAG"]:
            Logger.state(self, "Asking to point the bag")
            self.subtask_manager.hri.say("Please point to the bag you want me to carry")
            self.current_state = HelpMeCarryTM.TASK_STATES["DETECT_THE_BAG"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["DETECT_THE_BAG"]:
            Logger.state(self, "Detecting the bag")
            self.subtask_manager.hri.say("I will now detect the bag")
            # TODO: Vision detects bag, returns bounding box for moondream, and returns the estimated bag pose in 2D plane
            # bounding_box, bag_pose = self.subtask_manager.vision.detect("bag")
            status, bbox, point3d = self.subtask_manager.vision.get_pointing_bag(5)
            if status:
                self.get_logger().info(f"Vision task result: {bbox}")
                self.get_logger().info(f"Vision task result: {point3d}")
                self.running_task = False
                description = self.subtask_manager.vision.moondream_crop_query(bbox)
            else:

                self.get_logger().info("Vision task failed")
            # desription = self.subtask_manager.vision.describe_bag_moondream()
            self.subtask_manager.hri.say("I have detected the bag, now I will describe it")
            self.subtask_manager.hri.say(desription)

            # TODO: Vision receives a bounding box and gives it to moondream to describe the bag
            # bag_description = self.subtask_manager.vision.describe(bounding_box)
            # TODO: Manipulation receives the bag pose and aims the arm towards the pose
            # self.subtask_manager.manipulation.pan_to(bag_pose)
            # TODO: HRI says the bag description
            # self.subtask_manager.hri.say(bag_description)
            self.current_state = HelpMeCarryTM.TASK_STATES["RECEIVE_THE_BAG"]
            # TODO: Manipulation grasp the bag (Not to be implemented yey)
            # self.current_state = HelpMeCarryTM.TASK_STATES["GRASP_THE_BAG"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["RECEIVE_THE_BAG"]:
            Logger.state(self, "Receiving the bag")
            self.subtask_manager.hri.say("I will now receive the bag")
            # TODO: self.subtask_manager.manipulation.move_joint_positions(dict,"bag_recieving_pose")
            self.subtask_manager.manipulation.open_gripper()
            self.subtask_manager.hri.say("I have opened my gripper, please put the bag in it.")
            while (
                self.subtask_manager.hri.confirm(
                    "Have you put the bag in my gripper?", True, retries=8, wait_between_retries=4
                )
                != "yes"
            ):
                self.subtask_manager.hri.say("I have opened my gripper, please put the bag in it.")

            self.subtask_manager.manipulation.close_gripper()
            self.subtask_manager.hri.say(
                "I have received the bag, now I will return to the starting location"
            )
            self.current_state = HelpMeCarryTM.TASK_STATES["RETURN_TO_STARTING_LOCATION"]

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
