#!/usr/bin/env python3

"""
Task Manager for Carry my luggage task of Robocup @Home 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from subtask_managers.generic_tasks import GenericTask
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task
import time as t
from std_srvs.srv import Trigger

ATTEMPT_LIMIT = 3

FOLLOWING_TIMEOUT = 10

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
            self,
            task=Task.HELP_ME_CARRY,
            mock_areas=[],
        )
        
        self.is_tracking = False
        
        self.generic = GenericTask(self.subtask_manager)
        self.current_state = HelpMeCarryTM.TASK_STATES["START"]
        self.current_attempts = 0
        self.running_task = True
        
        Logger.info(self, "HelpMeCarryTaskManager has started.")
        self.subtask_manager.vision.track_person(False)
        self.subtask_manager.manipulation.follow_person(False)
        self.subtask_manager.hri.hear_multi(0)
        self.subtask_manager.nav.follow_person(False)

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
            # self.current_state = HelpMeCarryTM.TASK_STATES["FIND_GUEST"]
            self.current_state = HelpMeCarryTM.TASK_STATES["FOLLOWING_TO_DESTINATION"]

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
            Logger.state(self, "Following to guest")
            self.subtask_manager.nav.follow_person(True)
            Logger.state(self, "Executing following to guest")
            # self.subtask_manager.vision.
            # self.subtask_manager.nav.follow_person(self.is_following,)
            self.current_state = HelpMeCarryTM.TASK_STATES["FOLLOWING_TO_DESTINATION"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["FOLLOWING_TO_DESTINATION"]:
            Logger.state(self, "Following to destination")
            self.subtask_manager.hri.say("Please say stop when you want me to stop")
            self.subtask_manager.vision.track_person(True)
            self.subtask_manager.manipulation.follow_person(True)
            self.subtask_manager.nav.follow_person(True)
            self.subtask_manager.hri.hear_multi(1)
            #Encender Tracker
            #Encender brazo
            # self.subtask_manager.nav.follow_person(True)
            tracking_time = self.get_clock().now()
            prev_status = True
            while True:
                result = self.subtask_manager.hri.hear_multi(3)
                if result:
                    self.subtask_manager.hri.hear_multi(0)
                    self.subtask_manager.nav.follow_person(False)
                    self.subtask_manager.manipulation.follow_person(False)
                    self.subtask_manager.vision.track_person(False)
                    Logger.info(self, "Simulating follow disable")
                    self.subtask_manager.hri.say("I heard STOP, stopping now")

                    # self.current_state = HelpMeCarryTM.TASK_STATES["ASK_TO_POINT_THE_BAG"]
                    self.current_state = HelpMeCarryTM.TASK_STATES["END"]
                    break

                is_tracking = self.subtask_manager.vision.get_track_person()

                if not is_tracking == Status.EXECUTION_SUCCESS:
                    Logger.info(self,"Person not found stoping arm")
                    self.subtask_manager.manipulation.follow_person(False)
                    prev_status = False
                else:
                    if not prev_status:
                        Logger.info(self,"Starting arm")
                        self.subtask_manager.manipulation.follow_person(True)
                    prev_status = True
                    tracking_time = self.get_clock().now()

                if self.get_clock().now() - tracking_time > Duration(seconds=FOLLOWING_TIMEOUT):
                    Logger.info(self,"Activating deux ex machina")
                    Logger.info(self, "Simulating deux")
                    #Make deux ex machina
                    self.subtask_manager.hri.say("I lost you, please come back to the front of the camera")
                    self.current_state = HelpMeCarryTM.TASK_STATES["END"]
                    break

        
        if self.current_state == HelpMeCarryTM.TASK_STATES["ASK_TO_POINT_THE_BAG"]:
            Logger.state(self, "Asking to point the bag")
            self.subtask_manager.hri.say("Please point to the bag you want me to carry")
            self.current_state = HelpMeCarryTM.TASK_STATES["DETECT_THE_BAG"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["DETECT_THE_BAG"]:
            Logger.state(self, "Detecting the bag")
            self.subtask_manager.hri.say("I will now detect the bag")
            # TODO: Vision detects bag, returns bounding box for moondream, and returns the estimated bag pose in 2D plane
            # bounding_box, bag_pose = self.subtask_manager.vision.detect("bag")
            status, bbox, bag_point = self.subtask_manager.vision.get_pointing_bag(5)
            description = ""
            if status == Status.EXECUTION_SUCCESS:
                self.get_logger().info(f"Vision task result bbox: {bbox}")
                self.get_logger().info(f"Vision task result point: {bag_point}")
                self.running_task = False
                status, description = self.subtask_manager.vision.describe_bag(bbox, 60.0)
                # self.subtask_manager.manipulation.pan_to(bag_point)
            else:
                self.get_logger().info("Vision task failed")
                status, description = self.subtask_manager.vision.describe_bag_moondream()
            # desription = self.subtask_manager.vision.describe_bag_moondream()
            self.subtask_manager.hri.say("I have detected the bag, now I will describe it")
            self.subtask_manager.hri.say(description)

            # TODO: Manipulation receives the bag pose and aims the arm towards the pose
            #
            # TODO: HRI says the bag description
            # self.subtask_manager.hri.say(bag_description)
            self.current_state = HelpMeCarryTM.TASK_STATES["RECEIVE_THE_BAG"]
            # TODO: Manipulation grasp the bag (Not to be implemented yey)
            # self.current_state = HelpMeCarryTM.TASK_STATES["GRASP_THE_BAG"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["RECEIVE_THE_BAG"]:
            Logger.state(self, "Receiving the bag")
            self.subtask_manager.hri.say("I will now receive the bag")
            # TODO: self.subtask_manager.manipulation.move_joint_positions(dict,"bag_recieving_pose")
            # self.subtask_manager.manipulation.open_gripper()

            while True:
                self.subtask_manager.hri.say("I have opened my gripper, please put the bag in it.")
                s, confirmation = self.subtask_manager.hri.confirm(
                    "Have you put the bag in my gripper?", False, retries=8, wait_between_retries=4
                )

                if confirmation == "yes":
                    break

            # self.subtask_manager.manipulation.close_gripper()
            self.subtask_manager.hri.say(
                "I have received the bag, now I will return to the starting location"
            )
            self.current_state = HelpMeCarryTM.TASK_STATES["RETURN_TO_STARTING_LOCATION"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["GRASP_THE_BAG"]:
            Logger.state(self, "Grasping the bag")
            self.subtask_manager.manipulation.open_gripper()
            t.sleep(3)
            self.subtask_manager.manipulation.close_gripper()
            self.substask_manager.hri.say("I have grasped the bag")

            # MOCK: Self.subtask_manager.manipulation.pick(pose)

        if self.current_state == HelpMeCarryTM.TASK_STATES["RETURN_TO_STARTING_LOCATION"]:
            Logger.state(self, "Returning to starting location")
            self.subtask_manager.hri.say("I will now return to the starting location")
            # self.subtask_manager.vision.
            # TODO Define the starting location as the 0,0 coordinate for SLAM mode
            # self.subtask_manager.nav.move_to_location("starting_location,starting_location")
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
