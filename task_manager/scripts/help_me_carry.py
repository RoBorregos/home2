#!/usr/bin/env python3

"""
Task Manager for Carry my luggage task of Robocup @Home 2025
"""

import time as t

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from subtask_managers.generic_tasks import GenericTask
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3

FOLLOWING_TIMEOUT = 7


class HelpMeCarryTM(Node):
    """Class to manage the Help Me Carry task"""

    TASK_STATES = {
        "WAIT_BUTTON": 13,
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
        "TEST_STAGE": 11,
        "TEST_END": 12,
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
        self.current_state = HelpMeCarryTM.TASK_STATES["WAIT_BUTTON"]
        self.current_attempts = 0
        self.running_task = True

        Logger.info(self, "HelpMeCarryTaskManager has started.")
        self.subtask_manager.vision.track_person(False)
        # self.subtask_manager.manipulation.follow_person(False)
        self.subtask_manager.nav.follow_person(False)

    # def navigate_to(self, location: str, sublocation: str = ""):
    #     self.subtask_manager.hri.say(f"I will follow you now to {location}")
    #     return
    #     self.subtask_manager.manipulation.follow_face(False)
    #     self.subtask_manager.manipulation.move_to_position("navigation")
    #     future = self.subtask_manager.nav.move_to_location(location, sublocation)
    #     rclpy.spin_until_future_complete(self, future)

    def run(self):
        """State machine"""
        if self.current_state == HelpMeCarryTM.TASK_STATES["WAIT_BUTTON"]:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.")
            # Wait for the start button to be pressed
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, receptionist task will begin now")
            self.current_state = HelpMeCarryTM.TASK_STATES["START"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["TEST_STAGE"]:
            Logger.state(self, "Starting TEST")
            # self.subtask_manager.nav.change_bt("standard")
            # t.sleep(5)
            future = self.subtask_manager.nav.move_to_zero()
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            # self.subtask_manager.manipulation.move_to_position("carry_pose")
            # self.subtask_manager.manipulation.follow_person(True)
            # t.sleep(4)
            # self.subtask_manager.manipulation.follow_person(False)
            # self.subtask_manager.manipulation.move_to_position("nav_pose")
            # self.subtask_manager.nav.change_bt("standard")
            # future = self.subtask_manager.nav.move_to_zero()
            # rclpy.spin_until_future_complete(self, future)

            self.current_state = HelpMeCarryTM.TASK_STATES["TEST_END"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["TEST_END"]:
            Logger.state(self, "END")

        if self.current_state == HelpMeCarryTM.TASK_STATES["START"]:
            Logger.state(self, "Starting task")
            Logger.state(self, "papucagadas")
            self.subtask_manager.manipulation.move_to_position("carry_pose")
            self.subtask_manager.hri.say("I am ready to start my task.")
            self.current_state = HelpMeCarryTM.TASK_STATES["FIND_GUEST"]
            # self.current_state = HelpMeCarryTM.TASK_STATES["FOLLOWING_TO_DESTINATION"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["FIND_GUEST"]:
            Logger.state(self, "Finding guest")
            self.subtask_manager.hri.say(
                "Please stand in front of me and wait for following confirmation", wait=True
            )
            result = self.subtask_manager.vision.detect_person(timeout=10)
            if result == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(
                    "Thank you, I can see you now, wait for my confirmation to start following"
                )
                self.current_state = HelpMeCarryTM.TASK_STATES["CONFIRM_FOLLOWING"]
            else:
                self.subtask_manager.hri.say("Please stand in front of me")

        if self.current_state == HelpMeCarryTM.TASK_STATES["CONFIRM_FOLLOWING"]:
            Logger.state(self, "Confirming following to guest")
            Logger.state(self, "Following to guest")
            Logger.state(self, "Executing following to guest")
            self.current_state = HelpMeCarryTM.TASK_STATES["FOLLOWING_TO_DESTINATION"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["FOLLOWING_TO_DESTINATION"]:
            t.sleep(3)
            self.subtask_manager.vision.track_person(True)
            # self.subtask_manager.manipulation.follow_person(True)
            # self.subtask_manager.nav.follow_person(True)
            # Logger.state(self, "Please stand 2 meters away from me ")
            self.subtask_manager.hri.say("I will follow you now")
            self.subtask_manager.hri.say("Please say stop when you want me to stop")
            self.subtask_manager.nav.follow_person(True)
            tracking_time = self.get_clock().now()
            print("Canceling hear action")
            self.subtask_manager.hri.cancel_hear_action()
            print("Hearing again")
            future = self.subtask_manager.hri.hear_streaming(timeout=120, silence_time=120)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3)
            prev_status = True
            while True:
                if "stop" in self.subtask_manager.hri.current_transcription.lower():
                    self.subtask_manager.hri.cancel_hear_action()
                    self.subtask_manager.nav.follow_person(False)
                    # self.subtask_manager.manipulation.follow_person(False)
                    self.subtask_manager.vision.track_person(False)
                    self.subtask_manager.hri.say("I heard STOP, stopping now")
                    self.current_state = HelpMeCarryTM.TASK_STATES["ASK_TO_POINT_THE_BAG"]
                    # self.current_state = HelpMeCarryTM.TASK_STATES["END"]
                    break

                is_tracking = self.subtask_manager.vision.get_track_person()

                if not is_tracking == Status.EXECUTION_SUCCESS:
                    Logger.info(self, "Person not found stoping arm")
                    # self.subtask_manager.manipulation.follow_person(False)
                    prev_status = False
                else:
                    if not prev_status:
                        Logger.info(self, "Starting arm")
                        # self.subtask_manager.manipulation.follow_person(True)
                    prev_status = True
                    tracking_time = self.get_clock().now()

                if self.get_clock().now() - tracking_time > Duration(seconds=FOLLOWING_TIMEOUT):
                    Logger.info(self, "Activating deux ex machina")
                    self.subtask_manager.hri.cancel_hear_action()

                    self.subtask_manager.nav.follow_person(False)
                    # self.subtask_manager.manipulation.follow_person(False)
                    # MAYBE ADD TO PUT IN FRONT OF THE CAMERA
                    self.subtask_manager.vision.track_person(False)
                    self.subtask_manager.manipulation.move_to_position("carry_pose")
                    self.subtask_manager.hri.say(
                        "I lost you, please come back to the front of the camera"
                    )
                    self.current_state = HelpMeCarryTM.TASK_STATES["FIND_GUEST"]
                    break

        if self.current_state == HelpMeCarryTM.TASK_STATES["ASK_TO_POINT_THE_BAG"]:
            self.subtask_manager.hri.cancel_hear_action()
            Logger.state(self, "Asking to point the bag")
            self.subtask_manager.hri.say("Please point to the bag you want me to carry")
            self.current_state = HelpMeCarryTM.TASK_STATES["DETECT_THE_BAG"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["DETECT_THE_BAG"]:
            Logger.state(self, "Detecting the bag")
            self.subtask_manager.hri.say("I will now detect the bag")
            self.subtask_manager.manipulation.move_to_position("nav_pose")
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
            Logger.error(self, description)
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
                self.subtask_manager.hri.say(
                    "Please hang the bag in the white hook that is in the back of my gripper"
                )
                s, confirmation = self.subtask_manager.hri.confirm(
                    "Have you hang the bag? say yes when the speaker turns blue",
                    False,
                    retries=8,
                    wait_between_retries=5,
                )

                if confirmation == "yes":
                    self.subtask_manager.hri.say("ok")
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
            self.subtask_manager.hri.say("I have grasped the bag")

            # MOCK: Self.subtask_manager.manipulation.pick(pose)

        if self.current_state == HelpMeCarryTM.TASK_STATES["RETURN_TO_STARTING_LOCATION"]:
            Logger.state(self, "Returning to starting location")

            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self.subtask_manager.hri.say("Im processing the return path please wait some time")
            # Wait for behavior tree change to complete
            bt_future = self.subtask_manager.nav.change_bt("standard")
            rclpy.spin_until_future_complete(self, bt_future)
            bt_result = bt_future.result()
            if bt_result != Status.EXECUTION_SUCCESS:
                Logger.error(self, "Failed to change behavior tree")
                # Handle error - maybe retry or go to error state
                return
            Logger.info(self, "Behavior tree successfully changed, proceeding with navigation")
            t.sleep(13)
            self.subtask_manager.hri.say("I will now return to the starting location")
            result = Status.EXECUTION_ERROR
            while result == Status.EXECUTION_ERROR:
                future = self.subtask_manager.nav.move_to_zero()
                rclpy.spin_until_future_complete(self, future)
                result = future.result()

            self.current_state = HelpMeCarryTM.TASK_STATES["TEST_END"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["PLACE_THE_BAG"]:
            Logger.state(self, "Placing the bag")
            # MOCK: self.subtask_manager.manipulation.place(pose)
            self.current_state = HelpMeCarryTM.TASK_STATES["END"]

        if self.current_state == HelpMeCarryTM.TASK_STATES["END"]:
            Logger.state(self, "Ending task")
            # self.subtask_manager.hri.say("I have finished my task, I will rest now.")
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
    main()
