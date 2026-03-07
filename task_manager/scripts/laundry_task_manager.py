#!/usr/bin/env python3

"""
Task Manager for Doing Laundry Challenge of Robocup @Home 2026
"""

import time

import rclpy
from rclpy.node import Node

from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task


class DoingLaundryTM(Node):
    """Class to manage the Doing Laundry Challenge task"""

    class States:
        WAITING_FOR_BUTTON = -1
        START = 0
        PICKING_CLOTHES_FROM_BASKET= 1
        PICKING_CLOTHES_FROM_LM = 2
        PLACING_CLOTHES_IN_TABLE = 3
        DONE = 5

    def __init__(self):
        """Initialize the node"""
        super().__init__("doing_laundry_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.DOING_LAUNDRY, mock_areas=[""])

        self.current_state = (
            DoingLaundryTM.States.WAITING_FOR_BUTTON
        )
        self.prev_state = None
        self.running_task = True

        Logger.info(self, "DoingLaundryTM has started.")

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        if say:
            self.subtask_manager.hri.say(
                f"I will now guide you to the {location}. Please follow me."
            )
            self.subtask_manager.manipulation.follow_face(False)

        self.subtask_manager.manipulation.move_joint_positions(
            named_position="nav_pose", velocity=0.5, degrees=True
        )
        self.subtask_manager.nav.resume_nav()
        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        if "navigation" not in self.subtask_manager.get_mocked_areas():
            rclpy.spin_until_future_complete(self.subtask_manager.nav.node, future)

        self.subtask_manager.nav.pause_nav()

    def run(self):
        """State machine"""
        # Wait for start button
        if self.current_state == DoingLaundryTM.States.WAITING_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.")
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.current_state = DoingLaundryTM.States.START
            Logger.success(self, "Start button pressed, task will begin now")

        # Wait for door to open
        if self.current_state == DoingLaundryTM.States.START:
            res = "closed"
            while res == "closed":
                time.sleep(1)
                status, res = self.subtask_manager.nav.check_door()
                if status == Status.EXECUTION_SUCCESS:
                    Logger.info(self, f"Door status: {res}")
                else:
                    Logger.error(self, "Failed to check door status")
            # Go to basket
            self.navigate_to("basket", "", False)
            self.current_state = DoingLaundryTM.States.PICKING_CLOTHES_FROM_BASKET

        # Pick all clothes from basket and place on table
        if self.current_state == DoingLaundryTM.States.PICKING_CLOTHES_FROM_BASKET:
            self.prev_state = self.current_state
            pick_attempts = 0
            # Move arm to look at basket
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="basket_stare", velocity=0.5, degrees=True
            )

            basket_empty = False
            while not basket_empty:
                # TODO: Detect clothes in basket
                # status, clothes = self.subtask_manager.vision.detect_clothes_in_basket()
                # if not clothes:
                #     basket_empty = True
                #     break
                # Pick one cloth
                pick_result = self.subtask_manager.manipulation.pick_from_basket()
                if pick_result != Status.EXECUTION_SUCCESS:
                    pick_attempts += 1
                    Logger.error(self, f"Failed to pick cloth from basket, attempt {pick_attempts}")
                    if pick_attempts >= 3:
                        while True:
                            self.subtask_manager.manipulation.open_gripper()
                            s, confirmation = self.subtask_manager.hri.confirm(
                                "Have you given me the cloth? Say yes when you are done.",
                                False,
                                retries=5,
                                wait_between_retries=2,
                            )

                            if confirmation == "yes":
                                self.subtask_manager.hri.say("Thank you.")
                                break
                        self.subtask_manager.manipulation.close_gripper()
                        self.subtask_manager.hri.say(
                            "I have received the cloth. I will now place it on the table."
                        )       
                # Move to table
                self.navigate_to("laundry_room", "table", False)
                # Place on table
                self.subtask_manager.manipulation.move_joint_positions(
                    named_position="table_stare", velocity=0.5, degrees=True
                )
                self.subtask_manager.manipulation.place_on_table()
                self.subtask_manager.hri.say("Placed a cloth on the table.")
                # Return to basket
                self.navigate_to("laundry_room", "basket", False)
            # After basket is empty, go to laundry machine
            self.navigate_to("laundry_room", "laundry_machine", False)
            self.current_state = DoingLaundryTM.States.PICKING_CLOTHES_FROM_LM

        # Pick all clothes from laundry machine and place in basket
        if self.current_state == DoingLaundryTM.States.PICKING_CLOTHES_FROM_LM:
            self.prev_state = self.current_state
            pick_attempts = 0
            # Move arm to look at laundry machine
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="laundry_machine_stare", velocity=0.5, degrees=True
            )
            self.subtask_manager.hri.say("I am now picking clothes from the laundry machine.")
            laundry_empty = False
            while not laundry_empty:
                # TODO: Detect clothes in laundry machine 
                # status, clothes = self.subtask_manager.vision.detect_clothes_in_laundry()
                # if not clothes:
                #     laundry_empty = True
                #     break
                pick_result = self.subtask_manager.manipulation.pick_from_laundry_machine()
                if pick_result != Status.EXECUTION_SUCCESS:
                    pick_attempts += 1
                    Logger.error(self, f"Failed to pick cloth from laundry machine, attempt {pick_attempts}")
                    if pick_attempts >= 3:
                        while True:
                            self.subtask_manager.manipulation.open_gripper()
                            s, confirmation = self.subtask_manager.hri.confirm(
                                "Have you given me the cloth? Say yes when you are done.",
                                False,
                                retries=5,
                                wait_between_retries=2,
                            )

                            if confirmation == "yes":
                                self.subtask_manager.hri.say("Thank you.")
                                break
                        self.subtask_manager.manipulation.close_gripper()
                        self.subtask_manager.hri.say(
                            "I have received the cloth. I will now place it on the table."
                        )    
                # Move to table
                self.navigate_to("laundry_room", "table", False)
                # Place on table
                self.subtask_manager.manipulation.move_joint_positions(
                    named_position="table_stare", velocity=0.5, degrees=True
                )
                self.subtask_manager.manipulation.place_on_table()
                self.subtask_manager.hri.say("Placed a cloth on the table.")
                # Return to laundry machine
                self.navigate_to("laundry_room", "laundry_machine", False)
            # After laundry is empty, end task
            self.current_state = DoingLaundryTM.States.DONE

        # Done state
        if self.current_state == DoingLaundryTM.States.DONE:
            self.subtask_manager.hri.say(
                "I am done with the task.",
                wait=False,
            )
            self.running_task = False

def main(args=None):
    """Main function"""
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
