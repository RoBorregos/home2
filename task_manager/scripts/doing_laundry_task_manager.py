#!/usr/bin/env python3
"""
Task Manager for Doing Laundry Task
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task
from frida_constants.manipulation_constants import CLOTHES_BASKET_EXIT_HEIGHT

ATTEMPT_LIMIT = 3
# Pick clothes from the basket and place on the table.
BASKET_PLACE_ROUNDS = 2
# Pick clothes in Washing Machine and bring to the table.
WM_PLACE_ROUNDS = 1


class DoingLaundryTM(Node):
    """Task Manager for Doing Laundry"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        NAVIGATE_TO_BASKET = "NAVIGATE_TO_BASKET"
        PICK_LAUNDRY_BASKET = "PICK_LAUNDRY_BASKET"
        NAVIGATE_TO_LAUNDRY_TABLE = "NAVIGATE_TO_LAUNDRY_TABLE"
        UNLOAD_LAUNDRY = "UNLOAD_LAUNDRY"
        PICK_CLOTHES_BASKET = "PICK_CLOTHES_BASKET"
        PLACE_CLOTHES_TABLE = "PLACE_CLOTHES_TABLE"
        NAVIGATE_TO_LAUNDRY_MACHINE = "NAVIGATE_TO_LAUNDRY_MACHINE"
        PICK_CLOTHES_WM = "PICK_CLOTHES_WM"
        CLOSE_LAUNDRY_MACHINE = "CLOSE_LAUNDRY_MACHINE"
        NAVIGATE_TO_TABLE_WITH_CLOTHES = "NAVIGATE_TO_TABLE_WITH_CLOTHES"
        END = "END"

    def __init__(self):
        super().__init__("doing_laundry_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.DOING_LAUNDRY, mock_areas=[])
        self.current_state = DoingLaundryTM.TaskStates.WAIT_FOR_BUTTON
        self.running_task = True

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Per-action retry counters so basket/clothes failures don't share budget.
        self.basket_pick_attempts = 0
        self.clothes_pick_attempts = 0

        # Round counters for the two place loops.
        self.basket_placed = 0
        self.wm_placed = 0

        self.dock_offsets: dict[str, float] = {
            "laundry_basket": 0.0,
            "folding_surface": 0.0,
            "laundry_machine": 0.0,
        }

        self.subtask_manager.manipulation.move_to_position("nav_pose")
        Logger.info(self, "DoingLaundryTM has started.")

    def _dock_if_surface(self, sublocation: str):
        """Dock perpendicular to the surface if it's a known pick/place sublocation."""
        if sublocation not in self.dock_offsets:
            return
        offset = self.dock_offsets[sublocation]
        dock_status, dock_error = self.subtask_manager.nav.dock_table(offset=offset)
        if dock_status != Status.EXECUTION_SUCCESS:
            Logger.warn(self, f"Docking to {sublocation} failed: {dock_error}")

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to location, resetting arm to nav_pose first."""
        self.subtask_manager.vision.deactivate_face_recognition()
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        if say:
            Logger.info(self, f"Moving to {sublocation} in {location}")
            self.subtask_manager.hri.say(f"Navigating to {sublocation}.", wait=False)
        result, error = self.subtask_manager.nav.move_to_location(location, sublocation)
        if result == Status.EXECUTION_SUCCESS:
            self._dock_if_surface(sublocation)
        return result, error

    def navigate_holding(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate while holding basket — does NOT reset arm to nav_pose."""
        self.subtask_manager.vision.deactivate_face_recognition()
        self.subtask_manager.manipulation.follow_face(False)
        if say:
            Logger.info(self, f"Carrying basket to {sublocation} in {location}")
            self.subtask_manager.hri.say(f"Carrying basket to {sublocation}.", wait=False)
        result, error = self.subtask_manager.nav.move_to_location(location, sublocation)
        if result == Status.EXECUTION_SUCCESS:
            self._dock_if_surface(sublocation)
        return result, error

    def next_state_after_place(self):
        """Decide where to go after placing clothes on the table."""
        if self.basket_placed < BASKET_PLACE_ROUNDS:
            Logger.info(
                self,
                f"Basket round {self.basket_placed}/{BASKET_PLACE_ROUNDS}. Next: pick from basket.",
            )
            return DoingLaundryTM.TaskStates.PICK_CLOTHES_BASKET
        if self.wm_placed < WM_PLACE_ROUNDS:
            Logger.info(
                self,
                f"WM round {self.wm_placed}/{WM_PLACE_ROUNDS}. Next: navigate to laundry machine.",
            )
            return DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_MACHINE
        Logger.success(self, "All place rounds completed.")
        return DoingLaundryTM.TaskStates.END

    def run(self):
        if self.current_state == DoingLaundryTM.TaskStates.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.", wait=False)
            self.subtask_manager.manipulation.move_to_position("nav_pose")

            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, Doing Laundry task will begin now")
            self.current_state = DoingLaundryTM.TaskStates.START

        elif self.current_state == DoingLaundryTM.TaskStates.START:
            Logger.state(self, "Starting Doing Laundry Task")
            self.navigate_to("laundry", "safe_place")
            self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET:
            Logger.info(self, "Navigating to basket area")
            status, error = self.navigate_to("laundry", "laundry_basket")
            if status == Status.EXECUTION_SUCCESS:
                self.current_state = DoingLaundryTM.TaskStates.PICK_LAUNDRY_BASKET
            else:
                Logger.error(self, f"Navigation failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.PICK_LAUNDRY_BASKET:
            Logger.info(self, "Requesting integrated basket pick.")
            self.subtask_manager.hri.say("Picking up the laundry basket.", wait=False)
            result = self.subtask_manager.manipulation.pick_object("laundry_basket")

            if result == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Basket picked.")
                self.basket_pick_attempts = 0
                self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_TABLE
            else:
                self.basket_pick_attempts += 1
                if self.basket_pick_attempts >= ATTEMPT_LIMIT:
                    Logger.error(self, "Basket pick failed after max attempts. Ending.")
                    self.current_state = DoingLaundryTM.TaskStates.END
                else:
                    Logger.warn(
                        self,
                        f"Basket pick failed (attempt {self.basket_pick_attempts}), re-scanning.",
                    )

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_TABLE:
            Logger.info(self, "Navigating to laundry table while holding basket.")
            status, error = self.navigate_holding("laundry", "folding_surface")
            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Reached laundry table with basket.")
                self.current_state = DoingLaundryTM.TaskStates.UNLOAD_LAUNDRY
            else:
                Logger.error(self, f"Navigation to table failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.UNLOAD_LAUNDRY:
            Logger.info(self, "Opening gripper to release basket.")
            self.subtask_manager.manipulation.open_gripper()
            self.subtask_manager.hri.say("Basket delivered to the table.", wait=False)
            self.subtask_manager.manipulation.move_arm_vertical(
                CLOTHES_BASKET_EXIT_HEIGHT, descend=False
            )
            self.subtask_manager.manipulation.move_to_position("look_side_low_stare")
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
            self.current_state = DoingLaundryTM.TaskStates.PICK_CLOTHES_BASKET

        elif self.current_state == DoingLaundryTM.TaskStates.PICK_CLOTHES_BASKET:
            Logger.info(
                self,
                f"Picking clothes from basket ({self.basket_placed + 1}/{BASKET_PLACE_ROUNDS}).",
            )
            self.subtask_manager.hri.say("Picking clothes from the basket.", wait=False)
            result = self.subtask_manager.manipulation.pick_object("clothes")

            if result == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Clothes picked from basket.")
                self.clothes_pick_attempts = 0
                self.current_state = DoingLaundryTM.TaskStates.PLACE_CLOTHES_TABLE
            else:
                self.clothes_pick_attempts += 1
                if self.clothes_pick_attempts >= ATTEMPT_LIMIT:
                    Logger.error(self, "Clothes pick from basket failed. Ending.")
                    self.current_state = DoingLaundryTM.TaskStates.END
                else:
                    Logger.warn(
                        self,
                        f"Basket clothes pick failed (attempt {self.clothes_pick_attempts}), retrying.",
                    )

        elif self.current_state == DoingLaundryTM.TaskStates.PLACE_CLOTHES_TABLE:
            Logger.info(self, "Placing clothes on the laundry table.")
            self.subtask_manager.hri.say("Placing clothes on the table.", wait=False)
            self.subtask_manager.manipulation.move_to_position("nav_carry_bag_pose")
            result = self.subtask_manager.manipulation.place(from_current=True)

            if result == Status.EXECUTION_SUCCESS:
                # Attribute the placement to whichever loop we are currently in.
                if self.basket_placed < BASKET_PLACE_ROUNDS:
                    self.basket_placed += 1
                    Logger.success(
                        self,
                        f"Basket→table done {self.basket_placed}/{BASKET_PLACE_ROUNDS}.",
                    )
                else:
                    self.wm_placed += 1
                    Logger.success(
                        self,
                        f"WM→table done {self.wm_placed}/{WM_PLACE_ROUNDS}.",
                    )
                self.current_state = self.next_state_after_place()
            else:
                Logger.error(self, "Place clothes failed. Ending task.")
                self.current_state = DoingLaundryTM.TaskStates.END

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_MACHINE:
            Logger.info(self, "Navigating to laundry machine.")
            status, error = self.navigate_to("laundry", "laundry_machine")
            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Reached laundry machine.")
                self.current_state = DoingLaundryTM.TaskStates.PICK_CLOTHES_WM
            else:
                Logger.error(self, f"Navigation to laundry machine failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.PICK_CLOTHES_WM:
            Logger.info(
                self,
                f"Picking clothes from washing machine ({self.wm_placed + 1}/{WM_PLACE_ROUNDS}).",
            )
            self.subtask_manager.hri.say("Picking clothes from the washing machine.", wait=False)
            result = self.subtask_manager.manipulation.pick_object("clothes")

            if result == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Clothes picked from washing machine.")
                self.clothes_pick_attempts = 0
                # Close the door now while we are still in front of the machine,
                # but only once we have finished all WM pick rounds.
                if (self.wm_placed + 1) >= WM_PLACE_ROUNDS:
                    self.current_state = DoingLaundryTM.TaskStates.CLOSE_LAUNDRY_MACHINE
                else:
                    self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_TABLE_WITH_CLOTHES
            else:
                self.clothes_pick_attempts += 1
                if self.clothes_pick_attempts >= ATTEMPT_LIMIT:
                    Logger.error(self, "Clothes pick from WM failed. Ending.")
                    self.current_state = DoingLaundryTM.TaskStates.END
                else:
                    Logger.warn(
                        self,
                        f"WM clothes pick failed (attempt {self.clothes_pick_attempts}), retrying.",
                    )

        elif self.current_state == DoingLaundryTM.TaskStates.CLOSE_LAUNDRY_MACHINE:
            Logger.info(self, "Closing laundry machine door.")
            self.subtask_manager.hri.say("Closing the laundry machine door.", wait=False)
            self.subtask_manager.manipulation.close_gripper()
            # self.subtask_manager.manipulation.move_to_position("initial_close_laundry_pose")
            # self.subtask_manager.manipulation.move_to_position("mid_close_laundry_pose")
            # self.subtask_manager.manipulation.move_to_position("end_close_laundry_pose")
            Logger.success(self, "Laundry machine door closed.")
            self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_TABLE_WITH_CLOTHES

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_TABLE_WITH_CLOTHES:
            Logger.info(self, "Navigating back to the laundry table with clothes.")
            status, error = self.navigate_holding("laundry", "folding_surface")
            if status == Status.EXECUTION_SUCCESS:
                self.current_state = DoingLaundryTM.TaskStates.PLACE_CLOTHES_TABLE
            else:
                Logger.error(self, f"Navigation back to table failed: {error}. Retrying...")

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
