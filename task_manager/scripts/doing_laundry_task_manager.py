#!/usr/bin/env python3
"""
Task Manager for Doing Laundry Task

Flow:
  1. Wait for start button → check door open
  2. Navigate to the laundry machine (dishwasher area)
  3. Ask a person to open the machine and wait for confirmation
  4. Approach the machine (backward, ignoring obstacles behind)
  5. Detect laundry inside → obtain centroid / 3D point
  6. Reach in with the arm using the detected point and pick
  7. Back away from the machine
  8. Navigate to the basket area
  9. Place / release the laundry into the basket
 10. Done

Nav areas used: "dishwasher", "basket", "side_table"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3
DETECT_TIMEOUT = 15.0
APPROACH_DISTANCE = 0.30  # metres to stop from the machine when backing in


class DoingLaundryTM(Node):
    """Task Manager for Doing Laundry"""

    class S:
        """State names."""

        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        NAV_TO_MACHINE = "NAV_TO_MACHINE"
        ASK_OPEN_MACHINE = "ASK_OPEN_MACHINE"
        APPROACH_MACHINE = "APPROACH_MACHINE"
        DETECT_LAUNDRY = "DETECT_LAUNDRY"
        PICK_LAUNDRY = "PICK_LAUNDRY"
        BACK_AWAY = "BACK_AWAY"
        NAV_TO_BASKET = "NAV_TO_BASKET"
        PLACE_LAUNDRY = "PLACE_LAUNDRY"
        END = "END"

    def __init__(self):
        super().__init__("doing_laundry_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.DOING_LAUNDRY, mock_areas=[])
        self.state = self.S.WAIT_FOR_BUTTON
        self.running_task = True
        self.laundry_point: PointStamped | None = None
        self.pick_attempts = 0

        # Arm to navigation pose
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        Logger.info(self, "DoingLaundryTM initialised.")

    # -----------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------
    def _nav_pose(self):
        """Prepare the robot for navigation."""
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.move_to_position("nav_pose")

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to a map area following the common task-manager pattern."""
        self._nav_pose()
        if say:
            Logger.info(self, f"Navigating to {location}/{sublocation}")
            self.subtask_manager.hri.say(f"I am navigating to the {location}.", wait=False)
        return self.subtask_manager.nav.move_to_location(location, sublocation)

    # -----------------------------------------------------------------
    # State machine
    # -----------------------------------------------------------------
    def run(self):  # noqa: C901 — flat state machine, complexity is structural
        # ---- WAIT FOR BUTTON ----
        if self.state == self.S.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for start button…")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.", wait=False)
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed")
            self.subtask_manager.nav.check_door()
            self.state = self.S.START

        # ---- START ----
        elif self.state == self.S.START:
            Logger.state(self, "Starting Doing Laundry task")
            self.state = self.S.NAV_TO_MACHINE

        # ---- NAVIGATE TO THE LAUNDRY MACHINE (dishwasher) ----
        elif self.state == self.S.NAV_TO_MACHINE:
            status, error = self.navigate_to("dishwasher")
            if status == Status.EXECUTION_SUCCESS:
                self.state = self.S.ASK_OPEN_MACHINE
            else:
                Logger.error(self, f"Navigation to dishwasher failed: {error}. Retrying…")

        # ---- ASK SOMEONE TO OPEN THE MACHINE ----
        elif self.state == self.S.ASK_OPEN_MACHINE:
            Logger.state(self, "Asking person to open the laundry machine")
            status, answer = self.subtask_manager.hri.confirm(
                "Could you please open the laundry machine for me?",
                use_hotwords=True,
                retries=5,
                wait_between_retries=8.0,
            )
            if status == Status.EXECUTION_SUCCESS and answer == "yes":
                Logger.success(self, "Machine reported open")
                self.state = self.S.APPROACH_MACHINE
            else:
                Logger.warn(self, "No positive confirmation — asking again")

        # ---- APPROACH THE MACHINE (backward, ignoring rear obstacles) ----
        elif self.state == self.S.APPROACH_MACHINE:
            Logger.state(self, "Approaching the laundry machine")
            status, _ = self.subtask_manager.nav.move_backwards_until_object(
                stop_distance=APPROACH_DISTANCE
            )
            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Close enough to the machine")
                self.state = self.S.DETECT_LAUNDRY
            else:
                Logger.warn(self, "Approach failed — trying to detect from here")
                self.state = self.S.DETECT_LAUNDRY

        # ---- DETECT LAUNDRY INSIDE ----
        elif self.state == self.S.DETECT_LAUNDRY:
            Logger.state(self, "Detecting laundry inside the machine")
            self.subtask_manager.manipulation.move_to_position("pick_stare_at_table")
            status, detections = self.subtask_manager.vision.detect_objects(timeout=DETECT_TIMEOUT)
            if status == Status.EXECUTION_SUCCESS and detections:
                # Take the closest detection that has a valid 3-D point
                best = None
                for d in detections:
                    if d.point3d is not None:
                        if best is None or d.distance < best.distance:
                            best = d
                if best is not None:
                    self.laundry_point = best.point3d
                    Logger.info(
                        self,
                        f"Laundry detected ({best.classname}) at "
                        f"x={best.px:.2f} y={best.py:.2f} z={best.pz:.2f}",
                    )
                    self.state = self.S.PICK_LAUNDRY
                else:
                    Logger.warn(self, "Detections without 3D point — retrying")
            else:
                Logger.warn(self, "No laundry detected, retrying…")

        # ---- PICK LAUNDRY FROM INSIDE THE MACHINE ----
        elif self.state == self.S.PICK_LAUNDRY:
            Logger.state(self, "Reaching into the machine to pick laundry")

            # Move the arm to the detected centroid, then close the gripper
            result = self.subtask_manager.manipulation.go_to_hand(
                self.laundry_point, hand_offset=0.05
            )
            if result == Status.EXECUTION_SUCCESS:
                self.subtask_manager.manipulation.close_gripper()
                Logger.success(self, "Laundry grasped")
                self.subtask_manager.manipulation.move_to_position("nav_pose")
                self.state = self.S.BACK_AWAY
            else:
                self.pick_attempts += 1
                if self.pick_attempts >= ATTEMPT_LIMIT:
                    Logger.error(self, "Max pick attempts — asking for help")
                    self.subtask_manager.hri.say(
                        "I could not open the laundry machine. Could you please open it?"
                    )
                    self.subtask_manager.manipulation.open_gripper()
                    _, answer = self.subtask_manager.hri.confirm(
                        "Have you opened it?",
                        use_hotwords=True,
                    )
                    if answer == "yes":
                        self.subtask_manager.manipulation.close_gripper()
                        self.subtask_manager.manipulation.move_to_position("nav_pose")
                        self.state = self.S.BACK_AWAY
                    else:
                        Logger.error(self, "Could not obtain laundry — ending")
                        self.state = self.S.END
                else:
                    Logger.warn(self, f"Pick attempt {self.pick_attempts} failed — retrying")
                    self.state = self.S.DETECT_LAUNDRY

        # ---- BACK AWAY FROM THE MACHINE ----
        elif self.state == self.S.BACK_AWAY:
            Logger.state(self, "Backing away from the laundry machine")
            self._nav_pose()
            status, _ = self.subtask_manager.nav.move_forward(0.5)
            if status != Status.EXECUTION_SUCCESS:
                Logger.warn(self, "Forward move failed — trying nav anyway")
            self.state = self.S.NAV_TO_BASKET

        # ---- NAVIGATE TO THE BASKET ----
        elif self.state == self.S.NAV_TO_BASKET:
            status, error = self.navigate_to("basket")
            if status == Status.EXECUTION_SUCCESS:
                self.state = self.S.PLACE_LAUNDRY
            else:
                Logger.error(self, f"Navigation to basket failed: {error}. Retrying…")

        # ---- PLACE / DROP LAUNDRY INTO THE BASKET ----
        elif self.state == self.S.PLACE_LAUNDRY:
            Logger.state(self, "Placing laundry into the basket")
            self.subtask_manager.manipulation.move_to_position("pick_stare_at_table")
            result = self.subtask_manager.manipulation.open_gripper()
            if result == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Laundry released into the basket")
            else:
                Logger.warn(self, "Gripper open failed — laundry may still be held")
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self.state = self.S.END

        # ---- END ----
        elif self.state == self.S.END:
            Logger.state(self, "Task complete")
            self.subtask_manager.hri.say("The laundry task is finished. Thank you!")
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
