#!/usr/bin/env python3
"""
Task Manager for Doing Laundry Task
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task
from frida_constants.manipulation_constants import CLOTHES_BASKET_EXIT_HEIGHT

ATTEMPT_LIMIT = 3
# Pick clothes from the basket and place on the table.
BASKET_PLACE_ROUNDS = 2
# Pick clothes in Washing Machine and bring to the table.
WM_PLACE_ROUNDS = 1

# --- Washing-machine insert-and-pick parameters (tune on the robot) ---
# Vision subject for the drum opening centroid.
WM_SUBJECT = "round container entrance of color orange"
# Look pose used to snapshot the centroid before aligning the arm.
WM_LOOK_POSE = "front_low_stare"
# Pre-pose handed to the arrow-alignment solver.
WM_ARROW_POSE = "washing_machine_arrow_pose"
# Gripper frame used to measure how far the arm reaches forward after aligning.
WM_GRIPPER_FRAME = "gripper_grasp_frame"
# Extra distance (m) past the opening so the gripper AND joint5 end up inside.
WM_INSERT_DEPTH = 0.10
# Safety clamp on the computed forward push (m).
WM_MAX_INSERT = 0.40
# Wrist pitch (deg) applied to look down into the drum, then undone to straighten.
WM_LOOKDOWN_DEG = 90.0


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

        self.subtask_manager.manipulation.move_to_position("nav_pose")
        Logger.info(self, "DoingLaundryTM has started.")

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to location, resetting arm to nav_pose first."""
        self.subtask_manager.vision.deactivate_face_recognition()
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        if say:
            Logger.info(self, f"Moving to {sublocation} in {location}")
            self.subtask_manager.hri.say(f"Navigating to {sublocation}.", wait=False)
        return self.subtask_manager.nav.move_to_location(location, sublocation)

    def navigate_holding(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate while holding basket — does NOT reset arm to nav_pose."""
        self.subtask_manager.vision.deactivate_face_recognition()
        self.subtask_manager.manipulation.follow_face(False)
        if say:
            Logger.info(self, f"Carrying basket to {sublocation} in {location}")
            self.subtask_manager.hri.say(f"Carrying basket to {sublocation}.", wait=False)
        return self.subtask_manager.nav.move_to_location(location, sublocation)

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

    def _pick_clothes_in_washing_machine(self):
        """Align the arm at the drum opening, drive the base in so the gripper and
        joint5 enter the drum, rotate the wrist down, close to grab clothes, rotate
        back to horizontal, then back the base out the same distance.

        Returns Status.EXECUTION_SUCCESS / Status.EXECUTION_ERROR.
        """
        man = self.subtask_manager.manipulation
        vis = self.subtask_manager.vision
        nav = self.subtask_manager.nav

        # 1. Look pose + snapshot the opening centroid BEFORE moving (the ZED
        #    travels with the arm, so base_link is the only frame valid afterwards).
        man.move_to_position(WM_LOOK_POSE)
        vis.camera_upside_down(False)
        point = vis.get_moondream_point_3d(WM_SUBJECT)
        if point is None or point.header.frame_id == "":
            Logger.error(self, "WM: no drum-opening centroid returned")
            return Status.EXECUTION_ERROR

        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link",
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            opening = do_transform_point(point, tf)
            opening.header.frame_id = "base_link"
        except Exception as e:
            Logger.error(self, f"WM: TF {point.header.frame_id} -> base_link failed: {e}")
            return Status.EXECUTION_ERROR
        Logger.info(
            self,
            f"WM opening in base_link: ({opening.point.x:.3f}, "
            f"{opening.point.y:.3f}, {opening.point.z:.3f})",
        )

        # 2. Aim the arm shaft + gripper approach axis at the opening.
        if (
            man.align_arm_toward_centroid(opening, pre_pose=WM_ARROW_POSE)
            != Status.EXECUTION_SUCCESS
        ):
            Logger.error(self, "WM: align_arm_toward_centroid failed")
            return Status.EXECUTION_ERROR

        # 3. Forward push = how far the opening still is beyond the gripper, plus a
        #    margin so the gripper AND joint5 clear the rim. Derived from the actual
        #    aligned pose (scan pose differs from the insertion pose).
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        try:
            g = self.tf_buffer.lookup_transform(
                "base_link",
                WM_GRIPPER_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            ).transform.translation
        except Exception as e:
            Logger.error(self, f"WM: TF base_link <- {WM_GRIPPER_FRAME} failed: {e}")
            return Status.EXECUTION_ERROR
        forward = (opening.point.x - g.x) + WM_INSERT_DEPTH
        forward = max(0.0, min(forward, WM_MAX_INSERT))
        Logger.info(
            self,
            f"WM: gripper x={g.x:.3f}, opening x={opening.point.x:.3f} -> "
            f"forward push {forward:.3f} m",
        )

        # 4. Open, drive in, look down, grab, straighten, drive back out.
        man.open_gripper()
        status, error = nav.move_relative(forward, backward=False)
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"WM: forward insert failed: {error}")
            return Status.EXECUTION_ERROR

        man.rotate_wrist_pitch(WM_LOOKDOWN_DEG)
        man.close_gripper()
        man.rotate_wrist_pitch(-WM_LOOKDOWN_DEG)

        status, error = nav.move_relative(forward, backward=True)
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"WM: retreat failed: {error}")
            return Status.EXECUTION_ERROR

        Logger.success(self, "WM: insert-and-pick sequence complete")
        return Status.EXECUTION_SUCCESS

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
            result = self.subtask_manager.manipulation.place()

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
            result = self._pick_clothes_in_washing_machine()

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
