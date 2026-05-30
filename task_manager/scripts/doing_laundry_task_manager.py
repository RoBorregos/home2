#!/usr/bin/env python3
"""
Task Manager for Doing Laundry Task
"""

import math
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point  # noqa: F401
from geometry_msgs.msg import PointStamped
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

APPROACH_DIST = 0.55  # meters from basket center to robot base_link when backing in

ATTEMPT_LIMIT = 3


class DoingLaundryTM(Node):
    """Task Manager for Doing Laundry"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        NAVIGATE_TO_BASKET = "NAVIGATE_TO_BASKET"
        DETECT_BASKET = "DETECT_BASKET"
        NAVIGATE_BEHIND_BASKET = "NAVIGATE_BEHIND_BASKET"
        LOOK_BACK_AND_DETECT = "LOOK_BACK_AND_DETECT"
        PICK_LAUNDRY = "PICK_LAUNDRY"
        NAVIGATE_TO_LAUNDRY_TABLE = "NAVIGATE_TO_LAUNDRY_TABLE"
        UNLOAD_LAUNDRY = "UNLOAD_LAUNDRY"
        END = "END"

    def __init__(self):
        super().__init__("doing_laundry_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.DOING_LAUNDRY, mock_areas=[])
        self.current_state = DoingLaundryTM.TaskStates.WAIT_FOR_BUTTON
        self.running_task = True
        self.state_start_time = None
        self.state_times = {}
        self.previous_state = None
        self.basket_detection = None
        self.basket_close_detection = None
        self.detect_attempts = 0
        self.look_back_detect_attempts = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subtask_manager.manipulation.move_to_position("nav_pose")
        Logger.info(self, "DoingLaundryTM has started.")

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to location, resetting arm to nav_pose first."""
        self.subtask_manager.vision.deactivate_face_recognition()
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        if say:
            Logger.info(self, f"Moving to {location}")
            self.subtask_manager.hri.say(f"Navigating to {location}.", wait=False)
        return self.subtask_manager.nav.move_to_location(location, sublocation)

    def navigate_holding(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate while holding basket — does NOT reset arm to nav_pose."""
        self.subtask_manager.vision.deactivate_face_recognition()
        self.subtask_manager.manipulation.follow_face(False)
        if say:
            Logger.info(self, f"Carrying basket to {location}")
            self.subtask_manager.hri.say(f"Carrying basket to {location}.", wait=False)
        return self.subtask_manager.nav.move_to_location(location, sublocation)

    def _compute_behind_basket_pose(self, basket_detection):
        """
        Compute nav goal so robot backs up to basket (back faces basket).

        Uses basket point3d (camera frame) → map frame via TF, then places
        the robot at APPROACH_DIST from basket on the robot's current side,
        oriented away from basket so the arm can reach it from behind.

        Returns (x, y, yaw) in map frame, or None on TF failure.
        """
        try:
            basket_stamped = PointStamped()
            basket_stamped.header = basket_detection.point3d.header
            basket_stamped.point = basket_detection.point3d.point

            basket_map = self.tf_buffer.transform(
                basket_stamped, "map", timeout=Duration(seconds=1.0)
            )
        except TransformException as e:
            Logger.error(self, f"TF basket→map failed: {e}")
            return None

        bx, by = basket_map.point.x, basket_map.point.y

        try:
            robot_tf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            rx = robot_tf.transform.translation.x
            ry = robot_tf.transform.translation.y
        except TransformException as e:
            Logger.error(self, f"TF base_link→map failed: {e}")
            return None

        dx, dy = rx - bx, ry - by
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.001:
            Logger.error(self, "Robot too close to basket to compute approach direction")
            return None
        nx, ny = dx / dist, dy / dist

        approach_x = bx + nx * APPROACH_DIST
        approach_y = by + ny * APPROACH_DIST

        # Yaw: face AWAY from basket (back toward basket)
        yaw = math.atan2(ny, nx)

        Logger.info(
            self,
            f"Behind-basket pose: ({approach_x:.2f}, {approach_y:.2f}, {math.degrees(yaw):.1f}°) "
            f"| basket=({bx:.2f},{by:.2f}) robot=({rx:.2f},{ry:.2f})",
        )
        return approach_x, approach_y, yaw

    def run(self):
        if self.current_state == DoingLaundryTM.TaskStates.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.", wait=False)

            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, Doing Laundry task will begin now")
            self.current_state = DoingLaundryTM.TaskStates.START

        elif self.current_state == DoingLaundryTM.TaskStates.START:
            Logger.state(self, "Starting Doing Laundry Task")
            self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET:
            Logger.info(self, "Navigating to basket area")
            status, error = self.navigate_to("laundry", "basket_view")

            if status == Status.EXECUTION_SUCCESS:
                self.current_state = DoingLaundryTM.TaskStates.DETECT_BASKET
            else:
                Logger.error(self, f"Navigation failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.DETECT_BASKET:
            Logger.info(self, "Detecting laundry basket to compute behind-basket pose.")
            status, basket_detection = self.subtask_manager.vision.detect_laundry_basket()

            if status == Status.EXECUTION_SUCCESS and basket_detection:
                Logger.info(self, f"Basket detected: {basket_detection.classname}")
                self.basket_detection = basket_detection
                self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_BEHIND_BASKET
            else:
                self.detect_attempts += 1
                if self.detect_attempts >= ATTEMPT_LIMIT:
                    Logger.error(self, "Basket detection failed after max attempts. Ending.")
                    self.current_state = DoingLaundryTM.TaskStates.END
                else:
                    Logger.warn(
                        self, f"Basket not detected (attempt {self.detect_attempts}), retrying..."
                    )

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_BEHIND_BASKET:
            Logger.info(self, "Computing behind-basket approach pose.")
            pose = self._compute_behind_basket_pose(self.basket_detection)

            if pose is None:
                Logger.error(self, "Could not compute behind-basket pose. Re-detecting.")
                self.current_state = DoingLaundryTM.TaskStates.DETECT_BASKET
            else:
                approach_x, approach_y, yaw = pose
                self.subtask_manager.manipulation.move_to_position("nav_pose")
                self.subtask_manager.hri.say("Positioning behind the basket.", wait=False)
                status, error = self.subtask_manager.nav.navigate_to_pose(
                    approach_x, approach_y, yaw
                )

                if status == Status.EXECUTION_SUCCESS:
                    Logger.success(self, "Reached behind-basket position.")
                    self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_DETECT
                else:
                    Logger.error(self, f"Failed to reach behind-basket pose: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.LOOK_BACK_AND_DETECT:
            Logger.info(self, "Moving to look_back_stare to detect basket from behind.")
            result = self.subtask_manager.manipulation.move_to_position("look_back_stare")

            if result != Status.EXECUTION_SUCCESS:
                Logger.error(self, "Failed to move to look_back_stare. Ending.")
                self.current_state = DoingLaundryTM.TaskStates.END
                return

            Logger.info(self, "Detecting basket from look_back_stare pose.")
            status, basket_close = self.subtask_manager.vision.detect_laundry_basket(timeout=8)

            if status == Status.EXECUTION_SUCCESS and basket_close:
                Logger.info(
                    self,
                    f"Close basket detected at: {basket_close.point3d.point}",
                )
                self.basket_close_detection = basket_close
                self.current_state = DoingLaundryTM.TaskStates.PICK_LAUNDRY
            else:
                self.look_back_detect_attempts += 1
                if self.look_back_detect_attempts >= ATTEMPT_LIMIT:
                    Logger.error(
                        self, "Could not detect basket from behind after max attempts. Ending."
                    )
                    self.current_state = DoingLaundryTM.TaskStates.END
                else:
                    Logger.warn(
                        self,
                        f"Basket not detected from behind (attempt {self.look_back_detect_attempts}), retrying...",
                    )

        elif self.current_state == DoingLaundryTM.TaskStates.PICK_LAUNDRY:
            if self.basket_close_detection is None:
                Logger.error(self, "No close basket detection available. Re-detecting.")
                self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_DETECT
                return

            Logger.info(self, "Opening gripper before approach.")
            self.subtask_manager.manipulation.open_gripper()

            Logger.info(self, "Moving arm to basket via go_to_hand.")
            result = self.subtask_manager.manipulation.go_to_hand(
                point=self.basket_close_detection.point3d,
                hand_offset=0.05,
            )

            if result != Status.EXECUTION_SUCCESS:
                Logger.error(self, "go_to_hand failed. Retrying detection.")
                self.basket_close_detection = None
                self.look_back_detect_attempts = 0
                self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_DETECT
                return

            Logger.info(self, "Closing gripper to grab basket.")
            self.subtask_manager.manipulation.close_gripper()

            Logger.info(self, "Moving to basket_hold_back_pose for safe navigation.")
            self.subtask_manager.manipulation.move_to_position("basket_hold_back_pose")

            Logger.success(self, "Basket grabbed. Heading to laundry table.")
            self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_TABLE

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_TABLE:
            Logger.info(self, "Navigating to laundry table while holding basket.")
            status, error = self.navigate_holding("laundry", "laundry_table")

            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Reached laundry table.")
                self.current_state = DoingLaundryTM.TaskStates.UNLOAD_LAUNDRY
            else:
                Logger.error(self, f"Navigation to table failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.UNLOAD_LAUNDRY:
            Logger.info(self, "Opening gripper to release basket at table.")
            self.subtask_manager.manipulation.open_gripper()
            self.subtask_manager.hri.say("Basket delivered to the table.", wait=False)
            # Arm stays in basket_hold_back_pose — no nav_pose reset per task spec
            self.current_state = DoingLaundryTM.TaskStates.END

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
