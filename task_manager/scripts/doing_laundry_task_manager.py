#!/usr/bin/env python3
"""
Task Manager for Doing Laundry Task
"""

import math
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3


class DoingLaundryTM(Node):
    """Task Manager for Doing Laundry"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        NAVIGATE_TO_BASKET = "NAVIGATE_TO_BASKET"
        ROTATE_BEHIND_BASKET = "ROTATE_BEHIND_BASKET"
        LOOK_BACK_AND_SCAN = "LOOK_BACK_AND_SCAN"
        PICK_LAUNDRY = "PICK_LAUNDRY"
        NAVIGATE_TO_LAUNDRY_TABLE = "NAVIGATE_TO_LAUNDRY_TABLE"
        UNLOAD_LAUNDRY = "UNLOAD_LAUNDRY"
        END = "END"

    def __init__(self):
        super().__init__("doing_laundry_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.DOING_LAUNDRY, mock_areas=[""])
        self.current_state = DoingLaundryTM.TaskStates.WAIT_FOR_BUTTON
        self.running_task = True
        self.pick_attempts = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subtask_manager.manipulation.move_to_position("nav_pose")
        Logger.info(self, "DoingLaundryTM has started.")

    # ------------------------------------------------------------------ navigation

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

    def _rotate_180(self):
        """Rotate 180° in place so the robot's back faces the basket."""
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
        except TransformException as e:
            Logger.error(self, f"TF base_link→map failed for rotation: {e}")
            return Status.EXECUTION_ERROR

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y
        q = robot_tf.transform.rotation
        current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2),
        )
        new_yaw = current_yaw + math.pi

        Logger.info(
            self,
            f"Rotating 180°: {math.degrees(current_yaw):.1f}° → {math.degrees(new_yaw):.1f}°",
        )
        self.subtask_manager.hri.say("Turning to face basket.", wait=False)
        status, error = self.subtask_manager.nav.navigate_to_pose(rx, ry, new_yaw)
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"Rotation failed: {error}")
        return status

    # ------------------------------------------------------------------ FSM

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
            self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET:
            Logger.info(self, "Navigating to basket area")

            # status, error = self.navigate_to("laundry", "laundry_basket")
            status, error = self.navigate_to("living_room", "couches")

            if status == Status.EXECUTION_SUCCESS:
                # self.current_state = DoingLaundryTM.TaskStates.ROTATE_BEHIND_BASKET
                self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN
            else:
                Logger.error(self, f"Navigation failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.ROTATE_BEHIND_BASKET:
            Logger.info(self, "Rotating 180° so back faces basket.")
            status = self._rotate_180()

            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Rotation complete. Back now faces basket.")
                self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN
            else:
                Logger.error(self, "Rotation failed. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN:
            Logger.info(self, "Moving to look_back_stare for basket pick.")
            self.subtask_manager.manipulation.move_to_position("look_back_stare")
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
            self.current_state = DoingLaundryTM.TaskStates.PICK_LAUNDRY

        elif self.current_state == DoingLaundryTM.TaskStates.PICK_LAUNDRY:
            Logger.info(self, "Requesting integrated basket pick.")
            self.subtask_manager.hri.say("Picking up the laundry basket.", wait=False)

            result = self.subtask_manager.manipulation.pick_object("aundry_basket")

            if result == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Basket picked.")
                Logger.info(self, "Moving to basket_hold_back_pose for transport.")
                self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_TABLE
            else:
                self.pick_attempts += 1
                if self.pick_attempts >= ATTEMPT_LIMIT:
                    Logger.error(self, "Basket pick failed after max attempts. Ending.")
                    self.current_state = DoingLaundryTM.TaskStates.END
                else:
                    Logger.warn(
                        self,
                        f"Basket pick failed (attempt {self.pick_attempts}), re-scanning.",
                    )
                    self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_TABLE:
            Logger.info(self, "Navigating to laundry table while holding basket.")
            status, error = self.navigate_holding("kitchen", "breakfast_surface")

            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Reached laundry table.")
                self.current_state = DoingLaundryTM.TaskStates.UNLOAD_LAUNDRY
            else:
                Logger.error(self, f"Navigation to table failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.UNLOAD_LAUNDRY:
            Logger.info(self, "Opening gripper to release basket at table.")
            self.subtask_manager.manipulation.open_gripper()
            self.subtask_manager.hri.say("Basket delivered to the table.", wait=False)
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
