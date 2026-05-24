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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initial pose
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        Logger.info(self, "DoingLaundryTM has started.")

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location following the pattern from HRIC"""
        self.subtask_manager.vision.deactivate_face_recognition()
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        if say:
            Logger.info(self, f"Moving to {location}")
            self.subtask_manager.hri.say(f"Navigating to {location}.", wait=False)
        return self.subtask_manager.nav.move_to_location(location, sublocation)

    def _compute_behind_basket_pose(self, basket_detection):
        """
        Compute nav goal so robot backs up to basket (back faces basket).

        Uses basket point3d (camera frame) → map frame via TF, then places
        the robot at APPROACH_DIST from basket on the robot's current side,
        oriented away from basket so the arm can drag it from behind.

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

        # Unit vector from basket toward robot (robot's approach side)
        dx, dy = rx - bx, ry - by
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.001:
            Logger.error(self, "Robot too close to basket to compute approach direction")
            return None
        nx, ny = dx / dist, dy / dist

        # Place robot at APPROACH_DIST from basket, same side as current robot pos
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

            # Wait for the start button to be pressed
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, Doing Laundry task will begin now")
            self.current_state = DoingLaundryTM.TaskStates.START

        elif self.current_state == DoingLaundryTM.TaskStates.START:
            Logger.state(self, "Starting Doing Laundry Task")
            self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET:
            Logger.info(self, "Navigating to basket area")
            status, error = self.navigate_to("laundry", "dishwasher")

            if status == Status.EXECUTION_SUCCESS:
                self.current_state = DoingLaundryTM.TaskStates.DETECT_BASKET
            else:
                Logger.error(self, f"Navigation failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.DETECT_BASKET:
            Logger.info(self, "Detecting laundry basket from vision detections.")
            # Use detect_laundry_basket from vision_tasks.py
            status, basket_detection = self.subtask_manager.vision.detect_laundry_basket()

            if status == Status.EXECUTION_SUCCESS and basket_detection:
                Logger.info(self, f"Basket detected: {basket_detection.classname}")
                self.basket_detection = basket_detection
                self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_BEHIND_BASKET
            else:
                Logger.warn(self, "Could not detect laundry basket. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_BEHIND_BASKET:
            Logger.info(self, "Computing behind-basket approach pose.")
            pose = self._compute_behind_basket_pose(self.basket_detection)

            if pose is None:
                Logger.error(self, "Could not compute behind-basket pose. Retrying detection.")
                self.current_state = DoingLaundryTM.TaskStates.DETECT_BASKET
            else:
                approach_x, approach_y, yaw = pose
                self.subtask_manager.manipulation.move_to_position("nav_pose")
                self.subtask_manager.hri.say(
                    "Positioning to grab the basket from behind.", wait=False
                )
                status, error = self.subtask_manager.nav.navigate_to_pose(
                    approach_x, approach_y, yaw
                )

                if status == Status.EXECUTION_SUCCESS:
                    Logger.success(self, "Reached behind-basket position.")
                    self.current_state = DoingLaundryTM.TaskStates.PICK_LAUNDRY
                else:
                    Logger.error(self, f"Failed to reach behind-basket pose: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.PICK_LAUNDRY:
            Logger.info(self, "Opening gripper before picking.")
            self.subtask_manager.manipulation.open_gripper()

            Logger.info(self, "Attempting to pick the basket using pick_object.")
            # Use pick_object from manipulation
            # result = self.subtask_manager.manipulation.pick_object("laundry_basket")
            try:
                result = self.subtask_manager.manipulation.move_to_position("pick_basket_pose")
            except Exception as e:
                Logger.error(self, f"Error occurred while moving to pick position: {e}")
                result = self.subtask_manager.manipulation.pick_object("place_floor_right")

            if result == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Successfully picked the basket.")
                self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_TABLE
            else:
                Logger.error(self, "Failed to pick the basket.")
                self.current_state = DoingLaundryTM.TaskStates.END

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_TABLE:
            Logger.info(self, "Navigating to laundry table")
            status, error = self.navigate_to("laundry", "laundry_table")

            if status == Status.EXECUTION_SUCCESS:
                Logger.info(self, "Reached laundry table. Opening gripper.")
                self.subtask_manager.manipulation.open_gripper()
                self.current_state = DoingLaundryTM.TaskStates.UNLOAD_LAUNDRY
            else:
                Logger.error(self, f"Navigation failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.UNLOAD_LAUNDRY:
            Logger.info(self, "Starting to unload clothes from the basket.")

            # TODO: Add logic to pick clothes from the basket and put them on the table
            # START PICKING FROM BASKET AND PUTTING ON TABLE

            Logger.info(self, "Laundry unloading logic should go here.")
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
