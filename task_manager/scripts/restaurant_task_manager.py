#!/usr/bin/env python3

"""
Task Manager for Restaurant task of Robocup @Home 2026

Cycle: wait for a waving customer -> approach -> scan tables -> take the
caller's table orders -> bar (tell barman, pick) -> deliver each item ->
back to waiting for the next call. All scan results are stored in the MAP
frame at scan time (the robot moves between scan and use, so camera-frame
points would be mis-transformed later).
"""

import math
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point  # noqa: F401 (registers PointStamped transform)

from frida_constants.vision_constants import (
    RESTAURANT_TABLES_TOPIC,
    DETECTIONS_IMAGE_TOPIC,
    TRACKER_IMAGE_TOPIC,
    CUSTOMER,
)

from task_manager.utils.subtask_manager import SubtaskManager, Task
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status

ATTEMPT_LIMIT = 3

# ── Customer search ──
SEARCH_PAN_ANGLES = [0.0, -45.0, 45.0]  # camera sweep at each base heading (deg)
PAN_SETTLE_TIME = 1.0  # s to let the camera settle after a pan
BASE_ROTATION_DEG = 120.0  # in-place base rotation between sweeps (omni)
MAX_BASE_ROTATIONS = 2  # initial heading + 2 rotations ≈ full circle
SEARCH_STEP_SIZE = 0.8  # m forward per exploration step
MAX_SEARCH_STEPS = 4  # forward steps before returning to bar
CONFIRM_MATCH_RADIUS = 0.8  # m between two detections to accept a caller (persistence)

# ── Approach ──
CUSTOMER_STANDOFF = 2.0  # m from the caller for the table scan
TABLE_STANDOFF = 0.6  # m from the table point for order taking / delivery
DOCK_TABLE_OFFSET = 0.32  # m front offset for the perpendicular dock_table approach
CUSTOMER_CLOSE_DISTANCE = 3.5  # m — re-approach if the caller is farther than this
MAX_APPROACH_RETRIES = 3
MAX_SCAN_ATTEMPTS = 3  # table scans before falling back to the caller point

# ── Camera aiming ──
MAX_PAN_DEG = 90.0  # beyond this, rotate the omni base instead of the arm


class RestaurantTaskManager(Node):
    """Class to manage the restaurant task"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        WAIT_FOR_CALL = "WAIT_FOR_CALL"
        APPROACH_CUSTOMER = "APPROACH_CUSTOMER"
        SCAN_TABLES = "SCAN_TABLES"
        TAKE_ORDERS = "TAKE_ORDERS"
        NAVIGATE_TO_BAR = "NAVIGATE_TO_BAR"
        SAY_ORDER_TO_BARMAN = "SAY_ORDER_TO_BARMAN"
        DELIVER_ORDER = "DELIVER_ORDER"

    def __init__(self):
        """Initialize the node"""
        super().__init__("restaurant_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.RESTAURANT, mock_areas=[])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Reject callers behind the bar's initial facing direction (public /
        # recording zone is usually behind the start pose). Off by default —
        # enable at the venue once the layout is known.
        self.declare_parameter("restrict_search_sector", False)
        # Final heading when returning to the bar, in degrees CCW relative to
        # the START orientation: 180 = turn around (default), 90 = face left,
        # -90 = face right, 0 = face as started. Set per venue depending on
        # where the barman stands relative to the start pose.
        self.declare_parameter("bar_return_yaw_deg", 180.0)

        self.running_task = True

        # Bar (origin) pose saved at START via nav get_current_pose
        self.bar_pose = None

        # Per-cycle state (reset by _reset_cycle)
        self.target_person_point = None  # caller, map frame
        self.serve_table = None  # {'table_point', 'customers', 'orders', 'approach_pose'}
        self.current_customer_index = 0
        self.current_delivery_item_index = 0
        self.approach_attempts = 0
        self.scan_attempts = 0
        self.base_rotations = 0
        self.search_step = 0

        self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_BUTTON

        self.get_logger().info("RestaurantTaskManager has started.")

    # ────────────────────────── helpers ──────────────────────────

    def timeout(self, timeout: float = 2.0):
        """Wait while keeping the node's callbacks alive."""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _reset_cycle(self):
        """Clear per-cycle state so the next WAIT_FOR_CALL starts clean."""
        self.target_person_point = None
        self.serve_table = None
        self.current_customer_index = 0
        self.current_delivery_item_index = 0
        self.approach_attempts = 0
        self.scan_attempts = 0
        self.base_rotations = 0
        self.search_step = 0

    def _to_map(self, point_stamped):
        """Transform a PointStamped to the map frame using the CURRENT TF.
        Must be called while the robot is still at the pose where the point
        was detected (detections are stamped with time 0 = latest)."""
        if point_stamped is None or point_stamped.header.frame_id == "":
            return None
        if point_stamped.header.frame_id == "map":
            return point_stamped
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                point_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return do_transform_point(point_stamped, t)
        except TransformException as e:
            Logger.warn(self, f"TF {point_stamped.header.frame_id}->map failed: {e}")
            return None

    def _pan_angle_to(self, point_stamped):
        """Return the pan_to angle (degrees) to face a PointStamped, computed in base_link."""
        if point_stamped is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link", point_stamped.header.frame_id, rclpy.time.Time()
            )
            bl_point = do_transform_point(point_stamped, t)
        except TransformException as e:
            Logger.warn(self, f"TF to base_link failed for pan: {e}")
            return None

        # base_link: x forward, y left. pan_to treats +deg as customer-to-right,
        # so negate atan2(y, x). Result is inherently within [-180, 180].
        return -math.degrees(math.atan2(bl_point.point.y, bl_point.point.x))

    def _rotate_base(self, degrees: float):
        """Rotate the omni base in place by `degrees` (map-frame yaw change)."""
        status, cur = self.subtask_manager.nav.get_current_pose()
        if status != Status.EXECUTION_SUCCESS or cur is None:
            return Status.EXECUTION_ERROR
        yaw = self.subtask_manager.nav._yaw_from_quaternion(cur.pose.orientation)
        cur.pose.orientation = self.subtask_manager.nav._yaw_to_quaternion(
            yaw + math.radians(degrees)
        )
        status, _ = self.subtask_manager.nav.move_to_pose(cur)
        return status

    def look_at(self, map_point):
        """Aim the wrist camera at a map-frame point regardless of how the omni
        base ended up oriented: pan the arm when the bearing is within reach,
        otherwise rotate the base toward the point first, then trim with a pan."""
        angle = self._pan_angle_to(map_point)
        if angle is None:
            Logger.warn(self, "look_at: could not resolve bearing, facing forward.")
            self.subtask_manager.manipulation.pan_to(0.0)
            return Status.EXECUTION_ERROR

        if abs(angle) > MAX_PAN_DEG:
            Logger.info(self, f"look_at: bearing {angle:.0f} deg beyond pan range, rotating base")
            status, cur = self.subtask_manager.nav.get_current_pose()
            if status == Status.EXECUTION_SUCCESS and cur is not None:
                yaw = math.atan2(
                    map_point.point.y - cur.pose.position.y,
                    map_point.point.x - cur.pose.position.x,
                )
                cur.pose.orientation = self.subtask_manager.nav._yaw_to_quaternion(yaw)
                self.subtask_manager.nav.move_to_pose(cur)
            angle = self._pan_angle_to(map_point)
            if angle is None:
                angle = 0.0

        self.subtask_manager.manipulation.pan_to(max(-MAX_PAN_DEG, min(MAX_PAN_DEG, angle)))
        return Status.EXECUTION_SUCCESS

    def _in_search_sector(self, map_point) -> bool:
        """When restrict_search_sector is on, accept only points in the forward
        half-plane of the bar's initial facing (public zone is behind)."""
        if not self.get_parameter("restrict_search_sector").value:
            return True
        if self.bar_pose is None or map_point is None:
            return True
        yaw = self.subtask_manager.nav._yaw_from_quaternion(self.bar_pose.pose.orientation)
        dx = map_point.point.x - self.bar_pose.pose.position.x
        dy = map_point.point.y - self.bar_pose.pose.position.y
        if dx * math.cos(yaw) + dy * math.sin(yaw) < 0.0:
            Logger.info(self, "Detection behind the bar sector — ignoring (public zone).")
            return False
        return True

    def _sweep_for_customer(self):
        """Pan the camera over SEARCH_PAN_ANGLES looking for a waving customer.
        A candidate must be seen twice (persistence) within CONFIRM_MATCH_RADIUS
        to filter transient arm raises (e.g. public lifting phones). Returns the
        confirmed caller as a map-frame PointStamped, or None."""
        for pan in SEARCH_PAN_ANGLES:
            self.subtask_manager.manipulation.pan_to(pan)
            self.timeout(PAN_SETTLE_TIME)

            status, person_point = self.subtask_manager.vision.get_customer()
            if status != Status.EXECUTION_SUCCESS or person_point.header.frame_id == "":
                continue
            first_map = self._to_map(person_point)
            if first_map is None or not self._in_search_sector(first_map):
                continue

            # Persistence check: a real caller keeps waving; a phone lift doesn't.
            self.timeout(1.0)
            status, second_point = self.subtask_manager.vision.get_customer()
            if status != Status.EXECUTION_SUCCESS or second_point.header.frame_id == "":
                Logger.info(self, "Candidate not re-detected — ignoring transient wave.")
                continue
            second_map = self._to_map(second_point)
            if second_map is None:
                continue
            dist = math.hypot(
                second_map.point.x - first_map.point.x,
                second_map.point.y - first_map.point.y,
            )
            if dist > CONFIRM_MATCH_RADIUS:
                Logger.info(self, f"Candidate moved {dist:.1f} m between checks — ignoring.")
                continue

            self.subtask_manager.manipulation.pan_to(0.0)
            return second_map

        self.subtask_manager.manipulation.pan_to(0.0)
        return None

    # ────────────────────────── pick / place ──────────────────────────

    def deus_pick(self, object_name):
        """Fallback: ask the barman to place the object in the gripper."""
        Logger.warn(self, f"Requesting human assistance for {object_name}.")
        self.subtask_manager.manipulation.open_gripper()
        self.subtask_manager.hri.say(
            f"I am having trouble picking the {object_name}. Please place it in my gripper and say yes when done."
        )
        _, confirmation = self.subtask_manager.hri.confirm(
            "Have you placed the object in my gripper?",
            use_keyword=True,
            retries=3,
            wait_between_retries=5,
        )
        if confirmation == "yes":
            self.subtask_manager.manipulation.close_gripper()
            self.subtask_manager.hri.say("Thank you. I have received the object.")
            return Status.EXECUTION_SUCCESS
        return Status.EXECUTION_ERROR

    def pick_object(self, object_name):
        """Detect objects, find closest match, and pick."""
        self.subtask_manager.hri.say(f"I will pick the {object_name}.", wait=False)
        detections = []
        for attempt in range(ATTEMPT_LIMIT):
            status, detections = self.subtask_manager.vision.detect_objects()
            if status == Status.EXECUTION_SUCCESS and len(detections) > 0:
                break
            Logger.warn(self, f"No objects detected, attempt {attempt + 1}/{ATTEMPT_LIMIT}")

        if not detections:
            self.subtask_manager.hri.say(f"I could not see the {object_name}.")
            return self.deus_pick(object_name)

        labels = self.subtask_manager.vision.get_labels(detections)
        Logger.info(self, f"Detected labels: {labels}")
        status, closest = self.subtask_manager.hri.find_closest(labels, object_name)
        if status != Status.EXECUTION_SUCCESS or not closest.results:
            Logger.warn(self, f"Could not match '{object_name}' in detected labels: {labels}")
            return self.deus_pick(object_name)

        matched_label = closest.results[0]
        Logger.info(self, f"Matched '{object_name}' to detection label: '{matched_label}'")
        for attempt in range(ATTEMPT_LIMIT):
            Logger.info(self, f"Pick attempt {attempt + 1}/{ATTEMPT_LIMIT} for '{matched_label}'")
            status = self.subtask_manager.manipulation.pick_object(matched_label)
            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(f"I have picked the {object_name}.")
                return Status.EXECUTION_SUCCESS
            Logger.warn(self, "Pick failed, retrying...")

        return self.deus_pick(object_name)

    def place_object(self):
        """Attempt to place object with retries, fallback to deus ex machina."""
        for attempt in range(ATTEMPT_LIMIT):
            Logger.info(self, f"Place attempt {attempt + 1}/{ATTEMPT_LIMIT}")
            status = self.subtask_manager.manipulation.place()
            if status == Status.EXECUTION_SUCCESS:
                return Status.EXECUTION_SUCCESS
            self.timeout(1)

        self.subtask_manager.hri.say(
            "I couldn't place the object. Please grab it from my gripper and say yes when done."
        )
        _, confirmation = self.subtask_manager.hri.confirm(
            "Have you grabbed the object from my gripper?",
            use_keyword=True,
            retries=3,
            wait_between_retries=5,
        )
        if confirmation == "yes":
            self.subtask_manager.manipulation.open_gripper()
            self.subtask_manager.hri.say("Thank you.")
            return Status.EXECUTION_SUCCESS

        return Status.EXECUTION_ERROR

    def _return_to_bar(self):
        """Go back to the start pose, ending at the venue-configured heading
        (bar_return_yaw_deg param) so the robot faces the barman's side."""
        return self.subtask_manager.nav.return_to_origin(
            yaw_offset_deg=float(self.get_parameter("bar_return_yaw_deg").value)
        )

    def _dock_to_table(self):
        """Perpendicular-dock to the table/bar in front: tuck the arm first
        (nav_pose keeps it inside the footprint at the 0.32 m offset)."""
        self.subtask_manager.manipulation.move_to_position("nav_pose", velocity=0.5)
        status, _ = self.subtask_manager.nav.dock_table(offset=DOCK_TABLE_OFFSET)
        return status

    def _navigate_to_serve_table(self):
        """Go back to the caller's table: prefer the exact pose we reached when
        taking orders, else approach the map-frame table point."""
        table = self.serve_table
        if table.get("approach_pose") is not None:
            status, _ = self.subtask_manager.nav.move_to_pose(table["approach_pose"])
            if status == Status.EXECUTION_SUCCESS:
                return status
        status, _ = self.subtask_manager.nav.approach_point(
            table["table_point"], standoff=TABLE_STANDOFF
        )
        return status

    # ────────────────────────── main loop ──────────────────────────

    def run(self):
        """Running main loop"""

        if self.current_state == RestaurantTaskManager.TaskStates.WAIT_FOR_BUTTON:
            self.subtask_manager.manipulation.move_to_position("carry_pose", velocity=0.5)
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.")
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, restaurant task will begin now")
            self.current_state = RestaurantTaskManager.TaskStates.START

        if self.current_state == RestaurantTaskManager.TaskStates.START:
            Logger.state(self, "Starting restaurant task...")
            self.subtask_manager.manipulation.move_to_position("carry_pose", velocity=0.5)
            self.subtask_manager.hri.say(
                "Hello everyone, I am Frida, your waiter today. "
                "If you would like to order, please raise your arm and I will come to your table."
            )
            while self.bar_pose is None:
                status, self.bar_pose = self.subtask_manager.nav.get_current_pose()
                if status != Status.EXECUTION_SUCCESS or self.bar_pose is None:
                    Logger.warn(self, "TF not ready, retrying bar pose...")
                    rclpy.spin_once(self, timeout_sec=0.5)
            Logger.info(self, "Bar position saved.")
            self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL

        if self.current_state == RestaurantTaskManager.TaskStates.WAIT_FOR_CALL:
            Logger.state(
                self,
                f"Searching for waving customer (rotation {self.base_rotations}/"
                f"{MAX_BASE_ROTATIONS}, step {self.search_step}/{MAX_SEARCH_STEPS})...",
            )
            # Live pose-detection feed on the screen while searching
            self.subtask_manager.hri.publish_display_topic(TRACKER_IMAGE_TOPIC)

            caller = self._sweep_for_customer()

            if caller is not None:
                Logger.success(self, "Customer detected calling!")
                # Show the confirmed caller crop on the display (partial points
                # if the table is never reached, per rulebook).
                self.subtask_manager.hri.publish_display_topic(CUSTOMER)
                self.subtask_manager.hri.say(
                    "I see you! I am coming to take your order. Please keep your hand raised "
                    "You can check my screen to confirm I detected you."
                )
                self.target_person_point = caller
                self.base_rotations = 0
                self.search_step = 0
                self.current_state = RestaurantTaskManager.TaskStates.APPROACH_CUSTOMER

            elif self.base_rotations < MAX_BASE_ROTATIONS:
                # Cover the full circle with in-place rotations before moving:
                # cheap for the omni base and safe in an unknown restaurant.
                self.base_rotations += 1
                status = self._rotate_base(BASE_ROTATION_DEG)
                if status != Status.EXECUTION_SUCCESS:
                    Logger.warn(self, "In-place rotation failed, continuing sweep.")

            elif self.search_step < MAX_SEARCH_STEPS:
                self.base_rotations = 0
                self.search_step += 1
                self.subtask_manager.hri.say("I will move to look for customers.", wait=False)
                status, _ = self.subtask_manager.nav.explore_zone(SEARCH_STEP_SIZE)
                if status != Status.EXECUTION_SUCCESS:
                    Logger.warn(self, "Exploration step blocked, will rotate and retry.")
            else:
                Logger.info(self, "Full area scanned, no customer. Returning to bar.")
                self.subtask_manager.hri.say(
                    "I did not find any customers calling. I will return to the bar and wait."
                )
                self.base_rotations = 0
                self.search_step = 0
                self._return_to_bar()
                return

        if self.current_state == RestaurantTaskManager.TaskStates.APPROACH_CUSTOMER:
            if self.target_person_point is None:
                self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL
                return

            Logger.state(
                self,
                f"Navigating to customer (attempt {self.approach_attempts + 1}/{MAX_APPROACH_RETRIES})...",
            )
            # approach_point picks a costmap-free goal near the caller and
            # leaves the base FACING them (handles the omni base ending up
            # rotated after the goal).
            self.subtask_manager.manipulation.move_to_position("carry_pose", velocity=0.5)
            status, _ = self.subtask_manager.nav.approach_point(
                self.target_person_point, standoff=CUSTOMER_STANDOFF
            )

            if status == Status.EXECUTION_SUCCESS:
                # Re-detect customer to check if we're actually close enough
                Logger.info(self, "Arrived at goal. Re-checking customer distance...")
                self.look_at(self.target_person_point)
                re_status, re_point = self.subtask_manager.vision.get_customer()

                if re_status == Status.EXECUTION_SUCCESS and re_point.header.frame_id != "":
                    camera_dist = math.sqrt(
                        re_point.point.x**2 + re_point.point.y**2 + re_point.point.z**2
                    )
                    Logger.info(self, f"Customer re-detected at {camera_dist:.2f}m from camera")

                    if (
                        camera_dist > CUSTOMER_CLOSE_DISTANCE
                        and self.approach_attempts < MAX_APPROACH_RETRIES
                    ):
                        # Still too far — update target (in map frame, NOW,
                        # before moving again) and approach once more.
                        re_map = self._to_map(re_point)
                        if re_map is not None:
                            self.approach_attempts += 1
                            self.target_person_point = re_map
                            Logger.info(
                                self,
                                f"Customer still {camera_dist:.2f}m away, approaching again...",
                            )
                            self.subtask_manager.manipulation.pan_to(0.0)
                            return  # re-enter APPROACH_CUSTOMER on next loop
                    Logger.success(self, "Close enough for table detection.")
                else:
                    Logger.info(self, "Customer not re-detected, proceeding to table scan.")

                self.subtask_manager.manipulation.pan_to(0.0)
                self.approach_attempts = 0
                self.current_state = RestaurantTaskManager.TaskStates.SCAN_TABLES
            else:
                Logger.warn(self, "Navigation to customer failed. Returning to WAIT_FOR_CALL.")
                self.approach_attempts = 0
                self.target_person_point = None
                self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL

        if self.current_state == RestaurantTaskManager.TaskStates.SCAN_TABLES:
            Logger.state(self, "Performing full table scan...")
            self.subtask_manager.manipulation.move_to_position("carry_pose", velocity=0.5)
            status, customer_tables = self.subtask_manager.vision.customer_tables()
            self.subtask_manager.hri.publish_display_topic(RESTAURANT_TABLES_TOPIC)

            if status != Status.EXECUTION_SUCCESS or not customer_tables:
                self.scan_attempts += 1
                if self.scan_attempts < MAX_SCAN_ATTEMPTS:
                    Logger.warn(
                        self,
                        f"No tables detected (attempt {self.scan_attempts}/{MAX_SCAN_ATTEMPTS}), retrying...",
                    )
                    self.timeout(2)
                    return
                # Fallback: serve the confirmed caller directly — their point
                # doubles as the table point (partial points beat none).
                Logger.warn(self, "Table scan failed. Falling back to the caller position.")
                self.serve_table = {
                    "table_point": self.target_person_point,
                    "customers": [self.target_person_point],
                    "orders": [],
                    "approach_pose": None,
                }
                self.current_customer_index = 0
                self.current_state = RestaurantTaskManager.TaskStates.TAKE_ORDERS
                return

            # Everything to MAP frame NOW, while still at the scan pose —
            # later transforms would use the wrong (moved) camera pose.
            candidate_tables = []
            for table_msg in customer_tables:
                if len(table_msg.people.list) == 0:
                    continue
                table_map = self._to_map(table_msg.table_point)
                if table_map is None:
                    Logger.warn(self, "Skipping table with untransformable point.")
                    continue
                customer_points = []
                for person in table_msg.people.list:
                    p_map = self._to_map(person.point3d)
                    if p_map is not None:
                        customer_points.append(p_map)
                if not customer_points:
                    continue
                candidate_tables.append(
                    {
                        "table_point": table_map,
                        "customers": customer_points,
                        "orders": [],
                        "approach_pose": None,
                    }
                )

            if not candidate_tables:
                Logger.warn(self, "No customers mapped. Returning to WAIT_FOR_CALL.")
                self._reset_cycle()
                self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL
                return

            # Serve the CALLER's table: the one closest to the confirmed caller.
            def dist_to_caller(t):
                if self.target_person_point is None:
                    return 0.0
                return math.hypot(
                    t["table_point"].point.x - self.target_person_point.point.x,
                    t["table_point"].point.y - self.target_person_point.point.y,
                )

            self.serve_table = min(candidate_tables, key=dist_to_caller)
            total = len(self.serve_table["customers"])
            self.subtask_manager.hri.say(
                f"I found the table that called me, with {total} "
                f"customer{'s' if total != 1 else ''}. "
                "Please look at my screen to see the detections. I will take your orders now."
            )
            self.current_customer_index = 0
            self.current_state = RestaurantTaskManager.TaskStates.TAKE_ORDERS

        if self.current_state == RestaurantTaskManager.TaskStates.TAKE_ORDERS:
            table = self.serve_table

            if self.current_customer_index < len(table["customers"]):
                Logger.state(
                    self,
                    f"Taking order from customer {self.current_customer_index + 1}/"
                    f"{len(table['customers'])}",
                )

                # Navigate to the table on the first customer only
                if self.current_customer_index == 0:
                    self.subtask_manager.hri.say("Navigating to your table.")
                    self.subtask_manager.manipulation.move_to_position("carry_pose", velocity=0.5)
                    status, _ = self.subtask_manager.nav.approach_point(
                        table["table_point"], standoff=TABLE_STANDOFF
                    )
                    if status == Status.EXECUTION_SUCCESS:
                        # Save the actual arrived pose for delivery trips later
                        _, arrived_pose = self.subtask_manager.nav.get_current_pose()
                        if arrived_pose is not None:
                            table["approach_pose"] = arrived_pose
                    else:
                        Logger.warn(self, "Table approach failed, taking orders from here.")

                # Face the customer — bearing computed NOW from the map point,
                # valid no matter how the omni base ended up oriented.
                self.look_at(table["customers"][self.current_customer_index])

                order_received = False
                for order_attempt in range(ATTEMPT_LIMIT):
                    Logger.info(self, f"Order attempt {order_attempt + 1}/{ATTEMPT_LIMIT}")
                    status, orders = self.subtask_manager.hri.take_order(retries=3)
                    if status == Status.EXECUTION_SUCCESS and orders:
                        for order in orders:
                            table["orders"].append(order)
                            Logger.success(self, f"Order received: {order}")
                        order_received = True
                        break
                    else:
                        Logger.warn(self, f"Order attempt {order_attempt + 1} failed.")
                        if order_attempt < ATTEMPT_LIMIT - 1:
                            self.subtask_manager.hri.say(
                                "Sorry, I didn't catch that. Could you please repeat your order?"
                            )
                if not order_received:
                    Logger.warn(self, "Failed to get order after all attempts.")
                    self.subtask_manager.hri.say(
                        "Sorry, I couldn't get your order. I'll move to the next customer."
                    )

                self.current_customer_index += 1

            elif table["orders"]:
                Logger.info(self, "All orders taken at the caller table. Going to bar.")
                self.subtask_manager.hri.say(
                    "Thank you. I will bring your order as soon as possible."
                )
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR
            else:
                Logger.warn(self, "No orders taken at this table. Waiting for the next call.")
                self.subtask_manager.hri.say(
                    "I could not take any order here. Please wave again when you are ready."
                )
                self._reset_cycle()
                self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL

        if self.current_state == RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR:
            Logger.state(self, "Navigating to bar...")
            self.subtask_manager.manipulation.move_to_position("carry_pose", velocity=0.5)
            self._return_to_bar()
            # Perpendicular final approach to the bar counter (lidar/cloud based);
            # nav_central auto-undocks before the next nav goal.
            status = self._dock_to_table()
            if status != Status.EXECUTION_SUCCESS:
                Logger.warn(self, "Bar docking failed, staying at the origin pose.")
            self.current_state = RestaurantTaskManager.TaskStates.SAY_ORDER_TO_BARMAN

        if self.current_state == RestaurantTaskManager.TaskStates.SAY_ORDER_TO_BARMAN:
            orders = self.serve_table["orders"]
            Logger.state(self, f"Communicating order to barman: {orders}")
            self.subtask_manager.hri.say(
                f"Hello barman. I have {len(orders)} item{'s' if len(orders) != 1 else ''} "
                f"for one table: {', '.join(orders)}. Please help me prepare them."
            )
            self.current_delivery_item_index = 0
            self.current_state = RestaurantTaskManager.TaskStates.DELIVER_ORDER

        if self.current_state == RestaurantTaskManager.TaskStates.DELIVER_ORDER:
            self.subtask_manager.hri.publish_display_topic(DETECTIONS_IMAGE_TOPIC)
            table = self.serve_table
            orders = table["orders"]

            if self.current_delivery_item_index < len(orders):
                item = orders[self.current_delivery_item_index]
                Logger.state(
                    self,
                    f"Picking item {self.current_delivery_item_index + 1}/{len(orders)}: {item}",
                )

                self.subtask_manager.manipulation.move_to_position("table_stare", velocity=0.5)
                status = self.pick_object(item)

                if status == Status.EXECUTION_SUCCESS:
                    Logger.success(self, f"Picked {item}. Delivering to the table...")

                    self.subtask_manager.manipulation.move_to_position("carry_pose", velocity=0.5)
                    nav_status = self._navigate_to_serve_table()
                    if nav_status != Status.EXECUTION_SUCCESS:
                        Logger.warn(self, "Return to table failed, delivering from here.")
                    # Final perpendicular docking so the place happens at a safe,
                    # repeatable distance from the tabletop.
                    dock_status = self._dock_to_table()
                    if dock_status != Status.EXECUTION_SUCCESS:
                        Logger.warn(self, "Table docking failed, placing from approach pose.")
                    self.subtask_manager.hri.say(f"Here is your {item}.")
                    self.subtask_manager.manipulation.move_to_position("table_stare", velocity=0.5)
                    status = self.place_object()

                    if status == Status.EXECUTION_SUCCESS:
                        Logger.success(self, f"Delivered {item}")
                    else:
                        Logger.error(self, f"Failed to place {item}")
                    self.current_delivery_item_index += 1

                    # Return to bar for the next item
                    if self.current_delivery_item_index < len(orders):
                        self.subtask_manager.hri.say("I will get your next item.", wait=False)
                        self.subtask_manager.manipulation.move_to_position(
                            "carry_pose", velocity=0.5
                        )
                        self._return_to_bar()
                        self._dock_to_table()
                else:
                    Logger.error(self, f"Failed to pick {item}")
                    self.current_delivery_item_index += 1

            else:
                Logger.success(self, "Order fully delivered. Waiting for the next call.")
                self.subtask_manager.hri.say("Enjoy your meal!")
                self.subtask_manager.manipulation.move_to_position("carry_pose", velocity=0.5)
                self._reset_cycle()
                self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL


def main(args=None):
    rclpy.init(args=args)
    node = RestaurantTaskManager()

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
