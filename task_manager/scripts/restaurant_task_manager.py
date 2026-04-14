#!/usr/bin/env python3

"""
Task Manager for Restaurant task of Robocup @Home 2026
"""

from frida_constants.vision_constants import (
    RESTAURANT_TABLES_TOPIC,
    DETECTIONS_IMAGE_TOPIC,
)
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped

from task_manager.utils.subtask_manager import SubtaskManager, Task

from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
import time

ATTEMPT_LIMIT = 3

# Progressive customer search parameters
SEARCH_STEP_SIZE = 1.0  # meters between each search position
MAX_SEARCH_STEPS = 5  # total steps before returning to bar (~5 m coverage)


class RestaurantTaskManager(Node):
    """Class to manage the restaurant task"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        WAIT_FOR_CALL = "WAIT_FOR_CALL"
        MOVE_TO_TABLES_AREA = "MOVE_TO_TABLES_AREA"
        DETECT_CUSTOMERS = "DETECT_CUSTOMERS"
        NAVIGATE_TO_TABLE = "NAVIGATE_TO_TABLE"
        TAKE_ORDERS = "TAKE_ORDERS"
        NAVIGATE_TO_BAR = "NAVIGATE_TO_BAR"
        SAY_ORDER_TO_BARMAN = "SAY_ORDER_TO_BARMAN"
        PREPARE_DELIVERY = "PREPARE_DELIVERY"
        DELIVER_TO_TABLE = "DELIVER_TO_TABLE"
        END = "END"

    def __init__(self):
        """Initialize the node"""
        super().__init__("restaurant_task_manager")
        self.subtask_manager = SubtaskManager(
            self, task=Task.RESTAURANT, mock_areas=["manipulation"]
        )

        self.running_task = True

        self.original_goal_pub = self.create_publisher(
            PoseStamped, "/adaptive_nav/original_goal", 10
        )

        # Table data structure: {table_id: {'customer_points': [PointStamped], 'customer_angles': [float], 'orders': [str], 'coordinates': PointStamped, 'num_customers': int}}
        self.tables = {}
        self.tables_sorted_by_customers = []  # descending
        self.current_table_idx = 0

        # Tracking current operations
        self.current_customer_index = 0
        self.current_delivery_item_index = 0
        self.pick_attempts = 0

        # Pan angles per-customer
        self.pan_angles = []

        # Target customer detected during WAIT_FOR_CALL
        self.target_person_point = None

        # Progressive search state: index of the current search position (0 = bar)
        self.search_step = 0

        # Bar position (saved at START)
        self.bar_pose = None

        self._orders_given = False

        self.current_state = RestaurantTaskManager.TaskStates.START

        self.get_logger().info("RestaurantTaskManager has started.")

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def sort_tables_by_customers(self):
        """Sort tables by number of customers (descending - greedy strategy)."""
        self.tables_sorted_by_customers = sorted(
            self.tables.keys(), key=lambda tid: self.tables[tid]["num_customers"], reverse=True
        )
        Logger.info(
            self,
            f"Tables sorted by customers: {[(tid, self.tables[tid]['num_customers']) for tid in self.tables_sorted_by_customers]}",
        )

    def deus_pick(self, object_name):
        """Fallback: ask human to place object in gripper."""
        Logger.warn(self, f"Requesting human assistance for {object_name}.")
        self.subtask_manager.manipulation.open_gripper()
        self.subtask_manager.hri.say(
            f"I am having trouble picking the {object_name}. Please place it in my gripper and say yes when done."
        )
        status, confirmation = self.subtask_manager.hri.confirm(
            "Have you placed the object in my gripper?",
            use_hotwords=True,
            retries=3,
            wait_between_retries=5,
        )
        if confirmation == "yes":
            self.subtask_manager.manipulation.close_gripper()
            self.subtask_manager.hri.say("Thank you. I have received the object.")
            return (Status.EXECUTION_SUCCESS, "")
        return (Status.EXECUTION_ERROR, "Human assistance failed")

    def pick_object(self, object_name):
        """Detect objects, find closest match, and pick. Similar to GPSR flow."""
        self.subtask_manager.hri.say(f"I will pick the {object_name}.", wait=False)

        # Detect objects with retries
        detections = []
        for attempt in range(ATTEMPT_LIMIT):
            status, detections = self.subtask_manager.vision.detect_objects()
            # Handle tuple return from vision
            if isinstance(status, tuple):
                status = status[0]

            if status == Status.EXECUTION_SUCCESS and len(detections) > 0:
                break
            Logger.warn(self, f"No objects detected, attempt {attempt + 1}/{ATTEMPT_LIMIT}")

        if not detections:
            self.subtask_manager.hri.say(f"I could not see the {object_name}.")
            return self.deus_pick(object_name)

        # Find closest match to requested object
        labels = self.subtask_manager.vision.get_labels(detections)
        Logger.info(self, f"Detected labels: {labels}")
        status, closest = self.subtask_manager.hri.find_closest(labels, object_name)
        if isinstance(status, tuple):
            status = status[0]

        if status != Status.EXECUTION_SUCCESS or not closest.results:
            Logger.warn(self, f"Could not match '{object_name}' in detected labels: {labels}")
            return self.deus_pick(object_name)

        matched_label = closest.results[0]
        Logger.info(self, f"Matched '{object_name}' to detection label: '{matched_label}'")

        # Pick with retries
        for attempt in range(ATTEMPT_LIMIT):
            Logger.info(self, f"Pick attempt {attempt + 1}/{ATTEMPT_LIMIT} for '{matched_label}'")
            status = self.subtask_manager.manipulation.pick_object(matched_label)
            if isinstance(status, tuple):
                status = status[0]

            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(f"I have picked the {object_name}.")
                return (Status.EXECUTION_SUCCESS, "")
            Logger.warn(self, "Pick failed, retrying...")

        return self.deus_pick(object_name)

    def place_object(self):
        """Attempt to place object with retries, fallback to deus ex machina."""
        for attempt in range(ATTEMPT_LIMIT):
            Logger.info(self, f"Place attempt {attempt + 1}/{ATTEMPT_LIMIT}")
            status = self.subtask_manager.manipulation.place()
            if isinstance(status, tuple):
                status = status[0]

            if status == Status.EXECUTION_SUCCESS:
                return (Status.EXECUTION_SUCCESS, "")
            self.timeout(1)

        # Failed - ask human to take object
        self.subtask_manager.hri.say(
            "I couldn't place the object. Please grab it from my gripper and say yes when done."
        )
        status, confirmation = self.subtask_manager.hri.confirm(
            "Have you grabbed the object from my gripper?",
            use_hotwords=True,
            retries=3,
            wait_between_retries=5,
        )
        if confirmation == "yes":
            self.subtask_manager.manipulation.open_gripper()
            self.subtask_manager.hri.say("Thank you.")
            return (Status.EXECUTION_SUCCESS, "")

        return (Status.EXECUTION_ERROR, "Human assistance failed")

    def _compute_search_waypoint(self, step: int) -> PointStamped:
        """Return a PointStamped `step * SEARCH_STEP_SIZE` m ahead of the bar along the robot's initial heading."""
        if self.bar_pose is None:
            return None
        q = self.bar_pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        wp = PointStamped()
        wp.header.frame_id = "map"
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.point.x = self.bar_pose.pose.position.x + step * SEARCH_STEP_SIZE * math.cos(yaw)
        wp.point.y = self.bar_pose.pose.position.y + step * SEARCH_STEP_SIZE * math.sin(yaw)
        wp.point.z = 0.0
        return wp

    def run(self):
        """Running main loop"""
        if self.current_state == RestaurantTaskManager.TaskStates.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.")
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, restaurant task will begin now")
            self.current_state = RestaurantTaskManager.TaskStates.START

        if self.current_state == RestaurantTaskManager.TaskStates.START:
            Logger.state(self, "Starting restaurant task...")
            self.subtask_manager.hri.say("Starting the restaurant challenge. I am ready to help.")

            # Save current position as the Bar (Rule 5.5: Robot starts next to Kitchen-bar)
            Logger.info(self, "Saving current position as the Bar station.")
            self.bar_pose = self.subtask_manager.nav.get_current_pose()

            self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL

        if self.current_state == RestaurantTaskManager.TaskStates.WAIT_FOR_CALL:
            Logger.state(
                self,
                f"Searching for waving customer (step {self.search_step}/{MAX_SEARCH_STEPS})...",
            )
            status, person_point = self.subtask_manager.vision.get_customer()

            if status == Status.EXECUTION_SUCCESS and person_point.header.frame_id != "":
                Logger.success(self, "Customer detected calling!")
                self.subtask_manager.hri.say("I see you! I am coming to take your order.")
                self.target_person_point = person_point
                self.search_step = 0  # reset for future searches
                self.current_state = RestaurantTaskManager.TaskStates.MOVE_TO_TABLES_AREA

            elif self.search_step < MAX_SEARCH_STEPS:
                # Advance one step toward the dining area and check again next cycle
                self.search_step += 1
                waypoint = self._compute_search_waypoint(self.search_step)
                if waypoint is not None:
                    Logger.info(
                        self,
                        f"No customer detected. Moving to search position {self.search_step} "
                        f"({waypoint.point.x:.2f}, {waypoint.point.y:.2f}).",
                    )
                    self.subtask_manager.nav.resume_nav()
                    self.subtask_manager.nav.move_to_point(waypoint)
                    self.subtask_manager.nav.pause_nav()
                else:
                    self.timeout(1)
                return  # vision check happens at new position on next run() call

            else:
                # Scanned the full corridor — return to bar and restart
                Logger.info(self, "Full area scanned with no customer. Returning to bar.")
                self.subtask_manager.hri.say(
                    "I did not find any customers calling. I will return to the bar and wait."
                )
                self.search_step = 0
                if self.bar_pose is not None:
                    self.subtask_manager.nav.resume_nav()
                    self.subtask_manager.nav.move_to_pose(self.bar_pose)
                    self.subtask_manager.nav.pause_nav()
                self.timeout(2)
                return

        if self.current_state == RestaurantTaskManager.TaskStates.MOVE_TO_TABLES_AREA:
            if self.target_person_point is not None:
                Logger.state(self, "Navigating directly to detected customer position...")
                self.subtask_manager.nav.resume_nav()
                self.subtask_manager.nav.change_bt("adaptive")

                # Publish goal for radial sampling
                goal_msg = PoseStamped()
                goal_msg.header = self.target_person_point.header
                goal_msg.pose.position = self.target_person_point.point
                goal_msg.pose.orientation.w = 1.0
                self.original_goal_pub.publish(goal_msg)

                nav_result = self.subtask_manager.nav.move_to_point(self.target_person_point)
                status = nav_result[0] if isinstance(nav_result, tuple) else nav_result

                self.subtask_manager.nav.pause_nav()

                if status == Status.EXECUTION_SUCCESS:
                    Logger.success(self, "Arrived near table for detection.")
                    self.current_state = RestaurantTaskManager.TaskStates.DETECT_CUSTOMERS
            else:
                self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL
                return

        if self.current_state == RestaurantTaskManager.TaskStates.DETECT_CUSTOMERS:
            Logger.state(self, "Performing full table scan...")
            self.subtask_manager.manipulation.move_to_position("front_stare", velocity=0.5)
            # Get table groups and positions directly from vision task
            vision_result = self.subtask_manager.vision.customer_tables()
            status = vision_result[0] if isinstance(vision_result, tuple) else vision_result
            customer_tables = vision_result[1] if isinstance(vision_result, tuple) else []

            self.subtask_manager.hri.publish_display_topic(RESTAURANT_TABLES_TOPIC)
            if status != Status.EXECUTION_SUCCESS or not customer_tables:
                Logger.warn(self, "Could not confirm customers at this point. Retrying scan...")
                self.timeout(2)
                return

            Logger.info(self, f"Detected {len(customer_tables)} table(s)")
            table_idx = 0
            for table_msg in customer_tables:
                if len(table_msg.people.list) == 0:
                    continue
                customer_points = []
                customer_angles = []
                for person in table_msg.people.list:
                    customer_points.append(person.point3d)
                    customer_angles.append(person.angle)
                self.tables[table_idx] = {
                    "customer_points": customer_points,
                    "customer_angles": customer_angles,
                    "orders": [],
                    "coordinates": table_msg.table_point,
                    "num_customers": len(table_msg.people.list),
                }
                table_idx += 1

            if not self.tables:
                Logger.warn(self, "Scan finished but no customers mapped. Returning to WAIT.")
                self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL
                return

            # Sort tables by number of customers
            self.sort_tables_by_customers()
            self.subtask_manager.hri.say(
                f"I have mapped {len(self.tables)} table(s). I will start taking orders."
            )
            self.current_table_idx = 0
            self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_TABLE

        if self.current_state == RestaurantTaskManager.TaskStates.NAVIGATE_TO_TABLE:
            if self.current_table_idx >= len(self.tables_sorted_by_customers):
                Logger.info(self, "All tables served. Moving to bar.")
                self.subtask_manager.hri.say("I have taken all orders. I will now go to the bar.")
                self.current_table_idx = 0
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR
                return

            table_id = self.tables_sorted_by_customers[self.current_table_idx]
            table = self.tables[table_id]

            if self.current_customer_index < table["num_customers"]:
                # Take order from current customer
                Logger.state(
                    self,
                    f"Taking order from customer {self.current_customer_index} at table {table_id}",
                )

                # Navigation logic if it's the first customer at this table
                if self.current_customer_index == 0:
                    self.subtask_manager.hri.say("Navigating to your table using adaptive pathing.")
                    self.subtask_manager.nav.resume_nav()
                    self.subtask_manager.nav.change_bt("adaptive")

                    # Publish original goal for radial sampling
                    goal_msg = PoseStamped()
                    goal_msg.header.frame_id = "map"
                    goal_msg.header.stamp = self.get_clock().now().to_msg()
                    goal_msg.pose.position.x = table["coordinates"].point.x
                    goal_msg.pose.position.y = table["coordinates"].point.y
                    goal_msg.pose.position.z = 0.0
                    goal_msg.pose.orientation.w = 1.0
                    self.original_goal_pub.publish(goal_msg)

                    self.subtask_manager.nav.move_to_point(table["coordinates"])
                    self.subtask_manager.nav.pause_nav()

                # Get angle for the current customer (if available)
                customer_angle = table["customer_angles"][self.current_customer_index]

                self.subtask_manager.manipulation.pan_to(customer_angle)
                self.subtask_manager.vision.activate_face_recognition()
                self.subtask_manager.manipulation.follow_face(True)

                self.subtask_manager.manipulation.move_to_position("front_stare", velocity=0.5)

                # Use HRI take_order function
                status, orders = self.subtask_manager.hri.take_order(retries=3)
                if isinstance(status, tuple):
                    status = status[0]

                if status == Status.EXECUTION_SUCCESS and orders:
                    for order in orders:
                        table["orders"].append(order)
                        Logger.success(self, f"Order received: {order}")
                else:
                    Logger.warn(self, "Failed to get order from customer.")
                    self.subtask_manager.hri.say(
                        "Sorry, I didn't catch that. I'll move to the next customer."
                    )
                self.subtask_manager.manipulation.follow_face(False)
                self.current_customer_index += 1
            else:
                # Finished taking orders from all customers at this table
                Logger.info(
                    self,
                    f"Finished taking orders from all customers at table {table_id}",
                )

                # Move to next table
                self.subtask_manager.hri.say("I will take orders from other tables now.")
                self.current_table_idx += 1
                self.current_customer_index = 0

        if self.current_state == RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR:
            Logger.state(self, "Navigating to bar station to get items...")
            self.subtask_manager.hri.say("I will now go to the bar to get the orders.")

            self.subtask_manager.manipulation.move_to_position("nav_pose", velocity=0.5)
            self.subtask_manager.nav.resume_nav()
            # Return to the saved Bar pose
            Logger.info(self, "Navigating back to the Bar station pose.")
            self.subtask_manager.nav.move_to_pose(self.bar_pose)
            self.subtask_manager.nav.pause_nav()

            self.current_state = RestaurantTaskManager.TaskStates.SAY_ORDER_TO_BARMAN

        if self.current_state == RestaurantTaskManager.TaskStates.SAY_ORDER_TO_BARMAN:
            if not self._orders_given:
                Logger.state(self, "Communicating orders to the barman...")
                all_orders = []
                for tid in self.tables_sorted_by_customers:
                    all_orders.extend(self.tables[tid]["orders"])

                if all_orders:
                    order_text = ", ".join(all_orders)
                    self.subtask_manager.hri.say(
                        f"Hello barman, I have the following orders: {order_text}. Please help me prepare them."
                    )
                else:
                    self.subtask_manager.hri.say(
                        "Hello barman, actually I don't have any orders yet."
                    )
                self._orders_given = True

            self.current_state = RestaurantTaskManager.TaskStates.PREPARE_DELIVERY

        if self.current_state == RestaurantTaskManager.TaskStates.PREPARE_DELIVERY:
            if self.current_table_idx >= len(self.tables_sorted_by_customers):
                # All deliveries complete
                Logger.success(self, "All orders delivered!")
                self.subtask_manager.hri.say("I have delivered all orders. Service complete!")
                self.current_state = RestaurantTaskManager.TaskStates.END
            else:
                self.current_delivery_item_index = 0
                self.current_state = RestaurantTaskManager.TaskStates.DELIVER_TO_TABLE

        if self.current_state == RestaurantTaskManager.TaskStates.DELIVER_TO_TABLE:
            self.subtask_manager.hri.publish_display_topic(DETECTIONS_IMAGE_TOPIC)
            table_id = self.tables_sorted_by_customers[self.current_table_idx]
            table = self.tables[table_id]

            # Deliver items one by one
            if self.current_delivery_item_index < len(table["orders"]):
                item = table["orders"][self.current_delivery_item_index]

                Logger.state(
                    self,
                    f"Picking item {self.current_delivery_item_index + 1}/{len(table['orders'])}: {item}",
                )

                # Move arm to table_stare pose BEFORE detection/pick
                self.subtask_manager.manipulation.move_to_position("table_stare", velocity=0.5)

                status, error = self.pick_object(item)
                if isinstance(status, tuple):
                    status = status[0]

                if status == Status.EXECUTION_SUCCESS:
                    Logger.success(self, f"Picked {item}. Delivering to table {table_id}...")

                    # Navigate to the table using ADAPTIVE NAVIGATION
                    self.subtask_manager.manipulation.move_to_position("nav_pose", velocity=0.5)
                    self.subtask_manager.nav.resume_nav()

                    # 1. Activate Adaptive BT
                    self.subtask_manager.nav.change_bt("adaptive")

                    # 2. Publish Table Center as Anchor/Goal
                    goal_msg = PoseStamped()
                    goal_msg.header.frame_id = "map"
                    goal_msg.header.stamp = self.get_clock().now().to_msg()
                    goal_msg.pose.position.x = table["coordinates"].point.x
                    goal_msg.pose.position.y = table["coordinates"].point.y
                    goal_msg.pose.position.z = 0.0
                    goal_msg.pose.orientation.w = 1.0
                    self.original_goal_pub.publish(goal_msg)

                    # 3. Move to table using navigation stack
                    self.subtask_manager.nav.move_to_point(table["coordinates"])
                    self.subtask_manager.nav.pause_nav()

                    # Place item on table
                    self.subtask_manager.hri.say(f"Here is your {item}.")
                    self.subtask_manager.manipulation.move_to_position("table_stare", velocity=0.5)

                    status, error = self.place_object()
                    if isinstance(status, tuple):
                        status = status[0]

                    if status == Status.EXECUTION_SUCCESS:
                        Logger.success(self, f"Delivered {item} to table {table_id}")
                        self.current_delivery_item_index += 1

                        # Return to bar for next item
                        if self.current_delivery_item_index < len(table["orders"]):
                            self.subtask_manager.hri.say("I will get your next item.")
                            self.subtask_manager.nav.resume_nav()
                            self.subtask_manager.nav.move_to_pose(self.bar_pose)
                            self.subtask_manager.nav.pause_nav()
                    else:
                        Logger.error(self, f"Failed to place {item}: {error}")
                        self.current_delivery_item_index += 1
                else:
                    Logger.error(self, f"Failed to pick {item}: {error}")
                    self.current_delivery_item_index += 1

            else:
                # Finished delivering all items to this table
                self.subtask_manager.hri.say("Enjoy your meal!")
                self.current_table_idx += 1
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR

        if self.current_state == RestaurantTaskManager.TaskStates.END:
            Logger.state(self, "Restaurant task complete!")
            self.subtask_manager.hri.say("All customers have been served. Restaurant complete.")
            self.subtask_manager.hri.reset_task_status()
            self.running_task = False


def main(args=None):  #
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
