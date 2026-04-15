#!/usr/bin/env python3

"""
Task Manager for Restaurant task of Robocup @Home 2026
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped

from frida_constants.vision_constants import (
    RESTAURANT_TABLES_TOPIC,
    DETECTIONS_IMAGE_TOPIC,
)

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
        NAVIGATE_TO_BAR = "NAVIGATE_TO_BAR"
        SAY_ORDER_TO_BARMAN = "SAY_ORDER_TO_BARMAN"
        PREPARE_DELIVERY = "PREPARE_DELIVERY"
        DELIVER_TO_TABLE = "DELIVER_TO_TABLE"
        END = "END"

    def __init__(self):
        """Initialize the node"""
        super().__init__("restaurant_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.RESTAURANT, mock_areas=[])

        self.running_task = True

        self.original_goal_pub = self.create_publisher(
            PoseStamped, "/adaptive_nav/original_goal", 10
        )

        # Table data: {table_id: {'customer_points', 'customer_angles', 'orders', 'coordinates', 'num_customers'}}
        self.tables = {}
        self.tables_sorted_by_customers = []
        self.current_table_idx = 0

        self.current_customer_index = 0
        self.current_delivery_item_index = 0

        # Target customer detected during WAIT_FOR_CALL
        self.target_person_point = None

        # Progressive search state
        self.search_step = 0

        # Bar position saved at START via TF
        self.bar_pose = None

        # Orders already communicated to barman
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
            self.tables.keys(),
            key=lambda tid: self.tables[tid]["num_customers"],
            reverse=True,
        )
        Logger.info(
            self,
            f"Tables sorted by customers: {[(tid, self.tables[tid]['num_customers']) for tid in self.tables_sorted_by_customers]}",
        )

    def _compute_search_waypoint(self, step: int) -> PointStamped:
        """Return a PointStamped `step * SEARCH_STEP_SIZE` m ahead of the bar."""
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

    def _publish_adaptive_goal(self, point: PointStamped):
        """Publish original goal for adaptive BT radial sampling."""
        goal_msg = PoseStamped()
        goal_msg.header = point.header
        goal_msg.pose.position = point.point
        goal_msg.pose.orientation.w = 1.0
        self.original_goal_pub.publish(goal_msg)

    def deus_pick(self, object_name):
        """Fallback: ask human to place object in gripper."""
        Logger.warn(self, f"Requesting human assistance for {object_name}.")
        self.subtask_manager.manipulation.open_gripper()
        self.subtask_manager.hri.say(
            f"I am having trouble picking the {object_name}. Please place it in my gripper and say yes when done."
        )
        _, confirmation = self.subtask_manager.hri.confirm(
            "Have you placed the object in my gripper?",
            use_hotwords=True,
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
        Logger.info(self, f"Matched '{object_name}' to '{matched_label}'")

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
            use_hotwords=True,
            retries=3,
            wait_between_retries=5,
        )
        if confirmation == "yes":
            self.subtask_manager.manipulation.open_gripper()
            self.subtask_manager.hri.say("Thank you.")
            return Status.EXECUTION_SUCCESS

        return Status.EXECUTION_ERROR

    def _nav_to_bar(self):
        """Navigate back to the saved bar pose."""
        if self.bar_pose is None:
            Logger.warn(self, "Bar pose not saved, skipping navigation to bar.")
            return
        self.subtask_manager.manipulation.move_to_position("nav_pose", velocity=0.5)
        self.subtask_manager.nav.resume_nav()
        self.subtask_manager.nav.move_to_pose(self.bar_pose)
        self.subtask_manager.nav.pause_nav()

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
            self.bar_pose = self.subtask_manager.nav.get_current_pose()
            Logger.info(self, "Bar position saved.")
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
                self.search_step = 0
                self.current_state = RestaurantTaskManager.TaskStates.MOVE_TO_TABLES_AREA

            elif self.search_step < MAX_SEARCH_STEPS:
                self.search_step += 1
                waypoint = self._compute_search_waypoint(self.search_step)
                if waypoint is not None:
                    Logger.info(self, f"No customer. Moving to search position {self.search_step}.")
                    self.subtask_manager.nav.resume_nav()
                    self.subtask_manager.nav.move_to_point(waypoint)
                    self.subtask_manager.nav.pause_nav()
                else:
                    self.timeout(1)
                return

            else:
                Logger.info(self, "Full area scanned, no customer. Returning to bar.")
                self.subtask_manager.hri.say(
                    "I did not find any customers calling. I will return to the bar and wait."
                )
                self.search_step = 0
                self._nav_to_bar()
                self.timeout(2)
                return

        if self.current_state == RestaurantTaskManager.TaskStates.MOVE_TO_TABLES_AREA:
            if self.target_person_point is None:
                self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL
                return

            Logger.state(self, "Navigating to detected customer position...")
            self.subtask_manager.nav.resume_nav()
            self.subtask_manager.nav.change_bt("adaptive")
            self._publish_adaptive_goal(self.target_person_point)
            status = self.subtask_manager.nav.move_to_point(self.target_person_point)
            self.subtask_manager.nav.pause_nav()

            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Arrived near tables for detection.")
                self.current_state = RestaurantTaskManager.TaskStates.DETECT_CUSTOMERS
            else:
                Logger.warn(self, "Navigation to customer failed. Returning to WAIT_FOR_CALL.")
                self.target_person_point = None
                self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL

        if self.current_state == RestaurantTaskManager.TaskStates.DETECT_CUSTOMERS:
            Logger.state(self, "Performing full table scan...")
            self.subtask_manager.manipulation.move_to_position("front_stare", velocity=0.5)
            status, customer_tables = self.subtask_manager.vision.customer_tables()
            self.subtask_manager.hri.publish_display_topic(RESTAURANT_TABLES_TOPIC)

            if status != Status.EXECUTION_SUCCESS or not customer_tables:
                Logger.warn(self, "No tables detected, retrying...")
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
                Logger.warn(self, "No customers mapped. Returning to WAIT_FOR_CALL.")
                self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_CALL
                return

            self.sort_tables_by_customers()
            self.subtask_manager.hri.say(
                f"I have mapped {len(self.tables)} table(s). I will start taking orders."
            )
            self.current_table_idx = 0
            self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_TABLE

        if self.current_state == RestaurantTaskManager.TaskStates.NAVIGATE_TO_TABLE:
            if self.current_table_idx >= len(self.tables_sorted_by_customers):
                Logger.info(self, "All tables served. Going to bar.")
                self.subtask_manager.hri.say("I have taken all orders. I will now go to the bar.")
                self.current_table_idx = 0
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR
                return

            table_id = self.tables_sorted_by_customers[self.current_table_idx]
            table = self.tables[table_id]

            if self.current_customer_index < table["num_customers"]:
                Logger.state(
                    self,
                    f"Taking order from customer {self.current_customer_index} at table {table_id}",
                )

                # Navigate to table on first customer only
                if self.current_customer_index == 0:
                    self.subtask_manager.hri.say("Navigating to your table.")
                    self.subtask_manager.nav.resume_nav()
                    self.subtask_manager.nav.change_bt("adaptive")
                    self._publish_adaptive_goal(table["coordinates"])
                    self.subtask_manager.nav.move_to_point(table["coordinates"])
                    self.subtask_manager.nav.pause_nav()

                customer_angle = table["customer_angles"][self.current_customer_index]
                self.subtask_manager.manipulation.pan_to(customer_angle)
                self.subtask_manager.vision.activate_face_recognition()
                self.subtask_manager.manipulation.follow_face(True)
                self.subtask_manager.manipulation.move_to_position("front_stare", velocity=0.5)

                status, orders = self.subtask_manager.hri.take_order(retries=3)
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
                Logger.info(self, f"Done with table {table_id}. Moving to next.")
                self.subtask_manager.hri.say("I will take orders from other tables now.")
                self.current_table_idx += 1
                self.current_customer_index = 0

        if self.current_state == RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR:
            Logger.state(self, "Navigating to bar...")
            self.subtask_manager.hri.say("I will now go to the bar to get the orders.")
            self._nav_to_bar()
            self.current_state = RestaurantTaskManager.TaskStates.SAY_ORDER_TO_BARMAN

        if self.current_state == RestaurantTaskManager.TaskStates.SAY_ORDER_TO_BARMAN:
            if not self._orders_given:
                Logger.state(self, "Communicating orders to barman...")
                all_orders = []
                for tid in self.tables_sorted_by_customers:
                    all_orders.extend(self.tables[tid]["orders"])

                if all_orders:
                    order_text = ", ".join(all_orders)
                    self.subtask_manager.hri.say(
                        f"Hello barman, I have the following orders: {order_text}. Please help me prepare them."
                    )
                else:
                    self.subtask_manager.hri.say("Hello barman, I don't have any orders yet.")
                self._orders_given = True

            self.current_state = RestaurantTaskManager.TaskStates.PREPARE_DELIVERY

        if self.current_state == RestaurantTaskManager.TaskStates.PREPARE_DELIVERY:
            if self.current_table_idx >= len(self.tables_sorted_by_customers):
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

            if self.current_delivery_item_index < len(table["orders"]):
                item = table["orders"][self.current_delivery_item_index]
                Logger.state(
                    self,
                    f"Picking item {self.current_delivery_item_index + 1}/{len(table['orders'])}: {item}",
                )

                self.subtask_manager.manipulation.move_to_position("table_stare", velocity=0.5)
                status = self.pick_object(item)

                if status == Status.EXECUTION_SUCCESS:
                    Logger.success(self, f"Picked {item}. Delivering to table {table_id}...")

                    self.subtask_manager.manipulation.move_to_position("nav_pose", velocity=0.5)
                    self.subtask_manager.nav.resume_nav()
                    self.subtask_manager.nav.change_bt("adaptive")
                    self._publish_adaptive_goal(table["coordinates"])
                    self.subtask_manager.nav.move_to_point(table["coordinates"])
                    self.subtask_manager.nav.pause_nav()

                    self.subtask_manager.hri.say(f"Here is your {item}.")
                    self.subtask_manager.manipulation.move_to_position("table_stare", velocity=0.5)
                    status = self.place_object()

                    if status == Status.EXECUTION_SUCCESS:
                        Logger.success(self, f"Delivered {item} to table {table_id}")
                        self.current_delivery_item_index += 1

                        # Return to bar for next item
                        if self.current_delivery_item_index < len(table["orders"]):
                            self.subtask_manager.hri.say("I will get your next item.")
                            self._nav_to_bar()
                    else:
                        Logger.error(self, f"Failed to place {item}")
                        self.current_delivery_item_index += 1
                else:
                    Logger.error(self, f"Failed to pick {item}")
                    self.current_delivery_item_index += 1

            else:
                self.subtask_manager.hri.say("Enjoy your meal!")
                self.current_table_idx += 1
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR

        if self.current_state == RestaurantTaskManager.TaskStates.END:
            Logger.state(self, "Restaurant task complete!")
            self.subtask_manager.hri.say("All customers have been served. Restaurant complete.")
            self.subtask_manager.hri.reset_task_status()
            self.running_task = False


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
