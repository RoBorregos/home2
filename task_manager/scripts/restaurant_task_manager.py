#!/usr/bin/env python3

"""
Task Manager for Restaurant task of Robocup @Home 2025
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from utils.subtask_manager import SubtaskManager, Task

from utils.logger import Logger
from utils.status import Status
import time

MOCK_NAV = False
MOCK_MANIPULATION = False
MOCK_HRI = False
MOCK_VISION = False

ATTEMPT_LIMIT = 3


class RestaurantTaskManager(Node):
    """Class to manage the restaurant task"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        MOVE_TO_TABLES_AREA = "MOVE_TO_TABLES_AREA"
        DETECT_CUSTOMERS = "DETECT_CUSTOMERS"
        NAVIGATE_TO_TABLE = "NAVIGATE_TO_TABLE"
        TAKE_ORDERS = "TAKE_ORDERS"
        NAVIGATE_TO_BAR = "NAVIGATE_TO_BAR"
        PICK_ORDER = "PICK_ORDER"
        DELIVER_TO_TABLE = "DELIVER_TO_TABLE"
        END = "END"

    def __init__(self):
        """Initialize the node"""
        super().__init__("restaurant_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.RESTAURANT, mock_areas=[])

        self.running_task = True

        # Table data structure: {table_id: {'customer_points': [PointStamped], 'customer_angles': [float], 'orders': [str], 'coordinates': PointStamped, 'num_customers': int}}
        self.tables = {}
        self.table_counter = 0
        self.current_table_id = None
        self.tables_sorted_by_customers = []  # descending
        self.current_table_index = 0

        # Tracking current operations
        self.current_customer_index = 0
        self.current_delivery_item_index = 0
        self.pick_attempts = 0

        # Pan angles per-customer (from vision)
        self.pan_angles = []

        self.current_state = RestaurantTaskManager.TaskStates.WAIT_FOR_BUTTON

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

    def pick_object_with_retries(self, object_name):
        """Attempt to pick object with retries, fallback to deux ex machina."""
        for attempt in range(ATTEMPT_LIMIT):
            Logger.info(self, f"Pick attempt {attempt + 1}/{ATTEMPT_LIMIT} for {object_name}")
            status = self.subtask_manager.manipulation.pick_object(object_name)

            if status == Status.EXECUTION_SUCCESS:
                return Status.EXECUTION_SUCCESS

            self.timeout(1)

        # Failed after all attempts - deux ex machina
        Logger.warning(
            self,
            f"Failed to pick {object_name} after {ATTEMPT_LIMIT} attempts. Requesting human assistance.",
        )
        self.subtask_manager.manipulation.open_gripper()
        self.subtask_manager.hri.say(
            f"I am having trouble picking the {object_name}. Please place it in my gripper and say yes when done."
        )

        status, confirmation = self.subtask_manager.hri.confirm(
            "Have you placed the object in my gripper?", False, retries=3, wait_between_retries=2
        )

        if confirmation == "yes":
            self.subtask_manager.manipulation.close_gripper()
            self.subtask_manager.hri.say("Thank you. I have received the object.")
            return Status.EXECUTION_SUCCESS

        return Status.EXECUTION_ERROR

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
            self.current_state = RestaurantTaskManager.TaskStates.MOVE_TO_TABLES_AREA

        if self.current_state == RestaurantTaskManager.TaskStates.MOVE_TO_TABLES_AREA:
            Logger.state(self, "Moving closer to tables area for better detection...")
            self.subtask_manager.hri.say(
                "I will move closer to the tables area to detect customers."
            )

            # Move to a location with good view of tables
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="nav_pose", velocity=0.5, degrees=True
            )
            self.subtask_manager.nav.resume_nav()
            # Navigate to a predefined point near the tables
            near_tables_point = PointStamped()
            near_tables_point.header.frame_id = "map"
            near_tables_point.point.x = (
                2.0  # TODO:Adjust these coordinates based on actual environment
            )
            near_tables_point.point.y = 1.0
            near_tables_point.point.z = 0.0
            Logger.state(
                self,
                f"Navigating to point near tables: ({near_tables_point.point.x}, {near_tables_point.point.y})",
            )
            status = self.subtask_manager.nav.move_to_point(near_tables_point)
            self.subtask_manager.nav.pause_nav()
            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Arrived near tables for customer detection.")
                # Get table groups and positions directly from vision task
                status, customer_tables = self.subtask_manager.vision.customer_tables()
                if status != Status.EXECUTION_SUCCESS or not customer_tables:
                    Logger.error(
                        self,
                        "Could not get table positions from vision task. No table positions available.",
                    )
                    self.current_state = RestaurantTaskManager.TaskStates.END
                    return

                Logger.info(self, f"Detected {len(customer_tables)} table(s) with customers")

                # Create table entries (customer_tables is now a list of dicts or similar, not message objects)
                for idx, table_data in enumerate(customer_tables):
                    table_id = self.table_counter
                    self.table_counter += 1
                    # Only use 'people_points' and 'people_angles' now
                    customer_points = table_data.get("people_points", [])
                    customer_angles = table_data.get("people_angles", [])
                    num_customers = len(customer_points)
                    self.tables[table_id] = {
                        "customer_points": customer_points,
                        "customer_angles": customer_angles,
                        "orders": [],
                        "coordinates": None,  # No table_point anymore
                        "num_customers": num_customers,
                    }

                # Sort tables by number of customers (greedy strategy)
                self.sort_tables_by_customers()

                self.subtask_manager.hri.say(
                    f"I have detected {len(customer_tables)} table(s) with customers. I will start taking orders."
                )

                # Start taking orders from first table
                self.current_table_index = 0
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_TABLE

        if self.current_state == RestaurantTaskManager.TaskStates.NAVIGATE_TO_TABLE:
            if self.current_table_index >= len(self.tables_sorted_by_customers):
                # All tables visited for orders
                Logger.success(self, "All orders taken from all tables")
                self.subtask_manager.hri.say(
                    "I have taken all orders. I will now prepare to serve."
                )
                self.current_table_index = 0  # Reset for delivery phase
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR
            else:
                self.current_table_id = self.tables_sorted_by_customers[self.current_table_index]
                table = self.tables[self.current_table_id]

                Logger.state(
                    self,
                    f"Navigating to table {self.current_table_id} ({table['num_customers']} customers)...",
                )
                self.subtask_manager.hri.say(f"I will now go to table {self.current_table_id + 1}.")

                # No table point to navigate to; optionally, could use first customer point if needed
                # If you want to move to a customer, you could do:
                # if table["customer_points"]:
                #     target_point = table["customer_points"][0]
                #     self.subtask_manager.nav.move_to_point(target_point)
                # else:
                #     target_point = None
                # For now, skip navigation to table point

                self.current_customer_index = 0
                self.current_state = RestaurantTaskManager.TaskStates.TAKE_TABLE_ORDERS

        if self.current_state == RestaurantTaskManager.TaskStates.TAKE_TABLE_ORDERS:
            table = self.tables[self.current_table_id]

            if self.current_customer_index < table["num_customers"]:
                # Take order from current customer
                Logger.state(
                    self,
                    f"Taking order from customer {self.current_customer_index + 1}/{table['num_customers']} at table {self.current_table_id}",
                )

                # Get angle for the current customer
                customer_angle = None
                if (
                    "customer_angles" in table
                    and len(table["customer_angles"]) > self.current_customer_index
                ):
                    customer_angle = table["customer_angles"][self.current_customer_index]
                else:
                    customer_angle = 0  # fallback if not available

                # Face the customer using the angle
                self.subtask_manager.manipulation.pan_to(customer_angle)
                self.timeout(1)
                self.subtask_manager.manipulation.move_joint_positions(
                    named_position="front_stare", velocity=0.5, degrees=True
                )
                self.subtask_manager.manipulation.follow_face(True)

                self.subtask_manager.hri.say("Hello! What would you like to order?")

                status, order = self.subtask_manager.hri.ask_and_confirm(
                    question="What is your order?", use_hotwords=False
                )

                if status == Status.EXECUTION_SUCCESS and order:
                    # Save order with customer coordinates
                    customer_point = None
                    if (
                        "customer_points" in table
                        and len(table["customer_points"]) > self.current_customer_index
                    ):
                        customer_point = table["customer_points"][self.current_customer_index]
                    table["orders"].append({"order": order, "customer_point": customer_point})
                    self.subtask_manager.hri.say(f"Thank you! I have noted your order: {order}.")
                    Logger.success(
                        self,
                        f"Order {self.current_customer_index + 1} for table {self.current_table_id}: {order}",
                    )
                else:
                    # Ask again if not understood the first time
                    self.subtask_manager.hri.say(
                        "I'm sorry, I didn't catch that. Could you please repeat your order?"
                    )
                    status2, order2 = self.subtask_manager.hri.ask_and_confirm(
                        question="What is your order?", use_hotwords=False
                    )
                    if status2 == Status.EXECUTION_SUCCESS and order2:
                        customer_point = None
                        if (
                            "customer_points" in table
                            and len(table["customer_points"]) > self.current_customer_index
                        ):
                            customer_point = table["customer_points"][self.current_customer_index]
                        table["orders"].append({"order": order2, "customer_point": customer_point})
                        self.subtask_manager.hri.say(
                            f"Thank you! I have noted your order: {order2}."
                        )
                        Logger.success(
                            self,
                            f"Order {self.current_customer_index + 1} for table {self.current_table_id}: {order2}",
                        )
                    else:
                        self.subtask_manager.hri.say(
                            "I'm sorry, I still didn't catch that. I will move on to the next customer."
                        )
                self.subtask_manager.manipulation.follow_face(False)
                self.current_customer_index += 1

            else:
                # Finished taking orders from all customers, ask for additional orders
                Logger.info(
                    self,
                    f"Finished taking orders from all customers at table {self.current_table_id}",
                )
                self.subtask_manager.hri.say("Would anyone like to order something else?")

                status, additional = self.subtask_manager.hri.confirm(
                    "Does anyone want to order something else? Say yes or no.",
                    False,
                    retries=3,
                    wait_between_retries=2,
                )

                if additional == "yes":
                    self.subtask_manager.hri.say("Of course! What would you like to add?")
                    status, order = self.subtask_manager.hri.ask_and_confirm(
                        question="What is your additional order?", use_hotwords=False
                    )
                    if status == Status.EXECUTION_SUCCESS and order:
                        # Save with no specific customer_point (could be improved by asking)
                        table["orders"].append({"order": order, "customer_point": None})
                        self.subtask_manager.hri.say(f"I have added {order} to your table's order.")

                # Move to next table
                self.subtask_manager.hri.say("I will take orders from other tables now.")
                self.current_table_index += 1
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_TABLE

        if self.current_state == RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR:
            Logger.state(self, "Navigating to bar station to get items...")
            self.subtask_manager.hri.say("I will now go to the bar to get the orders.")

            self.subtask_manager.manipulation.move_joint_positions(
                named_position="nav_pose", velocity=0.5, degrees=True
            )
            self.subtask_manager.nav.resume_nav()
            future = self.subtask_manager.nav.move_to_zero()
            rclpy.spin_until_future_complete(self.subtask_manager.nav.node, future)
            self.subtask_manager.nav.pause_nav()

            self.current_state = RestaurantTaskManager.TaskStates.PREPARE_DELIVERY

        if self.current_state == RestaurantTaskManager.TaskStates.PREPARE_DELIVERY:
            if self.current_table_index >= len(self.tables_sorted_by_customers):
                # All deliveries complete
                Logger.success(self, "All orders delivered!")
                self.subtask_manager.hri.say("I have delivered all orders. Service complete!")
                self.current_state = RestaurantTaskManager.TaskStates.END
            else:
                self.current_table_id = self.tables_sorted_by_customers[self.current_table_index]
                table = self.tables[self.current_table_id]

                Logger.state(
                    self,
                    f"Preparing delivery for table {self.current_table_id} ({len(table['orders'])} items)...",
                )

                # Pick items one by one
                Logger.info(
                    self,
                    f"Table {self.current_table_id} has {table['num_customers']} customers. Picking items individually.",
                )
                table["using_tray"] = False
                self.current_delivery_item_index = 0
                self.current_state = RestaurantTaskManager.TaskStates.DELIVER_TO_TABLE

        if self.current_state == RestaurantTaskManager.TaskStates.DELIVER_TO_TABLE:
            table = self.tables[self.current_table_id]

            # Deliver items one by one

            if self.current_delivery_item_index < len(table["orders"]):
                order_entry = table["orders"][self.current_delivery_item_index]
                item = order_entry["order"] if isinstance(order_entry, dict) else order_entry
                customer_point = (
                    order_entry["customer_point"] if isinstance(order_entry, dict) else None
                )

                Logger.state(
                    self,
                    f"Picking item {self.current_delivery_item_index + 1}/{len(table['orders'])}: {item}",
                )

                # At bar, pick the item
                self.subtask_manager.manipulation.move_joint_positions(
                    named_position="front_stare", velocity=0.5, degrees=True
                )

                status = self.pick_object(item)

                if status == Status.EXECUTION_SUCCESS:
                    Logger.success(
                        self, f"Picked {item}. Delivering to table {self.current_table_id}..."
                    )

                    # Navigate to the saved customer_point if available, else fallback
                    self.subtask_manager.manipulation.move_joint_positions(
                        named_position="nav_pose", velocity=0.5, degrees=True
                    )
                    self.subtask_manager.nav.resume_nav()
                    if customer_point is not None:
                        self.subtask_manager.nav.move_to_point(customer_point)
                    else:
                        # fallback: move to table (could be None)
                        self.subtask_manager.nav.move_to_point(table["coordinates"])
                    self.subtask_manager.nav.pause_nav()

                    # Place item on table
                    self.subtask_manager.hri.say(f"Here is your {item}.")
                    self.subtask_manager.manipulation.move_joint_positions(
                        named_position="table_stare", velocity=0.5, degrees=True
                    )

                    status = self.place_object_with_retries()

                    if status == Status.EXECUTION_SUCCESS:
                        Logger.success(self, f"Delivered {item} to table {self.current_table_id}")
                        self.current_delivery_item_index += 1

                        # Return to bar for next item
                        if self.current_delivery_item_index < len(table["orders"]):
                            self.subtask_manager.hri.say("I will get your next item.")
                            self.subtask_manager.nav.resume_nav()
                            future = self.subtask_manager.nav.move_to_zero()
                            rclpy.spin_until_future_complete(self.subtask_manager.nav.node, future)
                            self.subtask_manager.nav.pause_nav()
                    else:
                        Logger.error(self, f"Failed to place {item}")
                        self.current_delivery_item_index += 1
                else:
                    Logger.error(self, f"Failed to pick {item}")
                    self.current_delivery_item_index += 1

            else:
                # Finished delivering all items to this table
                self.subtask_manager.hri.say("Enjoy your meal!")
                self.current_table_index += 1
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_BAR

        if self.current_state == RestaurantTaskManager.TaskStates.END:
            Logger.state(self, "Restaurant task complete!")
            self.subtask_manager.hri.say("All customers have been served. Restaurant complete.")
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
