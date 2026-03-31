#!/usr/bin/env python3

"""
Task Manager for Restaurant task of Robocup @Home 2026
"""

from frida_constants.vision_constants import (
    RESTAURANT_TABLES_TOPIC,
    DETECTIONS_IMAGE_TOPIC,
)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from task_manager.utils.subtask_manager import SubtaskManager, Task

from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
import time

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
        PREPARE_DELIVERY = "PREPARE_DELIVERY"
        DELIVER_TO_TABLE = "DELIVER_TO_TABLE"
        END = "END"

    def __init__(self):
        """Initialize the node"""
        super().__init__("restaurant_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.RESTAURANT, mock_areas=["navigation"])

        self.running_task = True

        # Table data structure: {table_id: {'customer_points': [PointStamped], 'customer_angles': [float], 'orders': [str], 'coordinates': PointStamped, 'num_customers': int}}
        self.tables = {}
        # self.tables = {
        #     0: {
        #         'customer_points': [PointStamped(), PointStamped()],
        #         'customer_angles': [0.0, 1.57],
        #         'orders': [],
        #         'coordinates': PointStamped(),
        #         'num_customers': 2
        #     },
        #     1: {
        #         'customer_points': [PointStamped(), PointStamped()],
        #         'customer_angles': [3.14, -1.57],
        #         'orders': [],
        #         'coordinates': PointStamped(),
        #         'num_customers': 2
        #     }
        # }
        self.table_counter = 0
        self.current_table_id = None
        self.tables_sorted_by_customers = []  # descending
        self.current_table_id = 0

        # Tracking current operations
        self.current_customer_index = 0
        self.current_delivery_item_index = 0
        self.pick_attempts = 0

        # Pan angles per-customer
        self.pan_angles = []

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

    def pick_object(self, object_name):
        """Attempt to pick object with retries, fallback to deux ex machina."""
        for attempt in range(ATTEMPT_LIMIT):
            Logger.info(self, f"Pick attempt {attempt + 1}/{ATTEMPT_LIMIT} for {object_name}")
            status = self.subtask_manager.manipulation.pick_object(object_name)

            if status == Status.EXECUTION_SUCCESS:
                return Status.EXECUTION_SUCCESS

            self.timeout(1)

        # Failed after all attempts - deux ex machina
        Logger.warn(
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
            self.subtask_manager.hri.say("I will move closer to the tables area.")
            # TODO: move to a point near the tables area
            # status = self.subtask_manager.nav.move_to_point(forward_point)
            status = Status.EXECUTION_SUCCESS
            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Arrived near tables for customer detection.")
                # Get table groups and positions directly from vision task
                status, customer_tables = self.subtask_manager.vision.customer_tables()
                self.subtask_manager.hri.publish_display_topic(RESTAURANT_TABLES_TOPIC)
                if status != Status.EXECUTION_SUCCESS or not customer_tables:
                    Logger.warn(
                        self, "No tables with customers detected. Waiting and retrying..."
                    )
                    self.timeout(2)
                    return
                Logger.info(self, f"Detected {len(customer_tables)} table(s) with customers")
                idx = 0
                for table_msg in customer_tables:
                    table_id = idx
                    customer_points = []
                    customer_angles = []
                    orders = []
                    num_customers = 0
                    if len(table_msg.people.list) == 0:
                        Logger.warning(
                            self, f"Table {table_id} has no detected customers, skipping..."
                        )
                        continue
                    for person in table_msg.people.list:
                        customer_points.append(person.point3d)
                        customer_angles.append(person.angle)
                    num_customers = len(table_msg.people.list)
                    self.tables[table_id] = {
                        "customer_points": customer_points,
                        "customer_angles": customer_angles,
                        "orders": orders,
                        "coordinates": table_msg.table_point,
                        "num_customers": num_customers,
                    }
                    idx += 1
                # Sort tables by number of customers (greedy strategy)
                self.sort_tables_by_customers()
                self.subtask_manager.hri.say(
                    f"I have detected {len(customer_tables)} table(s) with customers. I will start taking orders."
                )
                # Start taking orders from first table
                self.current_table_id = 0
                self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_TABLE

            # Logger.success(self, "Arrived near tables for customer detection.")
            # # Get table groups and positions directly from vision task
            # # Create table entries
            # # Sort tables by number of customers (greedy strategy)
            # self.sort_tables_by_customers()
            # self.subtask_manager.hri.say(
            #     f"I have detected {len(self.tables)} table(s) with customers. I will start taking orders."
            # )
            # # Start taking orders from first table
            # self.current_table_id = 0
            # self.current_state = RestaurantTaskManager.TaskStates.NAVIGATE_TO_TABLE

        if self.current_state == RestaurantTaskManager.TaskStates.NAVIGATE_TO_TABLE:
            table = self.tables[self.current_table_id]
            self.current_customer = 0
            if self.current_customer < table["num_customers"]:
                # Take order from current customer
                Logger.state(
                    self,
                    f"Taking order from customer {self.current_customer} at table {self.current_table_id}",
                )
                # Get angle for the current customer (if available)
                customer_angle = table["customer_angles"][self.current_customer_index]
                # Face the customer
                self.subtask_manager.manipulation.pan_to(customer_angle)
                self.subtask_manager.vision.activate_face_recognition()
                self.subtask_manager.manipulation.follow_face(True)

                self.subtask_manager.manipulation.move_joint_positions(
                    named_position="front_stare", velocity=0.5, degrees=True
                )
                self.subtask_manager.manipulation.follow_face(True)
                # Use HRI take_order function
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
                # Finished taking orders from all customers, ask for additional orders
                Logger.info(
                    self,
                    f"Finished taking orders from all customers at table {self.current_table_id + 1}",
                )

                # Move to next table
                self.subtask_manager.hri.say("I will take orders from other tables now.")
                self.current_table_id += 1
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
            if self.current_table_id >= len(self.tables_sorted_by_customers):
                # All deliveries complete
                Logger.success(self, "All orders delivered!")
                self.subtask_manager.hri.say("I have delivered all orders. Service complete!")
                self.current_state = RestaurantTaskManager.TaskStates.END
            else:
                self.current_table_id = self.tables_sorted_by_customers[self.current_table_id]
                table = self.tables[self.current_table_id]

                Logger.state(
                    self,
                    f"Preparing delivery for table {self.current_table_id} ({len(table['orders'])} items)...",
                )

                self.current_delivery_item_index = 0
                self.current_state = RestaurantTaskManager.TaskStates.DELIVER_TO_TABLE

        if self.current_state == RestaurantTaskManager.TaskStates.DELIVER_TO_TABLE:
            self.subtask_manager.hri.publish_display_topic(DETECTIONS_IMAGE_TOPIC)
            table = self.tables[self.current_table_id]

            # Deliver items one by one
            if self.current_delivery_item_index < len(table["orders"]):
                order_entry = table["orders"][self.current_delivery_item_index]
                item = order_entry["order"]
                customer_point = order_entry["customer_point"]

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
                self.current_table_id += 1
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
