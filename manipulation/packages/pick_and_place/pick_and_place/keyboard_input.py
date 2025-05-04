#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from frida_interfaces.action import ManipulationAction
from frida_interfaces.msg import ManipulationTask
import time
from frida_interfaces.msg import ObjectDetectionArray
from frida_constants.vision_constants import (
    DETECTIONS_TOPIC,
)
from frida_constants.manipulation_constants import (
    MANIPULATION_ACTION_SERVER,
)


class KeyboardInput(Node):
    def __init__(self):
        super().__init__("keyboard_input")
        self._action_client = ActionClient(
            self, ManipulationAction, MANIPULATION_ACTION_SERVER
        )
        self.objects = []  # Example list of objects

        # Add subscriber for objects
        self.objects_subscription = self.create_subscription(
            ObjectDetectionArray,
            DETECTIONS_TOPIC,
            self.objects_callback,
            10,
        )

    def objects_callback(self, msg):
        # Assuming msg contains a list of object names
        self.objects = []
        for detection in msg.detections:
            if detection.label_text not in self.objects:
                self.objects.append(detection.label_text)

    def send_pick_request(self, object_name):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PICK
        goal_msg.pick_params.object_name = object_name

        self.get_logger().info(f"Sending pick request for: {object_name}")
        self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.get_logger().info(f"Pick request for {object_name} sent")

    def send_place_request(
        self, is_shelf=False, table_height=None, table_height_tolerance=None
    ):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PLACE
        goal_msg.place_params.is_shelf = is_shelf
        goal_msg.scan_environment = is_shelf
        if table_height is not None and table_height_tolerance is not None:
            goal_msg.place_params.table_height = table_height
            goal_msg.place_params.table_height_tolerance = table_height_tolerance
        self.get_logger().info("Sending place request")
        self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.get_logger().info("Place request sent")

    def send_pour_request(self, object_name, bowl_name):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.POUR
        goal_msg.pour_params.object_name = object_name
        goal_msg.pour_params.bowl_name = bowl_name

        self.get_logger().info(f"Sending pour request for: {object_name}")
        self.get_logger().info(f"Pouring into: {bowl_name}")
        self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self.get_logger().info("Pour request sent")

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback}")

    def refresh_objects(self):
        self.get_logger().info("Refreshing objects list...")
        # Spin for 1 second to receive messages
        start_time = time.time()
        while time.time() - start_time < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Objects list refreshed.")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInput()

    try:
        while rclpy.ok():
            node.refresh_objects()
            print("\nAvailable objects:")
            for i, obj in enumerate(node.objects, start=1):
                print(f"{i}. {obj}")
            print("-2. Refresh objects list")
            print("-3. Place")
            print("-4. Place on shelf")
            print("-5. Place on shelf (with plane height)")
            print("-6. Pour")
            print("q. Quit")

            choice = input("\nEnter your choice: ")
            if choice.lower() == "q":
                break

            if choice.lower() == "q":
                break
            elif choice == "-2":
                node.refresh_objects()
            elif choice == "-3":
                node.send_place_request()
            elif choice == "-4":
                node.send_place_request(is_shelf=True)
            elif choice == "-5":
                # receive two floats, height and height tolerance
                height = input("Enter height (in m -> e.g. 0.3): ")
                height_tolerance = input("Enter height tolerance (in m): ")
                try:
                    height = float(height)
                    height_tolerance = float(height_tolerance)
                    node.get_logger().info(
                        f"Sending place request on shelf with height: {height} and tolerance: {height_tolerance}"
                    )
                    node.send_place_request(
                        is_shelf=True,
                        table_height=height,
                        table_height_tolerance=height_tolerance,
                    )
                except ValueError:
                    print(
                        "Invalid input. Please enter valid numbers for height and tolerance."
                    )
            elif choice == "-6":
                # receive object name
                object_name = input("Enter object name: ")
                bowl_name = input("Enter bowl name: ")
                node.send_pour_request(object_name, bowl_name)

            elif choice.isdigit():
                try:
                    choice_num = int(choice)
                    if 0 <= choice_num - 1 < len(node.objects):
                        node.send_pick_request(node.objects[choice_num - 1])
                    else:
                        print("Invalid choice. Please try again.")
                except ValueError:
                    print("Invalid input. Please enter a number.")
            else:
                node.send_pick_request(choice)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
