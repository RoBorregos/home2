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
            print("q. Quit")

            choice = input("\nEnter your choice: ")
            if choice.lower() == "q":
                break

            if choice.lower() == "q":
                break
            elif choice == "-2":
                node.refresh_objects()
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
