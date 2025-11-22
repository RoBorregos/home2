#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from frida_interfaces.action import ManipulationAction
from frida_interfaces.msg import ManipulationTask
import time
from geometry_msgs.msg import PointStamped, PoseStamped
from frida_interfaces.msg import ObjectDetectionArray
from frida_constants.vision_constants import (
    DETECTIONS_TOPIC,
)
from frida_constants.manipulation_constants import (
    MANIPULATION_ACTION_SERVER,
)
import json


class KeyboardInput(Node):
    def __init__(self):
        super().__init__("keyboard_input")

        callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self._action_client = ActionClient(
            self,
            ManipulationAction,
            MANIPULATION_ACTION_SERVER,
            callback_group=callback_group,
        )
        self.objects = []  # Example list of objects

        qos = rclpy.qos.QoSProfile(
            depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        )

        # Add subscriber for objects
        self.objects_subscription = self.create_subscription(
            ObjectDetectionArray,
            DETECTIONS_TOPIC,
            self.objects_callback,
            qos,
            callback_group=callback_group,
        )

        self.clicked_point = None
        self.clicked_point_subscription = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            qos,
            callback_group=callback_group,
        )

    def clicked_point_callback(self, msg):
        self.clicked_point = msg
        self.get_logger().info(
            f"Clicked point received: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}"
        )

    def objects_callback(self, msg):
        # Assuming msg contains a list of object names
        self.objects = []
        for detection in msg.detections:
            if detection.label_text not in self.objects:
                self.objects.append(detection.label_text)

    def send_pick_request(self, object_name):
        self.get_logger().warning(f"Sending pick request for: {object_name}")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server in pick not available!")
            return

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PICK
        goal_msg.pick_params.object_name = object_name

        self.get_logger().info(f"Sending pick request for: {object_name}")
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(lambda f: self.goal_response_callback(f, object_name))
        self.get_logger().info(f"Pick request for {object_name} sent")

    def send_place_request(
        self,
        is_shelf=False,
        table_height=None,
        table_height_tolerance=None,
        close_to="",
        special_request_position=None,
        special_request_object=None,
    ):
        self.get_logger().warning("Sending place request")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server in place not available!")
            return

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PLACE
        goal_msg.place_params.is_shelf = is_shelf
        goal_msg.place_params.close_to = close_to
        goal_msg.scan_environment = is_shelf
        if table_height is not None and table_height_tolerance is not None:
            goal_msg.place_params.table_height = table_height
            goal_msg.place_params.table_height_tolerance = table_height_tolerance
        if special_request_position is not None and special_request_object is not None:
            special_request_msg = {
                "request": "close_by",
                "object": special_request_object,
                "position": special_request_position,
            }
            special_request_msg = json.dumps(special_request_msg)
            goal_msg.place_params.special_request = special_request_msg
        self.get_logger().info("Sending place request")
        self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.get_logger().info("Place request sent")

    def send_pour_request(self, object_name, bowl_name):
        self.get_logger().warning(f"Sending pour request for: {object_name}")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server in pour request not available!")
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

    def send_place_on_clicked_point_request(self):
        self.get_logger().warning("Sending place on point request")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server in place not available!")
            return

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PLACE
        goal_msg.place_params.forced_pose = PoseStamped()

        goal_msg.place_params.forced_pose.header.frame_id = (
            self.clicked_point.header.frame_id
        )
        goal_msg.place_params.forced_pose.pose.position.x = self.clicked_point.point.x
        goal_msg.place_params.forced_pose.pose.position.y = self.clicked_point.point.y
        goal_msg.place_params.forced_pose.pose.position.z = self.clicked_point.point.z

        self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.get_logger().info("Place request sent")

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback}")

    def refresh_objects(self):
        self.get_logger().info("Refreshing objects list...")
        # Spin for 1 second to receive messages
        start_time = time.time()
        while time.time() - start_time < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Objects list refreshed.")

    def goal_response_callback(self, future, object_name):
        try:
            response = future.result()
            if response.accepted:
                self.get_logger().info(f"Goal accepted for {object_name}")
            else:
                self.get_logger().error(f"Goal rejected for {object_name}")
        except Exception as e:
            self.get_logger().error(f"Exception while sending goal: {e}")


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
            print("-7. Place on clicked point")
            print("-8. Place closeto")
            print("-9. Special Request Place")
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

            elif choice == "-7":
                print("Use rviz to click a point on the robot's base.")
                start_time = time.time()
                node.clicked_point = None
                while node.clicked_point is None and time.time() - start_time < 10:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if node.clicked_point is not None:
                    node.send_place_on_clicked_point_request()
                else:
                    print("No point clicked within the time limit. Please try again.")

            elif choice == "-8":
                close_to = input("Close to object: ")
                try:
                    node.get_logger().info(f"Sending place request close to {close_to}")
                    node.send_place_request(
                        is_shelf=True,
                        table_height=0.8,
                        table_height_tolerance=0.2,
                        close_to=close_to,
                    )
                except ValueError:
                    print("Invalid input.")
            elif choice == "-9":
                print("Special Request Place")
                print("Options: close, front, back, left, right")
                special_request_position = (
                    input("Enter your special request: ").strip().lower()
                )
                if special_request_position not in [
                    "close",
                    "front",
                    "back",
                    "left",
                    "right",
                ]:
                    print("Invalid special request. Please try again.")
                    return

                special_request_object = (
                    input("Enter the object for the special request: ").strip().lower()
                )

                node.send_place_request(
                    special_request_position=special_request_position,
                    special_request_object=special_request_object,
                )

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
