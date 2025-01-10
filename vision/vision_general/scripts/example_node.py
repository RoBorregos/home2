#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionServer
from std_srvs.srv import SetBool
from frida_interfaces.action import DetectPerson

# Topics
SUBSCRIBER_TOPIC = "/example/subscriber_topic"
PUBLISHER_TOPIC = "/example/publisher_topic"
SERVICE_TOPIC = "/example/service_topic"
ACTION_SERVICE_TOPIC = "/example/action_service_topic"


class ExampleNode(Node):
    def __init__(self):
        """Initialize the node"""
        super().__init__("example_node")

        # Create Subscriber
        self.example_subscriber = self.create_subscription(
            String, SUBSCRIBER_TOPIC, self.subscriber_callback, 10
        )

        # Create Publisher
        self.example_publisher = self.create_publisher(String, PUBLISHER_TOPIC, 10)

        # Create Timer (This will call a function every 1.0 sec)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Create Service
        self.example_service = self.create_service(
            SetBool, SERVICE_TOPIC, self.service_callback
        )

        # # Create Action Server
        self.example_action_server = ActionServer(
            self, DetectPerson, ACTION_SERVICE_TOPIC, self.action_callback
        )

        # Use logger to print messages (this can also be .warn or .error)
        self.get_logger().info("Example node ready.")

    def subscriber_callback(self, msg):
        """Print message data each time a message is received"""
        self.get_logger().info("I heard: " + msg.data)

    def timer_callback(self):
        """Publish a message"""
        # Create message and set data
        msg = String()
        msg.data = "Hello World"

        # Publish message
        self.example_publisher.publish(msg)

    def service_callback(self, request, response):
        """Print request data and return response"""

        # Get the request data
        req = request.data

        # Print
        if req:
            self.get_logger().info("Service called with request: True")
        else:
            self.get_logger().info("Service called with request: False")

        # Set the response data
        response.message = "Result msg"
        response.success = True

        return response

    def action_callback(self, goal_handle):
        """Print goal data, publish feedback and return result"""

        self.goal_handle = goal_handle
        request = goal_handle.request
        self.get_logger().info(f"Goal received: {request}")

        for i in range(1, 6):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return DetectPerson.Result()

            feedback_msg = DetectPerson.Feedback()
            feedback_msg.feedback = str(i)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Feedback: {i}")
            time.sleep(1)

        result = DetectPerson.Result()
        result.success = True
        goal_handle.succeed()
        return result


def main(args=None):
    """Main function to initialize the node and spin it"""
    rclpy.init(args=args)
    node = ExampleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
