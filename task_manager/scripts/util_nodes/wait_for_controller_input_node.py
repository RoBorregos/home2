#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Joy
from rclpy.callback_groups import ReentrantCallbackGroup

from frida_interfaces.srv import (
    WaitForControllerInput,
)  # Replace with the actual service definition

import time
import threading

BUTTON_MAP = {
    "square": 0,
    "x": 1,
    "circle": 2,
    "triangle": 3,
    "l1": 4,
    "r1": 5,
    "l2": 6,
    "r2": 7,
    "share": 8,
    "options": 9,
    "ps": 12,
    "l3": 10,
    "r3": 11,
    "touchpad": 13,
}


class WaitForControllerInputNode(Node):
    def __init__(self):
        super().__init__("wait_for_controller_input_node")
        self.callback_group = ReentrantCallbackGroup()
        # Replace SetBool with WaitForControllerInput after generating the service
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)

        self.srv = self.create_service(
            WaitForControllerInput,  # Replace with WaitForControllerInput
            "wait_for_controller_input",
            self.wait_for_input_callback,
            callback_group=self.callback_group,
        )
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, qos)
        self.last_joy_msg = None
        self.joy_lock = threading.Lock()
        self.get_logger().info("WaitForControllerInput service ready.")

    def joy_callback(self, msg):
        with self.joy_lock:
            self.last_joy_msg = msg
            self.get_logger().debug(f"Received Joy message: {msg}")
            for i, button in enumerate(msg.buttons):
                if button and i in BUTTON_MAP.values():
                    button_name = list(BUTTON_MAP.keys())[list(BUTTON_MAP.values()).index(i)]
                    self.get_logger().info(f"Button '{button_name}' pressed.")

    def wait_for_input_callback(self, request, response):
        button = request.button
        timeout = request.timeout

        button = button.lower()
        if button not in BUTTON_MAP:
            self.get_logger().error(f"Button '{button}' not recognized.")
            response.success = False
            return response

        button_idx = BUTTON_MAP[button]
        self.get_logger().info(
            f"Waiting for button '{button}' (index {button_idx}) for {timeout} seconds."
        )

        start_time = self.get_clock().now()
        pressed = False

        while (self.get_clock().now() - start_time) < Duration(seconds=timeout):
            time.sleep(0.1)
            with self.joy_lock:
                if self.last_joy_msg and len(self.last_joy_msg.buttons) > button_idx:
                    if self.last_joy_msg.buttons[button_idx]:
                        pressed = True
                        break

        response.success = pressed
        if pressed:
            self.get_logger().info(f"Button '{button}' pressed!")
        else:
            self.get_logger().info(f"Timeout waiting for button '{button}'.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaitForControllerInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
