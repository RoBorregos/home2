#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetDigitalIO  # Import the services interface


class XArmTGpioClient(Node):
    def __init__(self):
        super().__init__("xarm_tgpio_client")

        # Declare a parameter to control the gripper state (default is "open")
        self.declare_parameter("gripper_state", "open")

        # Create a client for the service
        self.client = self.create_client(SetDigitalIO, "/xarm/set_tgpio_digital")

        # Wait for the service
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for service /xarm/set_tgpio_digital...")

        self.get_logger().info("Service available, ready to send requests.")

    def send_request(self):
        # Read the parameter value
        gripper_state = (
            self.get_parameter("gripper_state").get_parameter_value().string_value
        )

        # Set the output value based on the gripper state
        ionum = int(0)  # Ensure it is int16
        value = int(0) if gripper_state == "open" else int(1)

        request = SetDigitalIO.Request()
        request.ionum = ionum
        request.value = value

        self.get_logger().info(
            f"Sending request: ionum={ionum}, value={value} (gripper_state={gripper_state})"
        )
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.ret == 0:
                self.get_logger().info("Operation successful on GPIO")
            else:
                self.get_logger().error("Error in operation")
        except Exception as e:
            self.get_logger().error(f"Exception in service call: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = XArmTGpioClient()

    node.send_request()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
