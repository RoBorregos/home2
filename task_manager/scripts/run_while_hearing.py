#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.srv import STT, HearMultiThread
from frida_constants.hri_constants import (
    STT_SERVICE_NAME,
)


class ServiceClientServerNode(Node):
    def __init__(self):
        super().__init__("run_while_hearing")

        self.stopped = False
        self.listen_hri = False
        self.cb_group = ReentrantCallbackGroup()

        # Create the server
        self.srv = self.create_service(
            HearMultiThread,
            "/integration/multi_stop",
            self.handle_stop,
            callback_group=self.cb_group,
        )

        # Create the client
        self.hear_service = self.create_client(STT, STT_SERVICE_NAME, callback_group=self.cb_group)

        # Wait for server to be available
        while not self.hear_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for server...")

        # Timer to send requests periodically
        self.timer = self.create_timer(3.0, self.send_request, callback_group=self.cb_group)

    def handle_stop(self, request, response):
        response.stopped = False

        if request.stop_service:
            self.listen_hri = False
            self.stopped = False

        if request.start_service:
            self.listen_hri = True
            self.stopped = False

        if self.stopped:
            response.stopped = True
            self.stopped = False

        return response

    def send_request(self):
        if self.listen_hri:
            request = STT.Request()

            future = self.hear_service.call_async(request)
            # Register done callback
            future.add_done_callback(self.response_callback)
        else:
            self.get_logger().info("Not listening hri")

    def response_callback(self, future):
        try:
            response = future.result()
            if len(response.text_heard) > 0 and "stop" in response.text_heard.lower():
                self.get_logger().info("Stop heard, updating variable")
                self.stopped = True
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main():
    rclpy.init()
    node = ServiceClientServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
