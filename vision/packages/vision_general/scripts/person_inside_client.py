#!/usr/bin/env python3

"""
Client to request whether a person is inside a specified area based on bounding box coordinates.
"""

from rclpy.node import Node
import time

from frida_interfaces.srv import PersonInsideReq

from frida_constants.vision_constants import (
    PERSON_INSIDE_REQUEST_TOPIC,
)

class PersonInsideClient:
    def __init__(self, node: Node, callback_group, service_name=PERSON_INSIDE_REQUEST_TOPIC):
        self.node = node
        self.client = node.create_client(PersonInsideReq, service_name, callback_group=callback_group)

        self.node.get_logger().info(f'Initializing PersonInsideClient with service name: {service_name}')

        while not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().info(f'Waiting for {service_name}...')

        self.node.get_logger().info(f'Service {service_name} is now available.')

    def call(self, ymin, xmin, ymax, xmax):
        request = PersonInsideReq.Request()
        request.ymin = float(ymin)
        request.xmin = float(xmin)
        request.ymax = float(ymax)
        request.xmax = float(xmax)

        future = self.client.call_async(request)
        future = self.wait_for_future(future, 15)
        result = future.result()
        if result is None:
            self.get_logger().error("PersonInside service call failed or timed out.")
            return None
        if result.success:
            self.get_logger().info(f"PersonInside service call successful.")
            return result
        
    def wait_for_future(self, future, timeout=5):
        start_time = time.time()
        while future is None and (time.time() - start_time) < timeout:
            pass
        if future is None:
            return False
        while not future.done() and (time.time() - start_time) < timeout:
            # print("Waiting for future to complete...")
            pass

        return future
     

