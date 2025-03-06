#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from frida_interfaces.srv import RemoveCollisionObject

"""
Usage
- ros2 run frida_motion_planning add_collision_object.py --ros-args -p id:="object"
"""


def main():
    rclpy.init()

    node = Node("ex_collision_primitive")
    node.declare_parameter("id", "object")

    remove_collision_object_client = node.create_client(
        RemoveCollisionObject, "/manipulation/remove_collision_object"
    )
    while not remove_collision_object_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")

    id = node.get_parameter("id").get_parameter_value().string_value

    request = RemoveCollisionObject.Request()
    request.id = id

    print(f"Sending id: {id}")

    future = remove_collision_object_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()

    print(f"Response: {response.success}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
