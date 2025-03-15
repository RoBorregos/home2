#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from frida_interfaces.srv import AddCollisionObjects
from frida_interfaces.msg import CollisionObject
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
import copy

"""
Usage
- ros2 run frida_motion_planning add_collision_object.py --ros-args -p type:="box" -p position:="[0.5, 0.0, 0.5]" -p dimensions:="[0.2, 0.2, 0.05]"
- ros2 run frida_motion_planning add_collision_object.py --ros-args -p type:="cylinder" -p position:="[0.2, 0.0, 0.5]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p dimensions:="[0.1, 0.4]"
- ros2 run frida_motion_planning add_collision_object.py --ros-args -p type:="sphere" -p position:="[0.35, 0.35, 0.35]" -p dimensions:="[0.05]"
- ros2 run frida_motion_planning add_collision_object.py --ros-args -p type:="mesh" -p position:="[0.2, 0.0, 0.5]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p mesh_file:="package://frida_motion_planning/meshes/box.stl"
"""


def main():
    rclpy.init()

    node = Node("ex_collision_primitive")
    node.declare_parameter("type", "box")
    node.declare_parameter("position", [0.5, 0.0, 0.5])
    node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("dimensions", [0.1, 0.1, 0.1])
    node.declare_parameter("mesh_file", "")

    add_collision_object_client = node.create_client(
        AddCollisionObjects, "/manipulation/add_collision_objects"
    )
    while not add_collision_object_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")

    type = node.get_parameter("type").get_parameter_value().string_value
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    dimensions = (
        node.get_parameter("dimensions").get_parameter_value().double_array_value
    )
    mesh_file = node.get_parameter("mesh_file").get_parameter_value().string_value

    node.get_logger().info("Adding collision object...")
    node.get_logger().info(f"Sending type: {type}")
    node.get_logger().info(f"Sending position: {position}")
    node.get_logger().info(f"Sending quat_xyzw: {quat_xyzw}")
    node.get_logger().info(f"Sending dimensions: {dimensions}")
    request = AddCollisionObjects.Request()
    collision_object = CollisionObject()
    collision_object.id = "object"
    collision_object.type = type
    collision_object.pose.header = Header()
    collision_object.pose.header.frame_id = "link_base"
    collision_object.pose.header.stamp = node.get_clock().now().to_msg()
    collision_object.pose.pose.position.x = float(position[0])
    collision_object.pose.pose.position.y = float(position[1])
    collision_object.pose.pose.position.z = float(position[2])
    if collision_object.type == "box":
        collision_object.dimensions.x = float(dimensions[0])
        collision_object.dimensions.y = float(dimensions[1])
        collision_object.dimensions.z = float(dimensions[2])
    elif collision_object.type == "cylinder":
        collision_object.dimensions.x = float(dimensions[0])
        collision_object.dimensions.z = float(dimensions[1])
    else:
        collision_object.dimensions.x = float(dimensions[0])
    collision_object.pose.pose.orientation = Quaternion()
    collision_object.pose.pose.orientation.x = float(quat_xyzw[0])
    collision_object.pose.pose.orientation.y = float(quat_xyzw[1])
    collision_object.pose.pose.orientation.z = float(quat_xyzw[2])
    collision_object.pose.pose.orientation.w = float(quat_xyzw[3])
    if collision_object.type == "mesh":
        collision_object.path_to_mesh = mesh_file
    request.collision_objects.append(collision_object)

    ### SEND DUPLICATE
    new_collision_object = copy.deepcopy(collision_object)
    new_collision_object.id = "object_2"
    new_collision_object.pose.pose.position.x = (
        -new_collision_object.pose.pose.position.x
    )
    request.collision_objects.append(new_collision_object)

    future = add_collision_object_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()

    node.get_logger().info(f"Response: {response.success}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
