#!/usr/bin/env python3

"""
Node to detect if a person is inside the house.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from frida_interfaces.srv import (
    PointTransformation,
)  # ajusta los nombres
import json
import os
from math import sqrt
from ament_index_python.packages import get_package_share_directory

from frida_constants.integration_constants import (
    POINT_TRANSFORMER_TOPIC,
)

from frida_constants.vision_constants import (
    PERSON_POINT_TOPIC,
    PERSON_INSIDE_REQUEST_TOPIC,
)

###
from geometry_msgs.msg import Point
from frida_interfaces.srv import PersonInside


class PersonLocation(Node):
    def __init__(self):
        super().__init__("person_location_service_node")

        self.person_point_sub = self.create_subscription(
            Point, PERSON_POINT_TOPIC, self.person_point_callback, 10
        )
        self.latest_person_point = None

        self.tf_client = self.create_client(
            PointTransformation, POINT_TRANSFORMER_TOPIC
        )
        while not self.tf_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for PointTransformation service...")

        self.srv = self.create_service(
            PersonInside, PERSON_INSIDE_REQUEST_TOPIC, self.handle_person_inside_request
        )

        self.get_logger().info("Node PersonLocation ready.")

    def person_point_callback(self, msg: Point):
        # Save the latest person point received
        self.get_logger().info(f"Received person point: {msg}")
        self.latest_person_point = msg

    def handle_person_inside_request(self, request, response):
        if self.latest_person_point is None:
            response.success = False
            response.message = "No person point received yet."
            return response

        point_stamped = PointStamped()
        point_stamped.header.frame_id = "zed_camera_link"
        point_stamped.point = self.latest_person_point

        # Transform the point to the 'map' frame
        req = PointTransformation.Request()
        req.point = point_stamped
        req.target_frame = "map"

        future = self.tf_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if not future.done() or not future.result().success:
            response.success = False
            response.error_message = "Error transforming point to map frame."
            return response

        transformed_point = future.result().transformed_point.point

        area, subareas = self.get_location_from_pose(
            transformed_point.x, transformed_point.y
        )

        if area is None:
            response.success = False
            response.message = "Person is outside the house."
            return response

        response.success = True
        response.area = area
        response.subareas = subareas
        response.message = f"Person is in {area}."

        return response

    def get_location_from_pose(self, x, y):
        path = os.path.join(
            get_package_share_directory("frida_constants"), "map_areas/areas.json"
        )
        with open(path, "r") as file:
            areas = json.load(file)

        for area_name, data in areas.items():
            polygon = data.get("polygon", [])
            if self.is_inside(x, y, polygon):
                subareas = []
                for place, coords in data.items():
                    if place == "polygon":
                        continue
                    dist = sqrt((coords[0] - x) ** 2 + (coords[1] - y) ** 2)
                    subareas.append((place, dist))
                subareas.sort(key=lambda t: t[1])
                return area_name, [s[0] for s in subareas]

        return None, []

    def is_inside(self, x, y, polygon):
        inside = False
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]
            if (y1 > y) != (y2 > y):
                xinters = (y - y1) * (x2 - x1) / (y2 - y1 + 1e-10) + x1
                if x < xinters:
                    inside = not inside
        return inside


def main(args=None):
    rclpy.init(args=args)
    node = PersonLocation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
