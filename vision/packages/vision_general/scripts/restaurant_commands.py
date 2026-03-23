#!/usr/bin/env python3

"""
Node to handle RESTAURANT commands.
"""

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from vision_general.utils.ros_utils import wait_for_future
import math

from frida_interfaces.srv import (
    CustomerTables,
    Customer,
    ObjectPoints,
)
from frida_interfaces.msg import CustomerTable, PersonList
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    IMAGE_TOPIC,
    GET_CUSTOMER_TOPIC,
    CAMERA_INFO_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CUSTOMER_TABLES_TOPIC,
    OBJECT_POINTS_TOPIC,
)
from vision_general.utils.calculations import (
    get_depth,
    deproject_pixel_to_point,
)

TABLE_CUSTOMER_DISTANCE_THRESHOLD = 1.5  # meters


class RESTAURANTCommands(Node):
    def __init__(self):
        super().__init__("restaurant_commands")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.client_debug_publisher = self.create_publisher(
            Image, "yolo_debug_image", 10
        )

        self.depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
        )

        self.image_publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)

        self.moondream_point_client = self.create_client(
            ObjectPoints, OBJECT_POINTS_TOPIC, callback_group=self.callback_group
        )

        self.customer_client = self.create_client(
            Customer, GET_CUSTOMER_TOPIC, callback_group=self.callback_group
        )

        self.customer_table_client = self.create_service(
            CustomerTables,
            CUSTOMER_TABLES_TOPIC,
            self.customer_table_callback,
            callback_group=self.callback_group,
        )

        self.image = None
        self.output_image = []
        self.people = []
        self.depth_image = []
        self.depth_image_time = None
        self.imageInfo = None

        self.get_logger().info("RESTAURANT Commands Ready.")
        # self.create_timer(0.1, self.publish_image)

    def customer_table_callback(self, request, response):
        self.get_logger().info("Received customer table request")

        # Get tables from moondream.
        tables_points2d = self.get_moondream_points("table")
        customer_people = self.get_customers()

        if not customer_people or not tables_points2d:
            self.get_logger().error("No detections found")
            response.customer_tables = []
            response.success = False
            return response

        # Prepare one output CustomerTable per detected table.
        table_groups = []
        table_pixels = []
        for table in tables_points2d:
            table_msg = CustomerTable()

            pixel_x = int(table[0] * self.imageInfo.width)
            pixel_y = int(table[1] * self.imageInfo.height)
            table_pixels.append((pixel_x, pixel_y))

            point3d = deproject_pixel_to_point(
                self.imageInfo,
                (pixel_x, pixel_y),
                get_depth(self.depth_image, (pixel_y, pixel_x)),
            )

            table_msg.table_point = self.build_point_stamped_from_xyz(point3d)
            table_msg.people = PersonList()
            table_msg.people.list = []
            table_groups.append(table_msg)

        assigned_customers = 0
        for person in customer_people:
            customer_xyz = (
                float(person.point3d.point.x),
                float(person.point3d.point.y),
                float(person.point3d.point.z),
            )

            closest_table_idx = -1
            closest_distance = float("inf")

            for idx, table in enumerate(table_groups):
                table_xyz = (
                    table.table_point.point.x,
                    table.table_point.point.y,
                    table.table_point.point.z,
                )
                distance = self.euclidean_distance(table_xyz, customer_xyz)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_table_idx = idx

            if (
                closest_table_idx >= 0
                and closest_distance <= TABLE_CUSTOMER_DISTANCE_THRESHOLD
            ):
                table_groups[closest_table_idx].people.list.append(person)
                assigned_customers += 1

        response.customer_tables = table_groups
        response.success = True

        self.publish_table_customer_image(response.customer_tables, table_pixels)
        self.get_logger().info(
            f"Associated {assigned_customers}/{len(customer_people)} customers to tables"
        )
        return response

    def publish_table_customer_image(self, table_groups, table_pixels):
        if self.image is None or self.imageInfo is None:
            return

        debug_image = self.image.copy()

        table_color = (0, 255, 0)  # green for tables
        person_color = (0, 0, 255)  # red for people

        for i, table in enumerate(table_groups):
            table_name = f"Table {i+1}"
            u, v = table_pixels[i]

            # Draw table center
            cv2.circle(debug_image, (u, v), 15, table_color, -1)
            cv2.putText(
                debug_image,
                table_name,
                (u - 20, v - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                table_color,
                2,
            )

            for person in table.people.list:
                try:
                    px, py = int(person.x), int(person.y)
                    # Draw person
                    cv2.circle(debug_image, (px, py), 10, person_color, -1)

                    # Indicate table number next to the person
                    cv2.putText(
                        debug_image,
                        table_name,
                        (px - 20, py - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        person_color,
                        2,
                    )
                except Exception:
                    pass

        self.client_debug_publisher.publish(
            self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
        )

    def build_point_stamped_from_xyz(self, xyz):
        point_stamped = PointStamped()
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.header.frame_id = "zed_left_camera_optical_frame"
        point_stamped.point.x = float(xyz[2])
        point_stamped.point.y = float(xyz[0])
        point_stamped.point.z = float(xyz[1])
        return point_stamped

    def euclidean_distance(self, p1, p2):
        return math.sqrt(
            (float(p1[0]) - float(p2[0])) ** 2
            + (float(p1[1]) - float(p2[1])) ** 2
            + (float(p1[2]) - float(p2[2])) ** 2
        )

    def get_customers(self):
        """Get customers using the customer service, returns list of Person."""
        req = Customer.Request()
        future = self.customer_client.call_async(req)
        future = wait_for_future(future, 15)
        result = future.result()

        if result is None or not result.found:
            self.get_logger().warn("Customer not detected")
            return []

        return result.people.list

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.image is None:
                return

            self.output_image = self.image.copy()

        except Exception as e:
            print(f"Error: {e}")

    def image_info_callback(self, data):
        """Callback to receive camera info"""
        self.imageInfo = data

    def depth_callback(self, data):
        """Callback to receive depth image from camera"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image = depth_image
            self.depth_image_time = data.header.stamp
        except Exception as e:
            print(f"Error: {e}")

    def publish_image(self):
        """Publish the image with the detections if available."""
        if len(self.output_image) != 0:
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

    def get_moondream_points(self, subject) -> list[tuple[float, float]]:
        """Get object points from the MoonDream service."""
        req = ObjectPoints.Request()
        req.subject = subject

        future = self.moondream_point_client.call_async(req)
        future = wait_for_future(future, 15)
        result = future.result()

        if result is None or not result.success:
            self.get_logger().error("MoonDream table point detection failed")
            return []

        points = []
        for p in result.points:
            points.append((p.x, p.y))

        return points


def main(args=None):
    rclpy.init(args=args)
    node = RESTAURANTCommands()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
