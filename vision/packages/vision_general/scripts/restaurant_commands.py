#!/usr/bin/env python3

"""
Node to handle RESTAURANT commands.
"""

import math
import os
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge

from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from frida_constants.vision_constants import (
    CAMERA_INFO_TOPIC,
    CAMERA_TOPIC,
    CUSTOMER_TABLES_TOPIC,
    DEPTH_IMAGE_TOPIC,
    GET_CUSTOMER_TOPIC,
    OBJECT_POINTS_TOPIC,
    RESTAURANT_TABLES_TOPIC,
    CAMERA_FRAME,
)
from frida_interfaces.msg import CustomerTable, PersonList
from frida_interfaces.srv import Customer, CustomerTables, ObjectPoints

from builtin_interfaces.msg import Time
from vision_general.utils.calculations import point2d_to_ros_point_stamped
from vision_general.utils.ros_utils import wait_for_future

TABLE_CUSTOMER_DISTANCE_THRESHOLD = 1.5  # meters


class RESTAURANTCommands(Node):
    def __init__(self):
        super().__init__("restaurant_commands")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image = None
        self.depth_image = None
        self.imageInfo = None
        # Last annotated tables image — republished on a timer because the
        # display renders an MJPEG stream (web_video_server): a single-shot
        # publish is lost if the stream (re)subscribes after the scan.
        self.last_debug_image = None

        # Scan images saved here (repo is volume-mounted at /workspace/src).
        self.declare_parameter("save_dir", "/workspace/src/restaurant_runs")
        self.save_dir = self.get_parameter("save_dir").value
        try:
            os.makedirs(self.save_dir, exist_ok=True)
        except OSError as e:
            self.get_logger().warn(f"Cannot create save dir {self.save_dir}: {e}")
            self.save_dir = None

        self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, 10)
        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
        )

        self.client_debug_publisher = self.create_publisher(
            Image, RESTAURANT_TABLES_TOPIC, 10
        )
        self.create_timer(0.1, self.republish_debug_image)

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

        self.get_logger().info("RESTAURANT Commands Ready.")

    def customer_table_callback(self, request, response):
        self.get_logger().info("Received customer table request")

        # An exception in this callback would drop the response and stall the
        # task manager for its whole client timeout — refuse early instead.
        if self.image is None or self.depth_image is None or self.imageInfo is None:
            self.get_logger().warn("Camera image/depth/info not available yet")
            response.customer_tables = []
            response.success = False
            return response

        tables_points2d = self.get_moondream_points("table")
        customer_people = self.get_customers()

        if not tables_points2d:
            self.get_logger().error("No tables detected")
            response.customer_tables = []
            response.success = False
            return response

        table_groups = []
        table_pixels = []

        for raw_point2d in tables_points2d:
            table_msg = CustomerTable()
            point2d = (
                int(raw_point2d[0] * self.imageInfo.width),
                int(raw_point2d[1] * self.imageInfo.height),
            )

            table_msg.table_point = point2d_to_ros_point_stamped(
                self.imageInfo,
                self.depth_image,
                point2d,
                CAMERA_FRAME,
                Time(sec=0, nanosec=0),
            )
            # A moondream point on a table edge can land on a bad depth pixel;
            # a NaN/zero 3D point would become a garbage nav goal downstream.
            p = table_msg.table_point.point
            depth_norm = math.sqrt(p.x**2 + p.y**2 + p.z**2)
            if not math.isfinite(depth_norm) or depth_norm < 0.1:
                self.get_logger().warn("Skipping table point with invalid depth")
                continue

            table_pixels.append(point2d)
            table_msg.people = PersonList()
            table_msg.people.list = []
            table_groups.append(table_msg)

        assigned_customers = 0
        for person in customer_people or []:
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
                distance = math.dist(table_xyz, customer_xyz)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_table_idx = idx

            if (
                closest_table_idx >= 0
                and closest_distance <= TABLE_CUSTOMER_DISTANCE_THRESHOLD
            ):
                table_groups[closest_table_idx].people.list.append(person)
                assigned_customers += 1

        if not table_groups:
            self.get_logger().error("All table points had invalid depth")
            response.customer_tables = []
            response.success = False
            return response

        response.customer_tables = table_groups
        response.success = True

        self.publish_table_customer_image(response.customer_tables, table_pixels)
        self.get_logger().info(f"Tables detected: {len(table_groups)}")
        self.get_logger().info(
            f"Associated {assigned_customers}/{len(customer_people)} customers to tables"
        )
        return response

    def republish_debug_image(self):
        """Keep the annotated tables image alive on the topic for the MJPEG display."""
        if self.last_debug_image is not None:
            self.client_debug_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.last_debug_image, "bgr8")
            )

    def publish_table_customer_image(self, table_groups, table_pixels):
        if self.image is None or self.imageInfo is None:
            return
        self.get_logger().info("Publishing table and customer debug image")

        debug_image = self.image.copy()
        for i, (table, (u, v)) in enumerate(zip(table_groups, table_pixels)):
            table_name = f"Table {i+1}"

            # Draw table center (Green)
            cv2.circle(debug_image, (u, v), 15, (0, 255, 0), -1)
            cv2.putText(
                debug_image,
                table_name,
                (u - 20, v - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            for person in table.people.list:
                try:
                    px, py = int(person.x), int(person.y)
                    # Draw person (Red)
                    cv2.circle(debug_image, (px, py), 10, (0, 0, 255), -1)
                    cv2.putText(
                        debug_image,
                        table_name,
                        (px - 20, py - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2,
                    )
                except Exception:
                    pass

        self.last_debug_image = debug_image
        self.client_debug_publisher.publish(
            self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
        )
        if self.save_dir is not None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            try:
                cv2.imwrite(
                    os.path.join(self.save_dir, f"tables_{stamp}.jpg"), debug_image
                )
            except Exception as e:
                self.get_logger().warn(f"Could not save scan image: {e}")

    def get_customers(self):
        """Get customers using the customer service, returns list of Person."""
        req = Customer.Request()
        req.include_non_waving = (
            True  # table scan: map seated customers, not just hand-raisers
        )
        future = self.customer_client.call_async(req)
        # customer_node may moondream-check the sitting pose of several people
        # (~5 s each) — the budget here must cover that worst case.
        future = wait_for_future(future, 25)

        if future is False or not future.done():
            self.get_logger().warn("Customer service call timed out or failed")
            return []

        result = future.result()

        if result is None or not result.found:
            self.get_logger().warn("Customer not detected")
            return []

        return result.people.list

    def get_moondream_points(self, subject) -> list[tuple[float, float]]:
        """Get object points from the MoonDream service."""
        req = ObjectPoints.Request()
        req.subject = subject

        future = self.moondream_point_client.call_async(req)
        future = wait_for_future(future, 20)

        if future is False or not future.done():
            self.get_logger().error("MoonDream service call timed out or failed")
            return []

        result = future.result()

        if result is None or not result.success:
            self.get_logger().error("MoonDream table point detection failed")
            return []

        return [(p.x, p.y) for p in result.points]

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def image_info_callback(self, data):
        self.imageInfo = data

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
            self.depth_image = None


def main(args=None):
    rclpy.init(args=args)
    node = RESTAURANTCommands()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
