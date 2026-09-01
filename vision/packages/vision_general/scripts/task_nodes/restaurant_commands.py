#!/usr/bin/env python3

"""
Node to handle RESTAURANT commands.
"""

import math
import os
from datetime import datetime

import cv2
import rclpy

from sensor_msgs.msg import Image

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    CUSTOMER_TABLES_TOPIC,
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
from vision_runtime import VisionRuntime, spin

TABLE_CUSTOMER_DISTANCE_THRESHOLD = 1.5  # meters


class RESTAURANTCommands(VisionRuntime):
    def __init__(self):
        super().__init__(
            "restaurant_commands",
            image_topic=CAMERA_TOPIC,
            use_depth=True,
            use_camera_info=True,
        )
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
        if self.image is None or self.depth_image is None or self.camera_info is None:
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
                int(raw_point2d[0] * self.camera_info.width),
                int(raw_point2d[1] * self.camera_info.height),
            )

            table_msg.table_point = point2d_to_ros_point_stamped(
                self.camera_info,
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
        if self.image is None or self.camera_info is None:
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
                    # Draw person bbox (Red); fall back to the center dot for
                    # producers that don't fill the bbox fields.
                    if person.x2 > person.x1 and person.y2 > person.y1:
                        cv2.rectangle(
                            debug_image,
                            (int(person.x1), int(person.y1)),
                            (int(person.x2), int(person.y2)),
                            (0, 0, 255),
                            2,
                        )
                        label_pos = (int(person.x1), max(int(person.y1) - 8, 20))
                    else:
                        px, py = int(person.x), int(person.y)
                        cv2.circle(debug_image, (px, py), 10, (0, 0, 255), -1)
                        label_pos = (px - 20, py - 20)
                    cv2.putText(
                        debug_image,
                        table_name,
                        label_pos,
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


def main(args=None):
    rclpy.init(args=args)
    spin(RESTAURANTCommands(), threads=8)


if __name__ == "__main__":
    main()
