#!/usr/bin/env python3

"""
Node to handle RESTAURANT commands.
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from vision_general.utils.ros_utils import wait_for_future
import math

from frida_interfaces.srv import (
    CropQuery,
    CustomerTables,
    Customer,
)
from frida_interfaces.msg import CustomerTable
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    IMAGE_TOPIC,
    CROP_QUERY_TOPIC,
    GET_CUSTOMER_TOPIC,
    CAMERA_INFO_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CUSTOMER_TABLES_TOPIC,
)
from vision_general.utils.calculations import (
    get_depth,
    deproject_pixel_to_point,
)
from frida_interfaces.srv import YoloDetect

TABLE_CUSTOMER_DISTANCE_THRESHOLD = 1.5  # meters


class RESTAURANTCommands(Node):
    def __init__(self):
        super().__init__("restaurant_commands")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
        )

        self.image_publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)

        self.yolo_client = self.create_client(
            YoloDetect, "yolo_detect", callback_group=self.callback_group
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

        while not self.yolo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("YOLO service not available, waiting...")

        self.image = None
        self.output_image = []
        self.people = []
        self.depth_image = []
        self.depth_image_time = None
        self.imageInfo = None

        self.get_logger().info("RESTAURANT Commands Ready.")
        # self.create_timer(0.1, self.publish_image)

        self.moondream_client = self.create_client(
            CropQuery, CROP_QUERY_TOPIC, callback_group=self.callback_group
        )

    def customer_table_callback(self, request, response):
        self.get_logger().info("Received customer table request")

        # Get tables from YOLO.
        tables = self.get_detections([60])
        customer_points = self.get_customer_points()

        if not customer_points or not tables:
            self.get_logger().error("No detections found")
            response.customerTables = []
            response.success = False
            return response

        reference_header = customer_points[0].header if customer_points else None

        # Prepare one output CustomerTable per detected table.
        table_groups = []
        for table in tables:
            table_msg = CustomerTable()
            table_msg.table_point = self.build_point_stamped_from_xyz(
                table["point3d"], reference_header
            )
            table_msg.people_points = []
            table_groups.append(table_msg)

        assigned_customers = 0
        for customer in customer_points:
            customer_xyz = (
                float(customer.point.x),
                float(customer.point.y),
                float(customer.point.z),
            )

            closest_table_idx = -1
            closest_distance = float("inf")

            for idx, table in enumerate(tables):
                distance = self.euclidean_distance(table["point3d"], customer_xyz)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_table_idx = idx

            if (
                closest_table_idx >= 0
                and closest_distance <= TABLE_CUSTOMER_DISTANCE_THRESHOLD
            ):
                table_groups[closest_table_idx].people_points.append(customer)
                assigned_customers += 1

        response.customerTables = table_groups

        response.success = True

        self.get_logger().info(
            f"Associated {assigned_customers}/{len(customer_points)} customers to tables"
        )
        return response

    def build_point_stamped_from_xyz(self, xyz, reference_header=None):
        point_stamped = PointStamped()

        if reference_header is not None:
            point_stamped.header = reference_header

        point_stamped.point.x = float(xyz[0])
        point_stamped.point.y = float(xyz[1])
        point_stamped.point.z = float(xyz[2])
        return point_stamped

    def euclidean_distance(self, p1, p2):
        return math.sqrt(
            (float(p1[0]) - float(p2[0])) ** 2
            + (float(p1[1]) - float(p2[1])) ** 2
            + (float(p1[2]) - float(p2[2])) ** 2
        )

    def get_customer_points(self):
        """Get customers using the customer service."""
        req = Customer.Request()
        future = self.customer_client.call_async(req)
        future = wait_for_future(future, 15)
        result = future.result()

        if result is None or not result.found:
            self.get_logger().error("Customer detection failed")
            return []

        return result.points

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

    def get_detections(self, comp_class=None, timeout=5.0):
        """
        Obtain YOLO detections via the YOLO service.
        comp_class: int[] or None (None = detect all classes)
        """

        # Create request
        req = YoloDetect.Request()
        req.classes = comp_class if comp_class is not None else []

        # Call YOLO service
        future = self.yolo_client.call_async(req)

        # Wait for the future while spinning the node
        future = wait_for_future(future, 15)
        result = future.result()

        if result is None or not result.success:
            self.get_logger().error("YOLO detection failed")
            return []

        # Parse detections
        detections = []
        for det in result.detections:
            x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2
            conf, cls_id = det.confidence, det.class_id
            centroid = ((y1 + y2) / 2, (x1 + x2) / 2)
            depth = get_depth(self.depth_image, centroid)
            point3d = deproject_pixel_to_point(self.imageInfo, centroid, depth)
            detections.append(
                {
                    "bbox": (x1, y1, x2, y2),
                    "confidence": conf,
                    "class_id": cls_id,
                    "area": (x2 - x1) * (y2 - y1),
                    "point3d": point3d,
                }
            )

        return detections


def main(args=None):
    rclpy.init(args=args)
    node = RESTAURANTCommands()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
