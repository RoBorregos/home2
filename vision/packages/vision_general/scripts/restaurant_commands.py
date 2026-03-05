#!/usr/bin/env python3

"""
Node to handle GPSR commands.
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_general.utils.ros_utils import wait_for_future
import os
import json

from frida_interfaces.srv import (
    CropQuery,
    CustomerTable,
)

from ament_index_python.packages import get_package_share_directory

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    IMAGE_TOPIC,
    CROP_QUERY_TOPIC,
)

from frida_constants.vision_enums import Poses, Gestures, DetectBy

from frida_interfaces.srv import YoloDetect

from pose_detection import PoseDetection

package_share_dir = get_package_share_directory("vision_general")

constants = get_package_share_directory("frida_constants")
file_path = os.path.join(constants, "map_areas/areas.json")


class GPSRCommands(Node):
    def __init__(self):
        super().__init__("gpsr_commands")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )


        self.image_publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)

        self.yolo_client = self.create_client(
            YoloDetect, "yolo_detect", callback_group=self.callback_group
        )
        
        self.customer_table_client = self.create_service(
            CustomerTable, 
            "customer_table", 
            self.customer_table_callback, 
            callback_group=self.callback_group
        )

        while not self.yolo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("YOLO service not available, waiting...")

        self.image = None
        self.pose_detection = PoseDetection()
        self.output_image = []
        self.people = []

        self.get_logger().info("RESTAURANT Commands Ready.")
        # self.create_timer(0.1, self.publish_image)

        self.moondream_client = self.create_client(
            CropQuery, CROP_QUERY_TOPIC, callback_group=self.callback_group
        )

        # Load areas from the JSON file
        with open(file_path, "r") as file:
            self.areas = json.load(file)

    def customer_table_callback(self, request, response):
        self.get_logger().info("Received customer table request")

        # Get detections from YOLO persons and tables
        detections = self.get_detections([0,60])

        persons = [det for det in detections if det["class_id"] == 0]
        tables = [det for det in detections if det["class_id"] == 60]

        if not persons or not tables:
            self.get_logger().error("No detections found")
            response.success = False
            return response

        # Process detections to find customers and their locations
        customers = []
        for det in detections:
            if det["class_id"] == 0:  # Assuming class_id 0 corresponds to 'person'
                customers.append(det)

        response.number_of_customers = len(customers)
        response.people.people = customers
        response.success = True

        self.get_logger().info(f"Found {len(customers)} customers at the table")
        return response

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.image is None:
                return

            self.output_image = self.image.copy()

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
            detections.append(
                {
                    "bbox": (x1, y1, x2, y2),
                    "confidence": conf,
                    "class_id": cls_id,
                    "area": (x2 - x1) * (y2 - y1),
                }
            )

        return detections

def main(args=None):
    rclpy.init(args=args)
    node = GPSRCommands()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
