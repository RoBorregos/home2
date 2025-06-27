#!/usr/bin/env python3

import cv2
import pathlib
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.task import Future

from frida_interfaces.action import DetectPerson
from frida_interfaces.srv import TrashDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    IMAGE_TOPIC,
    TRASH_DETECTIONS_TOPIC
)

from ament_index_python.packages import get_package_share_directory

package_share_dir = get_package_share_directory("vision_general")
YOLO_LOCATION = str(pathlib.Path(__file__).parent) + "/Utils/yolov8n.pt"

PERCENTAGE = 0.3
MAX_DEGREE = 70
AREA_PERCENTAGE_THRESHOLD = 0.01
CONF_THRESHOLD = 0.4

class trashDetection(Node):
    def __init__(self):
        super().__init__("Trash detection")
        self.bridge = CvBridge()
        
        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.find_trash_service = self.create_service(
            TrashDetection, TRASH_DETECTIONS_TOPIC, self.trash_detection_callback
        )
        self.image_publisher = self.create_publisher(
            Image, IMAGE_TOPIC, 10
        )

        self.image = None
        self.yolo_model = YOLO(YOLO_LOCATION)
        self.output_image = []
        self.check = False

        self.get_logger().info("Trash Detection Ready.")

        self.create_timer(0.1, self.publish_image)

        def image_callback(self, data):
            """Callback to receive the image from the camera."""
            # self.id = self.data
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        def trash_detection_callback(self,request,response):
            """ Callback to detect trash in the floor """
            self.get_logger().info("Executing service Trash Detection")

            if self.image is None:
                response.success = False
                self.get_logger().warn("No image received yet.")
                return response
            
            frame = self.image()
            self.output_image = frame.copy()

            self.trash = []

            self.get_trash(frame)

            is_trash, angle = self.check_trash(frame)

            if is_trash:
                response.sucess = True
                return response
            
            response.success = False
            self.get_logger().warn("No trash found")
            return response
        
        def publish_image(self):
            """Publish the image with the detections if available."""
            if len(self.output_image) != 0:
                # cv2.imshow("Receptionist Commands", self.output_image)
                # cv2.waitKey(1)
                self.image_publisher.publish(
                    self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
                )

        def detect_trash(self):
            """ Detecth objects in the floor """
            
            if self.image is None:
                self.get_logger().warn("No image received yet.")
                return

            frame = self.image
            self.output_image = frame.copy()
            width = frame.shape[1]

            results = self.yolo_model(frame, verbose=False)
            trash_count = 0

            for out in results:
                for box in out.boxes:
                    x, y, w, h = [round(i) for i in box.xywh[0].tolist()]
                    confidence = box.conf.item()
                    trash_count =+ 1

                    if (
                        confidence > CONF_THRESHOLD
                        and x >= int(width * PERCENTAGE)
                        and x <= int(width * (1 - PERCENTAGE))
                    ):
                        self.trash_found = True
                        cv2.rectangle(
                            self.output_image,
                            (int(x - w / 2), int(y - h / 2)),
                            (int(x + w / 2), int(y + h / 2)),
                            (0, 255, 0),
                            2,
                        )
                        break

                    cv2.rectangle(
                        self.output_image,
                        (int(x - w / 2), int(y - h / 2)),
                        (int(x + w / 2), int(y + h / 2)),
                    (255, 0, 0),
                    2,
                )
        
            return trash_count



def main(args=None):
    rclpy.init(args=args)
    node = trashDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()