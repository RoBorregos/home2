#!/usr/bin/env python3

"""
Node to handle Storing Groceries commands.
"""

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
# from datetime import datetime

from sensor_msgs.msg import Image
from frida_interfaces.msg import ShelfDetection, ShelfArray
from frida_interfaces.srv import ShelfDetectionHandler

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    SHELF_DETECTION_TOPIC,
    IMAGE_TOPIC,
)
from Shelf_OCV import OpenCVShelfDetector


class StoringGroceriesCommands(Node):
    def __init__(self):
        super().__init__("storing_groceries_commands")
        self.bridge = CvBridge()
        self.image = None
        self.detector = OpenCVShelfDetector()  # OpenCVShelfDetector instance

        # Subscription to the camera image topic
        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        # Service to handle shelf detection
        self.detect_shelf_service = self.create_service(
            ShelfDetectionHandler, SHELF_DETECTION_TOPIC, self.detect_shelf_callback
        )

        # Publisher to send processed images with detected shelves
        self.image_publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)

        self.get_logger().info("StoringGroceriesCommands Ready.")

    # def create_shelf_detector(self, image=None):
    #     """Helper function to create and return an OpenCVShelfDetector."""
    #     if image is not None:
    #         self.detector = OpenCVShelfDetector(
    #             image
    #         )  # Create the detector with the current image
    #     return self.detector

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            if self.image is None:
                self.get_logger().warn("Received empty image.")
                return  # Return early if no valid image is received

            if self.image.size == 0:
                self.get_logger().warn("Received empty image array.")
                return  # Exit if the image is empty

            # Reset the original image before processing (in case it was previously drawn on)
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Process the image with OpenCVShelfDetector
            # detector = OpenCVShelfDetector(self.image)
            final_horizontal, filtered_vertical = self.detector.detect_shelves(
                self.image
            )

            if not final_horizontal or not filtered_vertical:
                # self.get_logger().info("No shelves or lines detected.")
                pass

            # Reset the image before drawing new lines
            self.image = self.bridge.imgmsg_to_cv2(
                data, "bgr8"
            )  # Reset image before drawing

            # Draw bounding boxes for each detected shelf
            shelf_detections = self.detector.get_shelves()
            for detection in shelf_detections:
                cv2.rectangle(
                    self.image,
                    (int(detection.bbox_.x1), int(detection.bbox_.y1)),
                    (int(detection.bbox_.x2), int(detection.bbox_.y2)),
                    (0, 255, 0),
                    2,
                )

                # Add shelf level text
                cv2.putText(
                    self.image,
                    f"Level: {detection.level_}",
                    (int(detection.bbox_.x1), int(detection.bbox_.y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            # Draw horizontal and vertical lines
            for line in final_horizontal:
                cv2.line(
                    self.image, (line[0], line[1]), (line[2], line[3]), (0, 255, 0), 2
                )

            for line in filtered_vertical:
                cv2.line(
                    self.image, (line[0], line[1]), (line[2], line[3]), (255, 0, 0), 2
                )

            # Publish and show the processed image
            self.publish_image()

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    # def save_annotated_image(self):
    #     """Save the image with annotations."""
    #     timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    #     annotated_image_path = f'./tests/shelf_detection_{timestamp}.jpg'

    #     # Create test folder if it doesn't exist
    #     if not os.path.exists('./tests'):f
    #         os.makedirs('./tests')

    #     # Save the annotated image
    #     cv2.imwrite(annotated_image_path, self.image)
    #     self.get_logger().info(f"Image saved as {annotated_image_path}")

    def publish_image(self):
        """Publish the image with the detections and show it in real-time."""
        if self.image is not None and len(self.image) != 0:
            # Show image in real-time
            cv2.imshow("Shelf Detection", self.image)
            cv2.waitKey(1)  # Wait 1 ms to update OpenCV window

            # Publish the processed image with detections
            img_msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
            self.image_publisher.publish(img_msg)

    def success(self, message):
        """Log a success message."""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def detect_shelf_callback(self, request, response):
        """Service callback to handle shelf detection."""
        self.get_logger().info("Executing service detect shelf")
        if self.image is None:
            response.success = False
            return response

        # self.detector = self.create_shelf_detector(self.image)
        self.detector.detect_shelves(self.image)

        # Get shelf detections and prepare the response
        shelf_detections = self.detector.get_shelves()
        shelf_array = ShelfArray()
        response.success = False

        for detection in shelf_detections:
            shelf_detection = ShelfDetection(
                level=detection.level_,
                ymin=detection.bbox_.y1,
                xmin=detection.bbox_.x1,
                ymax=detection.bbox_.y2,
                xmax=detection.bbox_.x2,
            )
            shelf_array.shelf_detections.append(shelf_detection)
            response.success = True
        # Save the image only when the service is called
        # self.save_annotated_image()

        if response.success is False:
            self.get_logger().warn("No shelves detected.")
            response.success = False
            return response

        self.success("Detected shelves successfully")
        response.shelf_array = shelf_array
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StoringGroceriesCommands()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
