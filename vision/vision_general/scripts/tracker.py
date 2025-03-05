#!/usr/bin/env python3

"""
Node to track a single person and
re-id them if necessary
"""

import cv2

# import time
# from PIL import Image as PILImage
from ultralytics import YOLO

# import torch.nn as nn
# import torch
import tqdm

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from std_srvs.srv import SetBool

# from ReID import reid_model

# from ..Utils.reid_model import load_network, compare_images, extract_feature_from_img, get_structure
# from ..Utils.pose_model import check_visibility, getCenterPerson


CAMERA_TOPIC = "/zed2/zed_node/left/image_rect_color"
SET_TARGET_TOPIC = "/vision/set_tracking_target"
RESULTS_TOPIC = "/vision/tracking_results"


class SingleTracker(Node):
    def __init__(self):
        super().__init__("single_tracker")
        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.set_target_service = self.create_service(
            SetBool, SET_TARGET_TOPIC, self.set_target_callback
        )

        self.setup()
        self.create_timer(0.1, self.run)

    def setup(self):
        """Load models and initial variables"""
        self.target_set = False
        self.image = None
        pbar = tqdm.tqdm(total=1, desc="Loading models")

        self.model = YOLO("yolov8n.pt")
        pbar.update(1)

        pbar.close()
        self.get_logger().info("Single Tracker Ready")

    def image_callback(self, data):
        """Callback to receive image from camera"""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def set_target_callback(self, request, response):
        """Callback to set the target to track"""
        self.target_set = request.data
        if self.target_set:
            self.get_logger().info("Tracking enabled: Target set")
        else:
            self.get_logger().info("Tracking disabled")
        response.success = True
        return response

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def run(self):
        """Main loop to run the tracker"""
        if self.target_set:
            self.frame = self.image

            results = self.model.track(
                self.frame,
                persist=True,
                tracker="bytetrack.yaml",
                classes=0,
                verbose=False,
            )

            # Check each detection
            for out in results:
                for box in out.boxes:
                    x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]

                    # Get class name
                    class_id = box.cls[0].item()
                    label = self.model.names[class_id]

                    # Get confidence
                    prob = round(box.conf[0].item(), 2)

                    # Draw bounding box
                    cv2.rectangle(self.frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Get tracking id
                    # track_id = box.cls[0].id()

                    # Add text
                    cv2.putText(
                        self.frame,
                        f"{label} {class_id}, Prob: {prob}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 255, 0),
                        2,
                    )

            cv2.imshow("Tracking", self.frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = SingleTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
