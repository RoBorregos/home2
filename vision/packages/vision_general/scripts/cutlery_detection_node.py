#!/usr/bin/env python3
"""
Node to detect cutlery objects (knife, spoons and forks).
"""

import pathlib
import rclpy
import rclpy.qos
from sensor_msgs.msg import Image
from frida_interfaces.msg import ObjectDetection, ObjectDetectionArray
from rclpy.node import Node
from cv_bridge import CvBridge
from vision_general.utils.trt_utils import load_yolo_trt
from frida_constants.vision_constants import CAMERA_TOPIC, CUTLERY_DETECTIONS_TOPIC
from ament_index_python.packages import get_package_share_directory


CONF_THRESHOLD = 0.2

# Map Spanish class names to English
SPANISH_TO_ENGLISH = {
    "cuchara": "spoon",
    "tenedor": "fork",
    "cuchillo": "knife",
}


class CutleryDetectionNode(Node):
    def __init__(self):
        super().__init__("cutlery_detection_node")
        self.get_logger().info("Cutlery detection node initialized")
        self.bridge = CvBridge()
        self.image = None

        MODELS_PATH = (
            pathlib.Path(get_package_share_directory("vision_general"))
            / "Utils"
            / "models"
        )

        cutlery_pt = str(MODELS_PATH / "cutlery.pt")
        self.cutlery_model = load_yolo_trt(cutlery_pt)
        self.get_logger().info("Cutlery model loaded")

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos
        )

        # Publisher for ObjectDetectionArray on the same topic as object detector node
        self.cutlery_detections_publisher = self.create_publisher(
            ObjectDetectionArray, CUTLERY_DETECTIONS_TOPIC, 5
        )

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.publish_cutlery_annotations()

    def detect_callback(self, req, res):
        """Run YOLO on the latest cached frame and publish ObjectDetectionArray."""
        if self.image is None:
            self.get_logger().warn("No image received yet from camera.")
            res.success = False
            res.detections = []
            self.get_logger().info(
                f"YOLO res: success={res.success}, detections={res.detections}"
            )
            return res

        results = self.cutlery_model(self.image, verbose=False, classes=[0, 1, 3])

        detections = []
        for out in results:
            for box in out.boxes:
                conf = box.conf.item()
                if conf < CONF_THRESHOLD:
                    continue
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                cls_id = int(box.cls.item())

                det = ObjectDetection()
                det.label = cls_id
                label_text = (
                    self.cutlery_model.names[cls_id]
                    if hasattr(self.cutlery_model, "names")
                    else str(cls_id)
                )
                # Convert to English if in mapping
                det.label_text = SPANISH_TO_ENGLISH.get(label_text.lower(), label_text)
                det.score = conf
                det.ymin = float(y1)
                det.xmin = float(x1)
                det.ymax = float(y2)
                det.xmax = float(x2)
                # det.point3d left empty (no 3D info from 2D cutlery)
                detections.append(det)

        msg = ObjectDetectionArray()
        msg.detections = detections
        self.cutlery_detections_publisher.publish(msg)

        res.success = True
        res.detections = detections
        self.get_logger().info(
            f"YOLO res: success={res.success}, detections={[(d.label, d.score) for d in detections]}"
        )
        return res

    def publish_cutlery_annotations(self):
        """Detect cutlery and publish ObjectDetectionArray to /vision/cutlery_detections"""
        if self.image is None:
            return

        results = self.cutlery_model(self.image, verbose=False, classes=[0, 1, 3])
        detections = []
        for res in results:
            for box in res.boxes:
                conf = box.conf.item()
                if conf < CONF_THRESHOLD:
                    continue
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                cls_id = int(box.cls.item())
                label = f"{self.cutlery_model.names[cls_id]}: {conf:.2f}"
                self.get_logger().info(f"{label} at ({x1}, {y1}), ({x2}, {y2})")
                det = ObjectDetection()
                det.label = cls_id
                label_text = (
                    self.cutlery_model.names[cls_id]
                    if hasattr(self.cutlery_model, "names")
                    else str(cls_id)
                )
                det.label_text = SPANISH_TO_ENGLISH.get(label_text.lower(), label_text)
                det.score = conf
                det.ymin = float(y1)
                det.xmin = float(x1)
                det.ymax = float(y2)
                det.xmax = float(x2)
                detections.append(det)

        msg = ObjectDetectionArray()
        msg.detections = detections
        self.cutlery_detections_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CutleryDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
