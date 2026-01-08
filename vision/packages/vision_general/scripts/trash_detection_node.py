#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from frida_interfaces.srv import DetectionHandler, SetDetectorClasses
from frida_interfaces.msg import ObjectDetectionArray, ObjectDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    ZERO_SHOT_DETECTIONS_TOPIC,
    SET_DETECTOR_CLASSES_SERVICE,
)

class TrashDetectionNode(Node):
    def __init__(self):
        super().__init__('trash_detection_node')
        
        self.bridge = CvBridge()
        self.current_image = None
        self.latest_detections = ObjectDetectionArray()
        
        self.image_sub = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )
        
        self.detections_sub = self.create_subscription(
            ObjectDetectionArray, 
            ZERO_SHOT_DETECTIONS_TOPIC, 
            self.detections_callback, 
            10
        )
        
        self.detection_service = self.create_service(
            DetectionHandler, 
            '/vision/zero_shot_detection',
            self.detect_objects_callback
        )
        
        self.set_classes_client = self.create_client(
            SetDetectorClasses,
            SET_DETECTOR_CLASSES_SERVICE
        )
        
        self.get_logger().info("Trash Detection Node initialized.")
    
    def image_callback(self, msg):
        """
        Update current image
        """
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def detections_callback(self, msg):
        """
        Store latest yolo-e detections
        """
        self.latest_detections = msg
        
    def detect_objects_callback(self, request, response):
        """
        Handle detection request using yolo-e zero-shot detector
        """
        try:
            if hasattr(request, 'text_queries') and len(request.text_queries) > 0:
                classes_to_detect = request.text_queries
                self._set_yoloe_classes(classes_to_detect)
                self.get_logger().info(f"yolo-e classes: {classes_to_detect}")
                
                import time
                time.sleep(0.5)
            
            response.detections = []
            
            if self.current_image is not None:
                h, w = self.current_image.shape[:2]
                
                for detection in self.latest_detections.detections:
                    obj_detection = ObjectDetection()
                    obj_detection.label_text = detection.label
                    obj_detection.score = detection.confidence
                    
                    # Convert normalized coordinates to pixels
                    obj_detection.xmin = max(0, int(detection.bbox.x1 * w))
                    obj_detection.ymin = max(0, int(detection.bbox.y1 * h))
                    obj_detection.xmax = min(w, int(detection.bbox.x2 * w))
                    obj_detection.ymax = min(h, int(detection.bbox.y2 * h))
                    
                    response.detections.append(obj_detection)
            
            response.success = True
            response.message = f"Detected {len(response.detections)} objects"
            
            num_detections = len(response.detections)
            self.get_logger().info(f"{num_detections} detections")
            
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            response.success = False
            response.message = str(e)
        
        return response
    
    def _set_yoloe_classes(self, classes):
        """
        Set classes for YOLO-E detector
        """
        if not self.set_classes_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("set_classes service not available")
            return False
        
        request = SetDetectorClasses.Request()
        request.classes = classes
        
        future = self.set_classes_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            return True
        else:
            self.get_logger().warn("Failed to set classes")
            return False

def main():
    rclpy.init()
    node = TrashDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
