#!/usr/bin/env python3

"""
--On boarding task--
Node to count how many people 
are in front of the camera

PROBLEMS: No dockerfile for gpu support founded
"""

import pathlib
from ultralytics import YOLO
import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from frida_interfaces.vision.srv import ( 
    CountBy,
    CountSpecifiedObjects
) 
from frida_constants.vision_constants import (
    CAMARA_TOPIC,
    IMAGE_TOPIC,
    COUNT_PEOPLE_ONBOARDING_TOPIC,
    COUNT_OBJECT_ONBOARDING_TOPIC
)

YOLO_LOCATION = str(pathlib.Path(__file__).parent) + "/Utils/yolov8n.pt"
CONF_THRESHOLD = 0.5

class CameraDetections(Node):
    def __init__(self):
        super().__init__("how_many_people")

        self.image = None
        self.yolo_model = YOLO(YOLO_LOCATION)
        self.output_image = []
        self.bridge = CvBridge()

        self.camara_view = self.create_subscription(
            Image, CAMARA_TOPIC, self.image_callback, 10
        )
    
        self.how_many_people_service = self.create_service(
            CountBy, COUNT_PEOPLE_ONBOARDING_TOPIC, self.count_people_in_camara
        )
        
        self.count_specific_classes_service = self.create_service(
            CountSpecifiedObjects, COUNT_OBJECT_ONBOARDING_TOPIC, self.count_objects_in_camara
        )
    
    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        

    def detections(self, frame, comp_class : str) -> int:
        """Obtain YOLO detections for specified object."""
        results = self.yolo_model(frame, verbose=False) #comp_class is  list because of yolo's capacity to detect multiple different objects at the same time
        object_count = 0

        for res in results:
            for box in res.boxes:
                confidence = box.conf.item()
                    
                if confidence > CONF_THRESHOLD:
                    class_id = box.cls[0].item()
                    label = self.yolo_model.names[class_id]
                    if label == comp_class:
                        object_count += 1
        
        return object_count

                    
    def count_objects_in_camara(self, req, res) -> int:
        """
        Callback to count how many 
        instances of a user specified object
        exists in front of the camara
        """
        res.count = 0

        self.get_logger().info("Counting your selected classes instances in camara...")
        if self.image is None:
            return res
        
        frame = self.image
        class_name = req.object_name

        #identify objects in camara based on user input classes
        object_detections = self.detections(frame, class_name)

        if object_detections > 0:
            res.count = object_detections
            self.get_logger().info(f"{object_detections} instances of {class_name} class")
        else:
            self.get_logger().info(f"No instances of {class_name} class")

        return res
            
    def count_people_in_camara(self, req, res):
        """
        Callback to count how many 
        people are in front of the camara
        """
        self.get_logger().info("Counting people in camara...")
        
        if self.image is None:
            res.success = False
            res.count = 0
            self.get_logger().info("No image detected")
            return res
        
        frame = self.image

        people_in_camara = self.detections(frame, "person")

        if people_in_camara > 0:
            res.success = True
            res.count = people_in_camara
            self.get_logger().info(f"People in camara: {people_in_camara}")
        else:
            res.count = 0
            self.get_logger().info("No people detected")
        
        return res
    
def main(args=None):
    rclpy.init(args=args)
    node = CameraDetections()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()




