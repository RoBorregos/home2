#!/usr/bin/env python3

import cv2
import rclpy #python interface to ros2Add commentMore actions
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from frida_constants.vision_constants import(CAMERA_TOPIC, PEOPLE_TOPIC)
from cv_bridge import CvBridge, CvBridgeError

class crowdDetection(Node):
    def __init__(self):
        print("Crowd Counting Ready")
        self.bridge = CvBridge()
        self.image.subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )
        self.detections_publisher = self.create_publisher(
            Int16, PEOPLE_TOPIC, 10
        )
        self.model = YOLO("yolov8n.pt")
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.image = None
        self.people = 0
        self.person_id = 0

        def get_people_count(self, results):
            count = 0
            for result in results:
                if result.boxes.id is not None:
                    count += len(result.boxes.id)
                    cv2.imshow("Image", self.image)
                    cv2.waitKey(1)
                    self.detections_publisher.publish(Int16(data=count))
            return count
        
        def callback(self,data):
            try:
                self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(f"Error converting image: {e}")
                return
            
            results = self.model(self.image)
            self.people = self.get_people_count(results)
            print(f"People detected: {self.people}")
def main(args=None):
    rclpy.init(args=args)
    node = crowdDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()
