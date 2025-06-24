#!/usr/bin/env python3
import cv2
import rclpy #python interface to ros2
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from frida_constants.vision_constants import(CAMERA_TOPIC, PEOPLE_TOPIC)
from cv_bridge import CvBridge, CvBridgeError

class CountPeople(Node):
    def __init__(self):        
        super().__init__("count_people")
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, CAMERA_TOPIC, self.callback, 10)
        self.detection_publisher = self.create_publisher(Int16, PEOPLE_TOPIC, 10)
        self.yolo_model = YOLO("yolov8n.pt")
        self.timer = self.create_timer(1, self.get_people)
        self.image = None
        self.people = 0
        self.person_id = 0

    def get_people(self):
        if self.image is not None:
            results = self.yolo_model(self.image, verbose = False, classes = self.person_id)
            total_people = Int16()
            total_people.data = len(results[0].boxes)
            results_frame = results[0].plot()
            cv2.imshow("results", results_frame)
            cv2.waitKey(1)
            self.detection_publisher.publish(total_people)
        else:
            print("waiting for image")
    
    def callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)
    node = CountPeople()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()