#!/usr/bin/env python3
import cv2
import rclpy #python interface to ros2
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from frida_constants.vision_constants import(CAMERA_TOPIC, PEOPLE_TOPIC)
from cv_bridge import CvBridge, CvBridgeError
from frida_interfaces.srv import CountPersons
class CountPeople(Node):
    def __init__(self):        
        super().__init__("count_people")
        self.bridge = CvBridge()
        self.srv = self.create_service(CountPersons, 'count_persons', self.count_persons_callback)
        self.yolo_model = YOLO("yolov8n.pt")
        self.image = None
        self.people = 0
        self.person_id = 0

    def count_persons_callback(self, request, response):
        print("peticion recibida")
        self.image = self.bridge.imgmsg_to_cv2(request.img, "bgr8")
        results = self.yolo_model(self.image, verbose = False, classes = self.person_id)
        response.result = Int16()
        response.result.data = len(results[0].boxes)
        results_frame = results[0].plot()
        cv2.imshow("results", results_frame)
        cv2.waitKey(1)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    count_persons_srv = CountPeople()
    try:
        rclpy.spin(count_persons_srv)
    except KeyboardInterrupt:
        pass
    finally:
        count_persons_srv.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()