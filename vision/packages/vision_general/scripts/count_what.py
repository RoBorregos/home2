#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from std_msgs.msg import Int16, String
from std_srvs.srv import SetBool
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    PEOPLE_DETECTIONS_TOPIC,
    WHAT_DETECTIONS_TOPIC,
)
from cv_bridge import CvBridge
from frida_interfaces.srv import CountWhat, CountPeople


class CountWhat(Node):
    def __init__(self):
        super().init("how_many_what")

        self.yolo_model = YOLO("yolov8n.pt")
        

        self.what_publisher = self.create_publisher(
            String, WHAT_DETECTIONS_TOPIC, 10
        )

        self.people_publisher = self.create_publisher(
            Int16, PEOPLE_DETECTIONS_TOPIC, 10
        )

        self.people_service = self.create_service(
            SetBool, PEOPLE_DETECTIONS_TOPIC, self.peopleCounting()
        )

        print("Do you want detect an object or people?")
        print("1: Object    2: People")
        what = int(input(""))
        if what == 1:
            people_callback()
        elif what == 2:
            whatObject
        else:
            print("Try again")

        def whatObject(self, frame, object: int):
            class_dict = {
                0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 4: "airplane", 5: "bus",
                6: "train", 7: "truck", 8: "boat", 9: "traffic light", 10: "fire hydrant",
                11: "stop sign", 12: "parking meter", 13: "bench", 14: "bird", 15: "cat",
                16: "dog", 17: "horse", 18: "sheep", 19: "cow", 20: "elephant", 21: "bear",
                22: "zebra", 23: "giraffe", 24: "backpack", 25: "umbrella", 26: "handbag",
                27: "tie", 28: "suitcase", 29: "frisbee", 30: "skis", 31: "snowboard",
                32: "sports ball", 33: "kite", 34: "baseball bat", 35: "baseball glove",
                36: "skateboard", 37: "surfboard", 38: "tennis racket", 39: "bottle",
                40: "wine glass", 41: "cup", 42: "fork", 43: "knife", 44: "spoon", 45: "bowl",
                46: "banana", 47: "apple", 48: "sandwich", 49: "orange", 50: "brocolli",
                51: "carrot", 52: "hot dog", 53: "pizza", 54: "donut", 55: "cake", 56: "chair",
                57: "couch", 58: "potted plant", 59: "bed", 60: "dining table", 61: "toilet",
                62: "tv", 63: "laptop", 64: "mouse", 65: "remote", 66: "keyboard", 67: "cell phone",
                68: "microwave", 69: "oven", 70: "toaster", 71: "sink", 72: "refrigerator",
                73: "book", 74: "clock", 75: "vase", 76: "scissors", 77: "teddy bear",
                78: "hair drier", 79: "toothbrush"
            }

            print("Available objects to detect:")
            print("\n".join(f"{k}: {v}" for k, v in class_dict.items()))

            object = str(input("Insert the name of the object to detect: "))
            results = self.yolo_model(frame, classes=[object])
            counter = 0
            
            for res in results:
                for box in res.boxes:
                    id = box.cls[0].item()
                    if id == object:
                        counter += 1
            return counter
        
        def get_people_count(results):
            count = 0
            for result in results:
                if result.boxes.cls is not None:
                    count += sum(cls == 0 for cls in result.boxes.cls.cpu().numpy())
            return count
    
        def people_callback(self):
            self.get_logger().info("Starting to count people")
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().error("Cant open camera")
            frame = self.cap.read()
            results = self.model(frame, classes=[0])
            people_count = self.get_people_count(results)

            self.people_publisher.publish(Int16(data=int(people_count)))
            self.get_logger().info(f"People detected: {people_count}")
            
            cv2.imshow("People Detection", results[0].plot())
            cv2.waitKey(1)
        
        def destroy_node(self):
            super().destroy_node()
            self.cap.release()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CountWhat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()