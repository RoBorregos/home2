#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from std_msgs.msg import Int16, String
from std_srvs.srv import SetBool
from cv_bridge import CvBridge

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    PEOPLE_DETECTIONS_TOPIC,
    WHAT_DETECTIONS_TOPIC,
)

class CountNode(Node):
    def __init__(self):
        super().__init__("how_many_what")

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return

        self.people_pub = self.create_publisher(Int16, PEOPLE_DETECTIONS_TOPIC, 10)
        self.object_pub = self.create_publisher(String, WHAT_DETECTIONS_TOPIC, 10)

        self.detect_mode()

    def detect_mode(self):
        print("What do you want to detect?")
        print("1: Object    2: People")
        try:
            mode = int(input("Choice: "))
            if mode == 1:
                self.detect_object()
            elif mode == 2:
                self.detect_people()
            else:
                print("Invalid option")
        except Exception as e:
            print(f"Error: {e}")

    def detect_people(self):
        self.get_logger().info("Detecting people...")

        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to read frame")
                break

            results = self.model(frame, classes=[0])
            count = self.count_class(results, 0)

            self.people_pub.publish(Int16(data=int(count)))
            self.get_logger().info(f"People detected: {count}")

            cv2.imshow("YOLOv8 People Detection", results[0].plot())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cleanup()

    def detect_object(self):
        class_dict = self.model.model.names
        print("Available objects:")
        for k, v in class_dict.items():
            print(f"{k}: {v}")

        try:
            obj_name = input("Enter object name to detect: ").lower()
            obj_id = next((k for k, v in class_dict.items() if v.lower() == obj_name), None)
            if obj_id is None:
                print("Object not found in model classes.")
                return

            self.get_logger().info(f"Detecting {obj_name}...")

            while rclpy.ok():
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error("Failed to read frame")
                    break

                results = self.model(frame, classes=[obj_id])
                count = self.count_class(results, obj_id)

                self.object_pub.publish(String(data=f"{obj_name}:{count}"))
                self.get_logger().info(f"{obj_name} detected: {count}")

                cv2.imshow("YOLOv8 Object Detection", results[0].plot())
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except Exception as e:
            print(f"Detection error: {e}")

        self.cleanup()

    def count_class(self, results, class_id):
        count = 0
        for result in results:
            if result.boxes.cls is not None:
                count += sum(cls == class_id for cls in result.boxes.cls.cpu().numpy())
        return count

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Camera and windows closed.")

def main(args=None):
    rclpy.init(args=args)
    node = CountNode()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
