#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
from frida_constants.vision_constants import CAMERA_TOPIC, PEOPLE_TOPIC


class PeopleCounter(Node):
    def __init__(self):
        super().__init__("people_counter")
        self.bridge = CvBridge()
        self.image = None
        self.person_class_id = 0  
        self.people_count = 0

        self.yolo_model = YOLO("yolov8n.pt")
        
        self.subscription = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )
        self.publisher = self.create_publisher(Int16, PEOPLE_TOPIC, 10)

        self.timer = self.create_timer(1.0, self.detect_people)

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed: {e}")

    def detect_people(self):
        if self.image is None:
            self.get_logger().info("Waiting...")
            return

        results = self.yolo_model(self.image, verbose=False, classes=self.person_class_id)
        detected_people = len(results[0].boxes)

        people_msg = Int16()
        people_msg.data = detected_people
        self.publisher.publish(people_msg)

        annotated_frame = results[0].plot()
        cv2.imshow("Detected People", annotated_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PeopleCounter()
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
