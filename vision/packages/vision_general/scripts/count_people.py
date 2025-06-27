#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from std_msgs.msg import Int16
from frida_constants.vision_constants import CAMERA_TOPIC
from cv_bridge import CvBridge, CvBridgeError
#from frida_interfaces.srv import CountPeople

"""
Using node to count people in a camera feed using YOLOv8.

class CrowdDetection(Node):
    def __init__(self):
        super().__init__('crowd_detection')
        self.bridge = CvBridge()
        self.srv = self,creat_service(
            CountPeople, "count_people", self.count_people
        )
        self.model = YOLO("yolov8n.pt")
        self.image = None
        self.people = 0
        self.person_id = 0
        self.get_logger().info("Crowd Counting Node Initialized")

        def count_people(self, request, response):
            self.get_logger().info("Counting people in the camera")
            self.image = self.bridge.imgmsg_to_cv2(request.image, "bgr8")
            results = self.yolo_model(self.image, classes = self.person_id)
            response.result = Init16()
            response.result.data = len(results[0].boxes)
            result_frame = results[0].plot()
            cv2.imshow("Crowd Detection", result_frame)
            cv2.waitKey(1)
            response.success = True
            return response
        
def main(args=None):
    rclpy.init(args=args)
    count_crowd_service = CrowdDetection()
    try:
      rclpy.spin(count_crowd_service)
    except KeyboardInterrupt:
        pass    
    finally:
        count_crowd_service.get_logger().info("Shutting down Crowd Detection Node")
        cv2.destroyAllWindows()
        count_crowd_service.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
"""

# Using my camera to detect people instead of using a service
class CrowdDetection(Node):
    def __init__(self):
        super().__init__('crowd_detection')
        self.model = YOLO("yolov8n.pt")
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Cant open camera")
            return

        self.publisher = self.create_publisher(Int16, 'people_count', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Crowd Counting Node initialized")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Cant read frame from camera")
            return

        results = self.model(frame, classes=[0])
        people_count = self.get_people_count(results)

        self.publisher.publish(Int16(data=int(people_count)))
        self.get_logger().info(f"People detected: {people_count}")

        cv2.imshow("Crowd Detection", results[0].plot())
        cv2.waitKey(1)

    def get_people_count(self, results):
        count = 0
        for result in results:
            if result.boxes.cls is not None:
                count += sum(cls == 0 for cls in result.boxes.cls.cpu().numpy())
        return count

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CrowdDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
