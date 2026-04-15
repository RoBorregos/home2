#!/usr/bin/env python3

"""
Task Manager for testing the vision subtask manager
"""

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker

from frida_constants.vision_constants import CAMERA_FRAME, FOLLOW_TOPIC
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

# Choose which tests to perform
TEST_DETECT_OBJECTS = False
TEST_DETECT_PERSON = False
TEST_CUSTOMER_TABLES = False
TEST_MOONDREAM_QUERY = False
TEST_FIND_SEAT = False
TEST_GET_PERSON_NAME = False
TEST_FOLLOW_FACE = False
TEST_HAND_MARKER = False

FOLLOW_FACE_FLIP = False
HAND_MARKER_FLIP = False
HAND_MARKER_TOPIC = "/vision/test/hand_marker"


class TestVisionManager(Node):
    def __init__(self):
        super().__init__("test_vision_task_manager")
        self.vision_manager = VisionTasks(self, task=Task.DEBUG)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("TestVisionManager has started.")
        self.run()

    def run(self):
        if TEST_DETECT_OBJECTS:
            self.test_detect_objects()

        if TEST_DETECT_PERSON:
            self.test_detect_person()

        if TEST_CUSTOMER_TABLES:
            self.test_customer_tables()

        if TEST_MOONDREAM_QUERY:
            self.test_moondream_query()

        if TEST_FIND_SEAT:
            self.test_find_seat()

        if TEST_GET_PERSON_NAME:
            self.test_get_person_name()

        if TEST_FOLLOW_FACE:
            self.test_follow_face()

        if TEST_HAND_MARKER:
            self.test_hand_marker()

        exit(0)

    def test_detect_objects(self):
        Logger.info(self, "=== Testing detect_objects ===")
        status, detections = self.vision_manager.detect_objects()
        if status == Status.EXECUTION_SUCCESS:
            labels = self.vision_manager.get_labels(detections)
            Logger.success(self, f"Detected {len(detections)} objects: {labels}")
            for det in detections:
                Logger.info(
                    self,
                    f"  {det.classname}: distance={det.distance:.2f}m, "
                    f"point=({det.px:.3f}, {det.py:.3f}, {det.pz:.3f})",
                )
        else:
            Logger.error(self, "detect_objects failed")

    def test_detect_person(self):
        Logger.info(self, "=== Testing detect_person ===")
        status = self.vision_manager.detect_person()
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, "Person detected")
        else:
            Logger.warn(self, "No person detected")

    def test_customer_tables(self):
        Logger.info(self, "=== Testing customer_tables ===")
        status, tables = self.vision_manager.customer_tables()
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"Detected {len(tables)} table(s)")
            for i, table in enumerate(tables):
                num_people = len(table.people.list)
                pt = table.table_point.point
                Logger.info(
                    self,
                    f"  Table {i}: {num_people} customer(s), "
                    f"point=({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f})",
                )
                for person in table.people.list:
                    Logger.info(
                        self,
                        f"    Person: name='{person.name}', angle={person.angle:.1f}",
                    )
        else:
            Logger.error(self, "customer_tables failed")

    def test_moondream_query(self):
        Logger.info(self, "=== Testing moondream_query ===")
        status, result = self.vision_manager.moondream_query(
            "What objects do you see on the table?"
        )
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"Moondream response: {result}")
        else:
            Logger.error(self, "moondream_query failed")

    def test_find_seat(self):
        Logger.info(self, "=== Testing find_seat ===")
        status, angle = self.vision_manager.find_seat()
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"Found seat at angle: {angle:.2f}")
        else:
            Logger.warn(self, "No seat found")

    def test_get_person_name(self):
        Logger.info(self, "=== Testing get_person_name ===")
        Logger.info(self, "Activating face recognition...")
        self.vision_manager.activate_face_recognition()
        name = self.vision_manager.get_person_name()
        if name:
            Logger.success(self, f"Person name: {name}")
        else:
            Logger.warn(self, "No person name detected")
        self.vision_manager.deactivate_face_recognition()

    def test_follow_face(self):
        """Subscribe to the follow_face Point and log each incoming angular
        delta while face recognition is active. FOLLOW_FACE_FLIP controls
        whether the camera runs flipped (180°) or normal (0°).
        """
        Logger.info(self, "=== Testing follow_face ===")

        count = [0]

        def cb(msg: Point):
            count[0] += 1
            Logger.info(self, f"follow_face  x={msg.x:+.3f}  y={msg.y:+.3f}  (msg #{count[0]})")

        self.create_subscription(Point, FOLLOW_TOPIC, cb, 10)
        self.vision_manager.activate_face_recognition()
        self.vision_manager.follow_by_name("area")
        self.vision_manager.camera_upside_down(FOLLOW_FACE_FLIP)
        Logger.info(self, f"Move your face. flip={FOLLOW_FACE_FLIP}. Ctrl+C to stop.")

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.vision_manager.camera_upside_down(False)
            self.vision_manager.deactivate_face_recognition()
            Logger.info(self, f"follow_face test stopped ({count[0]} messages)")

    def test_hand_marker(self):
        """Continuously call detect_hand and publish its 3D point as a Marker
        in CAMERA_FRAME so it can be visualized in rviz. HAND_MARKER_FLIP
        controls whether the camera runs flipped (180°) or normal.
        """
        Logger.info(self, "=== Testing hand marker ===")

        marker_pub = self.create_publisher(Marker, HAND_MARKER_TOPIC, 10)
        self.vision_manager.camera_upside_down(HAND_MARKER_FLIP)
        Logger.info(
            self,
            f"Extend your hand in front of the camera. flip={HAND_MARKER_FLIP}. "
            f"Marker on '{HAND_MARKER_TOPIC}'. Ctrl+C to stop.",
        )

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                status, point = self.vision_manager.detect_hand()
                if status != Status.EXECUTION_SUCCESS or point is None:
                    continue

                marker = Marker()
                marker.header.frame_id = CAMERA_FRAME
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "hand_marker_test"
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position = point.point
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.08
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker_pub.publish(marker)
        except KeyboardInterrupt:
            pass
        finally:
            self.vision_manager.camera_upside_down(False)
            Logger.info(self, "hand marker test stopped")


def main(args=None):
    rclpy.init(args=args)
    node = TestVisionManager()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
