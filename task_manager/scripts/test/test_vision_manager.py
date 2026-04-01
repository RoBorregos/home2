#!/usr/bin/env python3

"""
Task Manager for testing the vision subtask manager
"""

import rclpy
from rclpy.node import Node

from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.task import Task
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

# Choose which tests to perform
TEST_DETECT_OBJECTS = False
TEST_DETECT_PERSON = False
TEST_CUSTOMER_TABLES = True
TEST_MOONDREAM_QUERY = False
TEST_FIND_SEAT = False
TEST_GET_PERSON_NAME = False


class TestVisionManager(Node):
    def __init__(self):
        super().__init__("test_vision_task_manager")
        self.vision_manager = VisionTasks(self, task=Task.DEBUG)
        self.marker_pub = self.create_publisher(MarkerArray, "/test/customer_markers", 10)
        self.last_markers = None
        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("TestVisionManager has started.")

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

        Logger.info(self, "Tests done. Keeping node alive to publish markers (Ctrl+C to stop).")

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
                st = table.table_point.header.stamp
                Logger.info(
                    self,
                    f"  Table {i}: {num_people} customer(s), "
                    f"point=({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f}), "
                    f"stamp={st.sec}.{st.nanosec:09d}",
                )
                for person in table.people.list:
                    ppt = person.point3d.point
                    pst = person.point3d.header.stamp
                    Logger.info(
                        self,
                        f"    Person: name='{person.name}', angle={person.angle:.1f}, "
                        f"point=({ppt.x:.3f}, {ppt.y:.3f}, {ppt.z:.3f}), "
                        f"stamp={pst.sec}.{pst.nanosec:09d}",
                    )
            # Publish markers for all people and tables
            marker_array = MarkerArray()
            marker_id = 0
            for i, table in enumerate(tables):
                # Table marker (green cube)
                tm = Marker()
                tm.header = table.table_point.header
                tm.ns = "tables"
                tm.id = marker_id
                marker_id += 1
                tm.type = Marker.CUBE
                tm.action = Marker.ADD
                tm.pose.position = table.table_point.point
                tm.pose.orientation.w = 1.0
                tm.scale.x = 0.15
                tm.scale.y = 0.15
                tm.scale.z = 0.05
                tm.color.g = 1.0
                tm.color.a = 0.8
                marker_array.markers.append(tm)

                # Person markers (red spheres)
                for person in table.people.list:
                    pm = Marker()
                    pm.header = person.point3d.header
                    pm.ns = "customers"
                    pm.id = marker_id
                    marker_id += 1
                    pm.type = Marker.SPHERE
                    pm.action = Marker.ADD
                    pm.pose.position = person.point3d.point
                    pm.pose.orientation.w = 1.0
                    pm.scale.x = 0.1
                    pm.scale.y = 0.1
                    pm.scale.z = 0.1
                    pm.color.r = 1.0
                    pm.color.a = 0.8
                    marker_array.markers.append(pm)

            self.last_markers = marker_array
            self.marker_pub.publish(marker_array)
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


def main(args=None):
    rclpy.init(args=args)
    node = TestVisionManager()
    node.run()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            if node.last_markers:
                node.marker_pub.publish(node.last_markers)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
