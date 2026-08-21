#!/usr/bin/env python3

"""
Task Manager for testing the vision subtask manager
"""

import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker

from frida_constants.vision_constants import CAMERA_FRAME, FOLLOW_TOPIC
from frida_constants.vision_enums import Gestures, Poses
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
TEST_CHAIRS_TO_REMOVE = False  # needs hric_commands + yolo + moondream + camera
TEST_SAVE_FACE_NAME = False
TEST_FIND_DRINK = False
TEST_GET_CUSTOMER = False
TEST_COUNT_PERSON = False
TEST_COUNT_BY_POSE = False
TEST_COUNT_BY_GESTURE = False
TEST_COUNT_BY_COLOR = False
TEST_FIND_PERSON_INFO = False
TEST_COUNT_OBJECTS = False
TEST_VISUAL_INFO = False
TEST_TRACKING = False  # tests track_person + get_track_person + get_tracked_person_point
TEST_GET_FOLLOW_FACE = False
TEST_ISPERSON = False
TEST_DESCRIBE_BAG = False  # get_pointing_bag + describe_bag (+ moondream_crop_query)
TEST_GET_DRINK_POSITION = False
TEST_DESCRIBE_PERSON = False

FOLLOW_FACE_FLIP = False
HAND_MARKER_FLIP = False
HAND_MARKER_TOPIC = "/vision/test/hand_marker"
SAVE_NAME = "Jp"
DRINK = "coke"
POSE = Poses.STANDING.value
GESTURE = Gestures.WAVING.value
COLOR = "grey"
CLOTHING = "shirt"
INFO_TYPE = "pose"
OBJECT = "cup"
VISUAL_INFO_DESCRIPTION = "closest"


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

        if TEST_CHAIRS_TO_REMOVE:
            self.test_chairs_to_remove()

        if TEST_SAVE_FACE_NAME:
            self.test_save_face_name()

        if TEST_FIND_DRINK:
            self.test_find_drink()

        if TEST_GET_CUSTOMER:
            self.test_get_customer()

        if TEST_COUNT_PERSON:
            self.test_count_person()

        if TEST_COUNT_BY_POSE:
            self.test_count_by_pose()

        if TEST_COUNT_BY_GESTURE:
            self.test_count_by_gesture()

        if TEST_COUNT_BY_COLOR:
            self.test_count_by_color()

        if TEST_FIND_PERSON_INFO:
            self.test_find_person_info()

        if TEST_COUNT_OBJECTS:
            self.test_count_objects()

        if TEST_VISUAL_INFO:
            self.test_visual_info()

        if TEST_TRACKING:
            self.test_tracking()

        if TEST_GET_FOLLOW_FACE:
            self.test_get_follow_face()

        if TEST_ISPERSON:
            self.test_isperson()

        if TEST_DESCRIBE_BAG:
            self.test_describe_bag()

        if TEST_GET_DRINK_POSITION:
            self.test_get_drink_position()

        if TEST_DESCRIBE_PERSON:
            self.test_describe_person()

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

    def test_chairs_to_remove(self):
        """Run the full pipeline via the hric_commands service."""
        Logger.info(self, "=== Testing detect_chairs_to_remove ===")
        status, chairs = self.vision_manager.detect_chairs_to_remove()
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"{len(chairs)} chair(s) to remove")
            for x1, y1, x2, y2 in chairs:
                Logger.info(self, f"  remove chair px bbox: ({x1}, {y1}) -> ({x2}, {y2})")
            Logger.info(
                self,
                "hric_commands keeps publishing the annotated frame on "
                "/vision/chair_removal_image (view it in rqt_image_view)",
            )
        elif status == Status.TARGET_NOT_FOUND:
            Logger.warn(self, "No table found; would fall back to generic request")
        else:
            Logger.error(self, "detect_chairs_to_remove failed")

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

    def test_save_face_name(self):
        Logger.info(self, "=== Testing save_face_name ===")
        status = self.vision_manager.save_face_name(SAVE_NAME)
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"Name saved: {SAVE_NAME}")
        else:
            Logger.error(self, "save_face_name failed")

    def test_find_drink(self):
        Logger.info(self, "=== Testing find_drink ===")
        status, location = self.vision_manager.find_drink(DRINK)
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"Drink '{DRINK}' found at: {location}")
        else:
            Logger.warn(self, f"Drink '{DRINK}' not found")

    def test_get_customer(self):
        Logger.info(self, "=== Testing get_customer ===")
        status, point = self.vision_manager.get_customer()
        if status == Status.EXECUTION_SUCCESS:
            pt = point.point
            Logger.success(self, f"Customer found at ({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f})")
        else:
            Logger.warn(self, "No customer found")

    def test_count_person(self):
        Logger.info(self, "=== Testing count_person ===")
        status, count = self.vision_manager.count_person()
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"People counted: {count}")
        else:
            Logger.warn(self, "count_person failed")

    def test_count_by_pose(self):
        Logger.info(self, "=== Testing count_by_pose ===")
        status, count = self.vision_manager.count_by_pose(POSE)
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"People with pose '{POSE}': {count}")
        else:
            Logger.warn(self, f"No people with pose '{POSE}' found")

    def test_count_by_gesture(self):
        Logger.info(self, "=== Testing count_by_gesture ===")
        status, count = self.vision_manager.count_by_gesture(GESTURE)
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"People with gesture '{GESTURE}': {count}")
        else:
            Logger.warn(self, f"No people with gesture '{GESTURE}' found")

    def test_count_by_color(self):
        Logger.info(self, "=== Testing count_by_color ===")
        status, count = self.vision_manager.count_by_color(COLOR, CLOTHING)
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"People with {COLOR} {CLOTHING}: {count}")
        else:
            Logger.warn(self, f"No people with {COLOR} {CLOTHING} found")

    def test_find_person_info(self):
        Logger.info(self, "=== Testing find_person_info ===")
        status, result = self.vision_manager.find_person_info(INFO_TYPE)
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"Person {INFO_TYPE}: {result}")
        else:
            Logger.warn(self, f"No {INFO_TYPE} detected")

    def test_count_objects(self):
        Logger.info(self, "=== Testing count_objects ===")
        status, labels = self.vision_manager.count_objects(OBJECT)
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"Objects detected: {labels}")
        else:
            Logger.warn(self, "count_objects failed")

    def test_visual_info(self):
        Logger.info(self, "=== Testing visual_info ===")
        status, result = self.vision_manager.visual_info(VISUAL_INFO_DESCRIPTION, object=OBJECT)
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"Visual info: {result}")
        else:
            Logger.error(self, "visual_info failed")

    def test_tracking(self):
        """Lock the tracker, poll its status, read a tracked 3D point, then unlock.

        Covers track_person, get_track_person and get_tracked_person_point,
        which are meant to be used together in that order.
        """
        Logger.info(
            self,
            "=== Testing tracking (track_person / get_track_person / get_tracked_person_point) ===",
        )
        status = self.vision_manager.track_person(True)
        if status != Status.EXECUTION_SUCCESS:
            Logger.warn(self, "track_person(True) failed to lock a target")
            return
        Logger.success(self, "track_person(True) locked a target")

        status = self.vision_manager.get_track_person()
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, "get_track_person confirms a locked target")
        else:
            Logger.warn(self, "get_track_person reports no locked target")

        status, point = self.vision_manager.get_tracked_person_point()
        if status == Status.EXECUTION_SUCCESS:
            pt = point.point
            Logger.success(self, f"Tracked point: ({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f})")
        else:
            Logger.warn(self, "No tracked person point received")

        self.vision_manager.track_person(False)

    def test_get_follow_face(self):
        """Poll get_follow_face() while face recognition is active."""
        Logger.info(self, "=== Testing get_follow_face ===")
        self.vision_manager.activate_face_recognition()
        x, y = None, None
        start = time.time()
        while time.time() - start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            x, y = self.vision_manager.get_follow_face()
            if x is not None:
                break
        self.vision_manager.deactivate_face_recognition()
        if x is not None:
            Logger.success(self, f"get_follow_face: x={x:+.3f} y={y:+.3f}")
        else:
            Logger.warn(self, "get_follow_face returned no data")

    def test_isperson(self):
        """Poll the live person_list (populated by person_list_callback) for SAVE_NAME."""
        Logger.info(self, "=== Testing isPerson ===")
        found = False
        start = time.time()
        while time.time() - start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.vision_manager.isPerson(SAVE_NAME):
                found = True
                break
        if found:
            Logger.success(self, f"isPerson found '{SAVE_NAME}' in the live person list")
        else:
            Logger.warn(
                self,
                f"'{SAVE_NAME}' not in the live person list "
                f"({len(self.vision_manager.person_list)} people seen)",
            )

    def test_describe_bag(self):
        """Covers get_pointing_bag, describe_bag and (via describe_bag) moondream_crop_query."""
        Logger.info(self, "=== Testing get_pointing_bag / describe_bag ===")
        status, bbox, point = self.vision_manager.get_pointing_bag()
        if status != Status.EXECUTION_SUCCESS:
            Logger.warn(self, "get_pointing_bag: no bag found")
            return
        pt = point.point
        Logger.success(
            self,
            f"Bag found: bbox=({bbox.x1},{bbox.y1})->({bbox.x2},{bbox.y2}) "
            f"point=({pt.x:.3f},{pt.y:.3f},{pt.z:.3f})",
        )

        status, description = self.vision_manager.describe_bag(bbox)
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"Bag description: {description}")
        else:
            Logger.error(self, "describe_bag failed")

    def test_get_drink_position(self):
        """detect_objects, then locate DRINK among the detections."""
        Logger.info(self, "=== Testing get_drink_position ===")
        status, detections = self.vision_manager.detect_objects()
        if status != Status.EXECUTION_SUCCESS or not detections:
            Logger.warn(self, "get_drink_position: no objects detected")
            return
        status, location = self.vision_manager.get_drink_position(detections, DRINK)
        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"Drink '{DRINK}' position: {location}")
        else:
            Logger.warn(self, f"Drink '{DRINK}' not among detected objects")

    def test_describe_person(self):
        """Async, multi-turn person description built from several moondream_query_async calls."""
        Logger.info(self, "=== Testing describe_person ===")
        done = [False]

        def cb(status, description):
            done[0] = True
            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, f"Person description: {description}")
            else:
                Logger.error(self, "describe_person failed")

        self.vision_manager.describe_person(callback=cb)
        start = time.time()
        while not done[0] and time.time() - start < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not done[0]:
            Logger.warn(self, "describe_person timed out")


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
