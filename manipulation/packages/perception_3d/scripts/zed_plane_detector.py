#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import threading

from geometry_msgs.msg import PointStamped, PoseStamped, Vector3, Quaternion
from zed_msgs.msg import PlaneStamped
from frida_interfaces.srv import DetectPlane, AddCollisionObjects
from frida_interfaces.msg import CollisionObject
from frida_constants.manipulation_constants import (
    DETECT_PLANE_SERVICE,
    ADD_COLLISION_OBJECT_SERVICE,
    ZED_PLANE_TOPIC,
    ZED_CLICKED_POINT_TOPIC,
    PLANE_NAMESPACE,
)

PLANE_COLLISION_THICKNESS = 0.025
PLANE_DETECTION_TIMEOUT = 3.0


class ZedPlaneDetector(Node):
    def __init__(self):
        super().__init__("zed_plane_detector")
        self.callback_group = ReentrantCallbackGroup()

        self._plane_received = threading.Event()
        self._last_plane = None

        # Publisher to trigger ZED plane detection
        self._clicked_point_pub = self.create_publisher(
            PointStamped,
            ZED_CLICKED_POINT_TOPIC,
            10,
        )

        # Subscriber for ZED plane detection result
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self._plane_sub = self.create_subscription(
            PlaneStamped,
            ZED_PLANE_TOPIC,
            self._plane_callback,
            qos,
            callback_group=self.callback_group,
        )

        # Service server for plane detection
        self._detect_plane_srv = self.create_service(
            DetectPlane,
            DETECT_PLANE_SERVICE,
            self._detect_plane_callback,
            callback_group=self.callback_group,
        )

        # Client to add collision objects
        self._add_collision_client = self.create_client(
            AddCollisionObjects,
            ADD_COLLISION_OBJECT_SERVICE,
        )

        self.get_logger().info("ZED Plane Detector node started")

    def _plane_callback(self, msg):
        """Store the latest plane detection result from ZED."""
        self._last_plane = msg
        self._plane_received.set()

    def _detect_plane_callback(self, request, response):
        """Handle DetectPlane service request."""
        self.get_logger().info("Received DetectPlane request")

        response.success = False

        # Clear previous detection
        self._plane_received.clear()
        self._last_plane = None

        # Publish point to trigger ZED plane detection
        self._clicked_point_pub.publish(request.point)
        self.get_logger().info(
            f"Published point ({request.point.point.x:.3f}, "
            f"{request.point.point.y:.3f}, {request.point.point.z:.3f}) "
            f"in frame '{request.point.header.frame_id}'"
        )

        # Wait for ZED to respond
        if not self._plane_received.wait(timeout=PLANE_DETECTION_TIMEOUT):
            self.get_logger().error("Timed out waiting for ZED plane detection")
            return response

        plane = self._last_plane
        if plane is None:
            self.get_logger().error("Plane message is None")
            return response

        # Build response from ZED PlaneStamped
        response.normal = Vector3(
            x=float(plane.normal.x),
            y=float(plane.normal.y),
            z=float(plane.normal.z),
        )
        response.width = float(plane.extents[0])
        response.height = float(plane.extents[1])
        response.coefficients = [
            float(plane.coefficients.coef[0]),
            float(plane.coefficients.coef[1]),
            float(plane.coefficients.coef[2]),
            float(plane.coefficients.coef[3]),
        ]

        # Build plane pose
        response.plane_pose = PoseStamped()
        response.plane_pose.header = plane.header
        response.plane_pose.pose.position.x = float(plane.center.x)
        response.plane_pose.pose.position.y = float(plane.center.y)
        response.plane_pose.pose.position.z = float(plane.center.z)
        response.plane_pose.pose.orientation = Quaternion(
            x=float(plane.pose.rotation.x),
            y=float(plane.pose.rotation.y),
            z=float(plane.pose.rotation.z),
            w=float(plane.pose.rotation.w),
        )

        # Add plane as collision object
        collision_added = self._add_plane_collision(response)
        if not collision_added:
            self.get_logger().error("Failed to add plane collision object")
            return response

        response.success = True
        self.get_logger().info(
            f"Plane detected: center=({plane.center.x:.3f}, {plane.center.y:.3f}, "
            f"{plane.center.z:.3f}), extents=({response.width:.3f}, {response.height:.3f})"
        )
        return response

    def _add_plane_collision(self, plane_response):
        """Add the detected plane as a box collision object to the planning scene."""
        if not self._add_collision_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("AddCollisionObjects service not available")
            return False

        request = AddCollisionObjects.Request()

        collision_obj = CollisionObject()
        collision_obj.id = PLANE_NAMESPACE
        collision_obj.type = "box"
        collision_obj.pose = plane_response.plane_pose
        collision_obj.dimensions.x = plane_response.width
        collision_obj.dimensions.y = plane_response.height
        collision_obj.dimensions.z = PLANE_COLLISION_THICKNESS

        request.collision_objects = [collision_obj]

        future = self._add_collision_client.call_async(request)

        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=5.0):
            self.get_logger().error("AddCollisionObjects call timed out")
            return False

        if future.result() is None:
            self.get_logger().error("AddCollisionObjects call failed")
            return False

        return future.result().success


def main(args=None):
    rclpy.init(args=args)
    node = ZedPlaneDetector()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
