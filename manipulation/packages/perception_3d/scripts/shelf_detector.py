#!/usr/bin/env python3
"""Model-free shelf detection service (DetectShelf).

Turns depth frames from the eye-in-hand ZED into shelf levels, walls and planning
scene boxes, all in base_link. The caller drives the camera: move to a survey pose,
``capture``; move to the next, ``capture``; then ``detect``. Frames stay buffered
until ``reset`` so several views combine into one scan. The geometry itself lives in
``perception_3d.shelf_geometry`` (no ROS there, see its tests).

Debug: the accumulated cloud goes out on SHELF_SCAN_CLOUD_TOPIC and the detected
boards/walls as markers on SHELF_MARKERS_TOPIC (both in base_link, for RViz).
"""

import threading
import time
from dataclasses import asdict

import numpy as np
import rclpy
from cv_bridge import CvBridge
from frida_constants.manipulation_constants import (
    DETECT_SHELF_SERVICE,
    SHELF_COLLISION_NAMESPACE,
    SHELF_FRAME,
    SHELF_MARKERS_TOPIC,
    SHELF_SCAN_CLOUD_TOPIC,
    ZED_DEPTH_SCALE,
)
from frida_constants.vision_constants import CAMERA_INFO_TOPIC, DEPTH_IMAGE_TOPIC
from frida_interfaces.msg import CollisionObject, ShelfLevel
from frida_interfaces.srv import AddCollisionObjects, DetectShelf, RemoveCollisionObject
from perception_3d.shelf_geometry import (
    Frame,
    ShelfGeometry,
    ShelfNotFound,
    ShelfParams,
    detect_shelf,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

ADD_COLLISION_OBJECTS_SERVICE = "/manipulation/add_collision_objects"
REMOVE_COLLISION_OBJECT_SERVICE = "/manipulation/remove_collision_object"
FRESH_FRAME_TIMEOUT = 2.0  # s to wait for a depth frame newer than the request
TF_TIMEOUT = 1.0


class ShelfDetector(Node):
    def __init__(self):
        super().__init__("shelf_detector")
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._depth = None  # (image, frame_id, receive time)
        self._intrinsics = None  # (fx, fy, cx, cy)
        self._frames: list[Frame] = []
        self._geometry: ShelfGeometry | None = None
        self._published_ids: list[str] = []
        self._busy = threading.Lock()

        # Every pixel of a HD720/2 depth image is far more than the 1 cm voxels
        # need; stride 2 keeps ~60k points per frame.
        self.declare_parameter("depth_stride", 2)
        defaults = ShelfParams()
        for name, value in asdict(defaults).items():
            self.declare_parameter(name, value)

        group = ReentrantCallbackGroup()
        self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self._depth_cb, 10, callback_group=group
        )
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._info_cb, 1)
        self._cloud_pub = self.create_publisher(PointCloud2, SHELF_SCAN_CLOUD_TOPIC, 1)
        self._markers_pub = self.create_publisher(MarkerArray, SHELF_MARKERS_TOPIC, 1)
        self._add_objects_client = self.create_client(
            AddCollisionObjects, ADD_COLLISION_OBJECTS_SERVICE, callback_group=group
        )
        self._remove_object_client = self.create_client(
            RemoveCollisionObject, REMOVE_COLLISION_OBJECT_SERVICE, callback_group=group
        )
        self.create_service(
            DetectShelf, DETECT_SHELF_SERVICE, self._detect_cb, callback_group=group
        )
        self.get_logger().info("Shelf detector ready")

    # ------------------------------------------------------------------
    # Service
    # ------------------------------------------------------------------

    def _detect_cb(self, request, response):
        if not self._busy.acquire(blocking=False):
            return self._fail(response, "busy: another request is running")
        try:
            return self._handle(request, response)
        except Exception as error:  # noqa: BLE001 - a service must always answer
            self.get_logger().error(f"detect_shelf failed: {error!r}")
            return self._fail(response, f"internal error: {error}")
        finally:
            self._busy.release()

    def _handle(self, request, response):
        if request.reset:
            self._frames = []
            self._geometry = None
        if request.capture:
            frame, message = self._capture()
            if frame is None:
                return self._fail(response, message)
            self._frames.append(frame)
            self.get_logger().info(
                f"Captured frame {len(self._frames)} with {len(frame.points)} points"
            )
        if request.detect:
            if not self._frames:
                return self._fail(
                    response, "nothing captured: call with capture=true first"
                )
            params = self._params()
            started = time.time()
            try:
                self._geometry = detect_shelf(self._frames, params)
            except ShelfNotFound as why:
                self._publish_debug(params, None)
                return self._fail(response, f"no shelf: {why}")
            self._publish_debug(params, self._geometry)
            self.get_logger().info(
                f"Shelf: {len(self._geometry.levels)} levels at "
                f"{[round(h, 3) for h in self._geometry.heights]} m, "
                f"y [{self._geometry.y_min:.2f}, {self._geometry.y_max:.2f}], "
                f"x [{self._geometry.front_x:.2f}, {self._geometry.back_x:.2f}], "
                f"top {self._geometry.top_z:.2f}, walls L/R/B "
                f"{self._geometry.has_left_wall}/{self._geometry.has_right_wall}/"
                f"{self._geometry.has_back_wall} ({time.time() - started:.2f} s)"
            )
        if request.add_collision_objects:
            if self._geometry is None:
                return self._fail(
                    response, "no shelf geometry to add to the planning scene"
                )
            self._replace_scene_objects(self._collision_objects(self._geometry))

        response.frames_used = len(self._frames)
        if self._geometry is not None:
            self._fill(response, self._geometry)
        response.success = True
        response.message = (
            f"{len(self._frames)} frame(s) buffered"
            if self._geometry is None
            else f"{len(self._geometry.levels)} level(s) detected"
        )
        return response

    # ------------------------------------------------------------------
    # Capture: depth image -> base_link points
    # ------------------------------------------------------------------

    def _capture(self):
        if self._intrinsics is None:
            return None, "camera intrinsics not received yet"
        deadline = time.time() + FRESH_FRAME_TIMEOUT
        requested = time.time()
        while time.time() < deadline:
            depth = self._depth
            if depth is not None and depth[2] >= requested:
                break
            time.sleep(0.02)
        else:
            return None, "no fresh depth frame"
        image, frame_id, _ = depth
        try:
            cam_to_base = self._to_base(frame_id)
        except TransformException as error:
            return None, f"no TF {frame_id} -> {SHELF_FRAME}: {error}"

        stride = max(1, int(self.get_parameter("depth_stride").value))
        sub = image[::stride, ::stride]
        valid = np.isfinite(sub) & (sub > 0.0)
        v, u = np.nonzero(valid)
        z = sub[v, u].astype(float) * ZED_DEPTH_SCALE
        fx, fy, cx, cy = self._intrinsics
        u = u * stride
        v = v * stride
        cam = np.column_stack(
            ((u - cx) * z / fx, (v - cy) * z / fy, z, np.ones_like(z))
        )
        points = (cam_to_base @ cam.T).T[:, :3]
        return Frame(points=points, viewpoint=cam_to_base[:3, 3].copy()), "ok"

    def _to_base(self, frame: str) -> np.ndarray:
        transform = self._tf_buffer.lookup_transform(
            SHELF_FRAME, frame, rclpy.time.Time(), timeout=Duration(seconds=TF_TIMEOUT)
        ).transform
        r, t = transform.rotation, transform.translation
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix

    def _depth_cb(self, msg: Image) -> None:
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self._depth = (image, msg.header.frame_id, time.time())

    def _info_cb(self, msg: CameraInfo) -> None:
        if self._intrinsics is None:
            self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _params(self) -> ShelfParams:
        values = {}
        for name, default in asdict(ShelfParams()).items():
            value = self.get_parameter(name).value
            values[name] = type(default)(value)
        return ShelfParams(**values)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def _fill(self, response, g: ShelfGeometry) -> None:
        for level in g.levels:
            msg = ShelfLevel()
            msg.height = float(level.height)
            msg.thickness = float(level.thickness)
            msg.front_x = float(level.front_x)
            msg.back_x = float(level.back_x)
            msg.y_min = float(level.y_min)
            msg.y_max = float(level.y_max)
            msg.clearance = float(level.clearance)
            msg.num_points = int(level.num_points)
            response.levels.append(msg)
        response.y_min = float(g.y_min)
        response.y_max = float(g.y_max)
        response.front_x = float(g.front_x)
        response.back_x = float(g.back_x)
        response.top_z = float(g.top_z)
        response.collision_objects = self._collision_objects(g)

    def _collision_objects(self, g: ShelfGeometry) -> list:
        objects = []
        for box in g.boxes:
            obj = CollisionObject()
            obj.id = box.id
            obj.type = "box"
            obj.pose.header.frame_id = SHELF_FRAME
            obj.pose.header.stamp = self.get_clock().now().to_msg()
            obj.pose.pose.position.x = float(box.center[0])
            obj.pose.pose.position.y = float(box.center[1])
            obj.pose.pose.position.z = float(box.center[2])
            obj.pose.pose.orientation.w = 1.0
            obj.dimensions.x = float(box.size[0])
            obj.dimensions.y = float(box.size[1])
            obj.dimensions.z = float(box.size[2])
            objects.append(obj)
        return objects

    def _replace_scene_objects(self, objects: list) -> None:
        """Drop the previously published shelf boxes, then add the current ones."""
        if self._remove_object_client.wait_for_service(timeout_sec=2.0):
            for object_id in self._published_ids:
                request = RemoveCollisionObject.Request()
                request.id = object_id
                self._remove_object_client.call_async(request)
        if not self._add_objects_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                "add_collision_objects unavailable; shelf not in the scene"
            )
            return
        request = AddCollisionObjects.Request()
        request.collision_objects = objects
        self._add_objects_client.call_async(request)
        self._published_ids = [obj.id for obj in objects]
        self.get_logger().info(
            f"Planning scene: {len(objects)} shelf boxes ({SHELF_COLLISION_NAMESPACE}*)"
        )

    def _publish_debug(self, params: ShelfParams, g: ShelfGeometry | None) -> None:
        header = Header()
        header.frame_id = SHELF_FRAME
        header.stamp = self.get_clock().now().to_msg()
        points = np.vstack([frame.points for frame in self._frames])
        if len(points) > 200_000:
            points = points[:: len(points) // 200_000 + 1]
        self._cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(header, points.astype(np.float32))
        )

        markers = [Marker(action=Marker.DELETEALL)]
        if g is not None:
            for i, box in enumerate(g.boxes):
                marker = Marker(ns="shelf", id=i, type=Marker.CUBE, action=Marker.ADD)
                marker.header = header
                (
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                ) = map(float, box.center)
                marker.pose.orientation.w = 1.0
                marker.scale.x, marker.scale.y, marker.scale.z = map(float, box.size)
                marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.5)
                markers.append(marker)
            for i, level in enumerate(g.levels):
                marker = Marker(
                    ns="shelf_level",
                    id=i,
                    type=Marker.TEXT_VIEW_FACING,
                    action=Marker.ADD,
                )
                marker.header = header
                marker.pose.position.x = float(level.front_x)
                marker.pose.position.y = float(g.y_min)
                marker.pose.position.z = float(level.height + 0.03)
                marker.pose.orientation.w = 1.0
                marker.scale.z = 0.05
                marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                marker.text = f"L{i + 1} z={level.height:.3f} clr={level.clearance:.2f}"
                markers.append(marker)
        self._markers_pub.publish(MarkerArray(markers=markers))

    def _fail(self, response, message: str):
        self.get_logger().warn(message)
        response.success = False
        response.message = message
        response.frames_used = len(self._frames)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ShelfDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
