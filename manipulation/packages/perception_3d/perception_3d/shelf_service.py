"""DetectShelf: captured depth views -> shelf levels, walls and planning scene boxes.

Hosted by grasp_generator, which already owns the depth image, intrinsics and TF.
"""

import threading
import time
from dataclasses import asdict

import numpy as np
import rclpy
from frida_constants.manipulation_constants import (
    ADD_COLLISION_OBJECT_SERVICE,
    DETECT_SHELF_SERVICE,
    REMOVE_COLLISION_OBJECT_SERVICE,
    SHELF_COLLISION_NAMESPACE,
    SHELF_FRAME,
    SHELF_MARKERS_TOPIC,
    SHELF_SCAN_CLOUD_TOPIC,
    ZED_DEPTH_SCALE,
)
from frida_interfaces.msg import CollisionObject, ShelfLevel
from frida_interfaces.srv import AddCollisionObjects, DetectShelf, RemoveCollisionObject
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

from perception_3d.shelf_geometry import (
    Frame,
    ShelfGeometry,
    ShelfNotFound,
    ShelfParams,
    detect_shelf,
)

PARAM_PREFIX = "shelf."
FRESH_FRAME_TIMEOUT = 2.0  # s to wait for a depth frame newer than the request
TF_TIMEOUT = 1.0
MAX_DEBUG_POINTS = 200_000
# Not ShelfParams: how the frames are captured and published.
NODE_PARAMS = {
    "depth_stride": 2,  # 1 cm voxels need far fewer pixels than the image has
    "min_range": 0.3,  # the ZED2 returns nothing closer
    "max_range": 2.5,  # too noisy past this
    "depth_scale": float(ZED_DEPTH_SCALE),  # 1.0 for a simulated camera
    "dump_path": "",  # non-empty: save the captured frames there on every detect
    "odom_frame": "odom",
    "max_base_shift": 0.03,  # m the base may move before the boxes are refused
    "max_base_turn_deg": 3.0,
}


class ShelfService:
    """Serves DetectShelf on ``node`` with frames from ``fresh_depth``."""

    def __init__(self, node, tf_buffer, callback_group, fresh_depth, intrinsics):
        self._node = node
        self._tf_buffer = tf_buffer
        self._fresh_depth = fresh_depth  # (timeout) -> (image, frame_id) or None
        self._intrinsics = intrinsics  # () -> object with fx, fy, cx, cy, or None
        self._frames: list[Frame] = []
        self._geometry: ShelfGeometry | None = None
        self._published_ids: list[str] = []
        self._scan_base = None  # odom -> base_link when the geometry was detected
        self._busy = threading.Lock()

        for name, value in {**NODE_PARAMS, **asdict(ShelfParams())}.items():
            node.declare_parameter(PARAM_PREFIX + name, value)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._cloud_pub = node.create_publisher(PointCloud2, SHELF_SCAN_CLOUD_TOPIC, 1)
        self._markers_pub = node.create_publisher(
            MarkerArray, SHELF_MARKERS_TOPIC, latched
        )
        self._add_objects_client = node.create_client(
            AddCollisionObjects,
            ADD_COLLISION_OBJECT_SERVICE,
            callback_group=callback_group,
        )
        self._remove_object_client = node.create_client(
            RemoveCollisionObject,
            REMOVE_COLLISION_OBJECT_SERVICE,
            callback_group=callback_group,
        )
        node.create_service(
            DetectShelf,
            DETECT_SHELF_SERVICE,
            self._detect_cb,
            callback_group=callback_group,
        )

    def _param(self, name: str):
        return self._node.get_parameter(PARAM_PREFIX + name).value

    def _detect_cb(self, request, response):
        if not self._busy.acquire(blocking=False):
            return self._fail(response, "busy: another request is running")
        try:
            return self._handle(request, response)
        except Exception as error:  # noqa: BLE001 - a service must always answer
            self._node.get_logger().error(f"detect_shelf failed: {error!r}")
            return self._fail(response, f"internal error: {error}")
        finally:
            self._busy.release()

    def _handle(self, request, response):
        log = self._node.get_logger()
        if request.reset:
            self._frames = []
            self._geometry = None
        if request.capture:
            frame, message = self._capture()
            if frame is None:
                return self._fail(response, message)
            self._frames.append(frame)
            log.info(f"Shelf frame {len(self._frames)} with {len(frame.points)} points")
        if request.detect:
            if not self._frames:
                return self._fail(
                    response, "nothing captured: call with capture=true first"
                )
            params = self._shelf_params()
            self._dump()
            started = time.time()
            try:
                self._geometry = detect_shelf(self._frames, params)
                self._scan_base = self._odom_base()
            except ShelfNotFound as why:
                self._publish_debug(None)
                return self._fail(response, f"no shelf: {why}")
            self._publish_debug(self._geometry)
            g = self._geometry
            log.info(
                f"Shelf: {len(g.levels)} levels at {[round(h, 3) for h in g.heights]} m, "
                f"y [{g.y_min:.2f}, {g.y_max:.2f}], x [{g.front_x:.2f}, {g.back_x:.2f}], "
                f"top {g.top_z:.2f}, walls L/R/B {g.has_left_wall}/{g.has_right_wall}/"
                f"{g.has_back_wall} ({time.time() - started:.2f} s)"
            )
        if request.add_collision_objects:
            error = self._add_to_scene()
            if error:
                return self._fail(response, error)

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

    def _add_to_scene(self) -> str:
        """Publish the boxes of the last scan. Returns why not, or ''."""
        if self._geometry is None:
            return "no shelf geometry to add to the planning scene"
        moved = self._base_moved()
        if moved:
            return f"robot moved since the shelf scan: {moved}"
        if not self._replace_scene_objects(self._collision_objects(self._geometry)):
            return "planning scene did not take the shelf boxes"
        return ""

    def _capture(self):
        """One depth frame as base_link points: (Frame, 'ok') or (None, why)."""
        intrinsics = self._intrinsics()
        if intrinsics is None:
            return None, "camera intrinsics not received yet"
        depth = self._fresh_depth(FRESH_FRAME_TIMEOUT)
        if depth is None:
            return None, "no fresh depth frame"
        image, frame_id = depth
        try:
            cam_to_base = self._lookup(SHELF_FRAME, frame_id, TF_TIMEOUT)
        except TransformException as error:
            return None, f"no TF {frame_id} -> {SHELF_FRAME}: {error}"

        stride = max(1, int(self._param("depth_stride")))
        sub = image[::stride, ::stride]
        v, u = np.nonzero(np.isfinite(sub) & (sub > 0.0))
        z = sub[v, u].astype(float) * float(self._param("depth_scale"))
        in_range = (z >= self._param("min_range")) & (z <= self._param("max_range"))
        v, u, z = v[in_range] * stride, u[in_range] * stride, z[in_range]
        cam = np.column_stack(
            (
                (u - intrinsics.cx) * z / intrinsics.fx,
                (v - intrinsics.cy) * z / intrinsics.fy,
                z,
                np.ones_like(z),
            )
        )
        points = (cam_to_base @ cam.T).T[:, :3]
        return Frame(points=points, viewpoint=cam_to_base[:3, 3].copy()), "ok"

    def _dump(self) -> None:
        path = self._param("dump_path")
        if not path:
            return
        np.savez_compressed(
            path,
            **{f"points_{i}": f.points for i, f in enumerate(self._frames)},
            **{f"viewpoint_{i}": f.viewpoint for i, f in enumerate(self._frames)},
        )
        self._node.get_logger().info(f"Dumped {len(self._frames)} frame(s) to {path}")

    def _odom_base(self):
        """odom -> base_link as a 4x4 matrix, or None without odometry."""
        try:
            return self._lookup(self._param("odom_frame"), SHELF_FRAME, 0.2)
        except TransformException:
            return None

    def _base_moved(self) -> str:
        """Why the stored geometry no longer matches the base pose, or ''."""
        if self._scan_base is None:
            return ""  # no odometry at scan time: a static base (simulation)
        now = self._odom_base()
        if now is None:
            return "odometry lost since the scan"
        delta = np.linalg.inv(self._scan_base) @ now
        shift = float(np.linalg.norm(delta[:2, 3]))
        turn = float(np.degrees(abs(np.arctan2(delta[1, 0], delta[0, 0]))))
        if shift > self._param("max_base_shift") or turn > self._param(
            "max_base_turn_deg"
        ):
            return f"base moved {shift * 100:.1f} cm / {turn:.1f} deg"
        return ""

    def _lookup(self, target: str, source: str, timeout: float) -> np.ndarray:
        transform = self._tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(), timeout=Duration(seconds=timeout)
        ).transform
        r, t = transform.rotation, transform.translation
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix

    def _shelf_params(self) -> ShelfParams:
        defaults = asdict(ShelfParams())
        return ShelfParams(
            **{name: type(value)(self._param(name)) for name, value in defaults.items()}
        )

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
            msg.seen_from_below = bool(level.seen_from_below)
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
            obj.pose.header.stamp = self._node.get_clock().now().to_msg()
            position = obj.pose.pose.position
            position.x, position.y, position.z = map(float, box.center)
            obj.pose.pose.orientation.w = 1.0
            size = obj.dimensions
            size.x, size.y, size.z = map(float, box.size)
            objects.append(obj)
        return objects

    def _replace_scene_objects(self, objects: list) -> bool:
        """Publish the shelf boxes and wait for the planning scene to take them."""
        log = self._node.get_logger()
        # Ids are stable, so adding replaces in place; remove only the ones now gone.
        new_ids = {obj.id for obj in objects}
        stale = [i for i in self._published_ids if i not in new_ids]
        if stale and self._remove_object_client.wait_for_service(timeout_sec=2.0):
            for object_id in stale:
                request = RemoveCollisionObject.Request()
                request.id = object_id
                self._wait(self._remove_object_client.call_async(request), 3.0)
        if not self._add_objects_client.wait_for_service(timeout_sec=3.0):
            log.warning("add_collision_objects unavailable; shelf not in the scene")
            return False
        request = AddCollisionObjects.Request()
        request.collision_objects = objects
        if not self._wait(self._add_objects_client.call_async(request), 5.0):
            log.warning("add_collision_objects did not answer")
            return False
        self._published_ids = sorted(new_ids)
        log.info(
            f"Planning scene: {len(objects)} shelf boxes ({SHELF_COLLISION_NAMESPACE}*)"
        )
        return True

    @staticmethod
    def _wait(future, timeout: float) -> bool:
        """Block this callback until ``future`` is done (other threads keep spinning)."""
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.005)
        return future.done()

    def _publish_debug(self, g: ShelfGeometry | None) -> None:
        header = Header()
        header.frame_id = SHELF_FRAME
        header.stamp = self._node.get_clock().now().to_msg()
        points = np.vstack([frame.points for frame in self._frames])
        if len(points) > MAX_DEBUG_POINTS:
            points = points[:: len(points) // MAX_DEBUG_POINTS + 1]
        self._cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(header, points.astype(np.float32))
        )

        markers = [Marker(action=Marker.DELETEALL)]
        for i, box in enumerate(g.boxes if g else ()):
            marker = Marker(ns="shelf", id=i, type=Marker.CUBE, action=Marker.ADD)
            marker.header = header
            position = marker.pose.position
            position.x, position.y, position.z = map(float, box.center)
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = map(float, box.size)
            marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.5)
            markers.append(marker)
        for i, level in enumerate(g.levels if g else ()):
            marker = Marker(
                ns="shelf_level", id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD
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
        self._node.get_logger().warn(message)
        response.success = False
        response.message = message
        response.frames_used = len(self._frames)
        return response
