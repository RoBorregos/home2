"""DetectShelf: captured depth views -> shelf levels, walls and their collision boxes.

Hosted by grasp_generator, which already owns the depth image, intrinsics and TF.
Only manipulation_core writes the planning scene: the boxes are returned, not added.
"""

import threading
import time
from dataclasses import asdict

import numpy as np
import rclpy
from frida_constants.manipulation_constants import DETECT_SHELF_SERVICE, ZED_DEPTH_SCALE
from frida_interfaces.msg import CollisionObject, ShelfLevel
from frida_interfaces.srv import DetectShelf
from rclpy.duration import Duration
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException

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
# ShelfParams fields tuned on site; the rest keep their dataclass defaults.
TUNED_SHELF_PARAMS = (
    "x_min",
    "x_max",
    "y_half",
    "z_min",
    "z_max",
    "min_width_fraction",
    "min_level_spacing",
    "min_level_points",
)
# Not ShelfParams: how the frames are captured.
NODE_PARAMS = {
    "depth_stride": 2,  # 1 cm voxels need far fewer pixels than the image has
    "min_range": 0.3,  # the ZED2 returns nothing closer
    "max_range": 2.5,  # too noisy past this
    "depth_scale": float(ZED_DEPTH_SCALE),  # 1.0 for a simulated camera
    "dump_path": "",  # non-empty: save the captured frames there on every detect
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
        self._busy = threading.Lock()

        defaults = asdict(ShelfParams())
        tuned = {name: defaults[name] for name in TUNED_SHELF_PARAMS}
        for name, value in {**NODE_PARAMS, **tuned}.items():
            node.declare_parameter(PARAM_PREFIX + name, value)
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
            except ShelfNotFound as why:
                return self._fail(response, f"no shelf: {why}")
            g = self._geometry
            log.info(
                f"Shelf: {len(g.levels)} levels at {[round(h, 3) for h in g.heights]} m, "
                f"y [{g.y_min:.2f}, {g.y_max:.2f}], x [{g.front_x:.2f}, {g.back_x:.2f}], "
                f"top {g.top_z:.2f}, walls L/R/B {g.has_left_wall}/{g.has_right_wall}/"
                f"{g.has_back_wall} ({time.time() - started:.2f} s)"
            )

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
            cam_to_base = self._lookup("base_link", frame_id, TF_TIMEOUT)
        except TransformException as error:
            return None, f"no TF {frame_id} -> base_link: {error}"

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
            **{
                name: type(defaults[name])(self._param(name))
                for name in TUNED_SHELF_PARAMS
            }
        )

    def _fill(self, response, g: ShelfGeometry) -> None:
        response.levels = [
            ShelfLevel(**asdict(level)) for level in g.levels
        ]  # same fields
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
            obj.pose.header.frame_id = "base_link"
            obj.pose.header.stamp = self._node.get_clock().now().to_msg()
            position = obj.pose.pose.position
            position.x, position.y, position.z = map(float, box.center)
            obj.pose.pose.orientation.w = 1.0
            size = obj.dimensions
            size.x, size.y, size.z = map(float, box.size)
            objects.append(obj)
        return objects

    def _fail(self, response, message: str):
        self._node.get_logger().warning(message)
        response.success = False
        response.message = message
        response.frames_used = len(self._frames)
        return response
