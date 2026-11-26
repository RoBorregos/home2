"""DetectDoorHandle (approach_door on fresh depth), hosted by grasp_generator."""

import threading
from collections import Counter
from dataclasses import asdict

import numpy as np
import rclpy
from frida_constants.manipulation_constants import (
    DETECT_DOOR_HANDLE_SERVICE,
    ZED_DEPTH_SCALE,
)
from frida_interfaces.srv import DetectDoorHandle
from rclpy.duration import Duration
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException

from perception_3d.approach_door import (
    HandleNotFound,
    HandleParams,
    deproject,
    detect_handle,
)

PARAM_PREFIX = "door."
BASE_FRAME = "base_link"  # the HandleParams frame
FRESH_FRAME_TIMEOUT = 2.0  # s for a depth frame newer than the request
TF_TIMEOUT = 1.0
# HandleParams fields exposed as door.* parameters; the rest keep their defaults.
HANDLE_PARAMS = (
    "max_x",
    "y_half",
    "z_min",
    "z_max",
    "length_min",
    "length_max",
    "height_min",
    "height_max",
    "proud_min",
    "proud_max",
    "standoff_min",
    "standoff_max",
)
NODE_PARAMS = {
    "frames": 3,  # frames per request when the caller passes 0
    "depth_scale": float(ZED_DEPTH_SCALE),  # 1.0 for a simulated camera
    "dump_path": "",  # non-empty: save each request's frames there (.npz)
}


class DoorService:
    """Serves DetectDoorHandle on ``node`` with frames from ``fresh_depth``."""

    def __init__(self, node, tf_buffer, callback_group, fresh_depth, intrinsics):
        self._node = node
        self._tf_buffer = tf_buffer
        self._fresh_depth = fresh_depth  # (timeout) -> (image, frame_id) or None
        self._intrinsics = intrinsics  # () -> object with fx, fy, cx, cy, or None
        self._busy = threading.Lock()

        defaults = asdict(HandleParams())
        tuned = {name: defaults[name] for name in HANDLE_PARAMS}
        for name, value in {**NODE_PARAMS, **tuned}.items():
            node.declare_parameter(PARAM_PREFIX + name, value)
        node.create_service(
            DetectDoorHandle,
            DETECT_DOOR_HANDLE_SERVICE,
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
            self._node.get_logger().error(f"detect_door_handle failed: {error!r}")
            return self._fail(response, f"internal error: {error}")
        finally:
            self._busy.release()

    def _handle(self, request, response):
        intrinsics = self._intrinsics()
        if intrinsics is None:
            return self._fail(response, "camera intrinsics not received yet")
        wanted = request.frames if request.frames > 0 else int(self._param("frames"))
        params = self._handle_params()
        detected = None
        if request.seed.header.frame_id:
            try:
                seed_to_base = self._lookup(
                    BASE_FRAME, request.seed.header.frame_id, TF_TIMEOUT
                )
            except TransformException:
                return self._fail(
                    response,
                    f"no TF {request.seed.header.frame_id} -> {BASE_FRAME} for the detected point",
                )
            seed = request.seed.point
            detected = (seed_to_base @ [seed.x, seed.y, seed.z, 1.0])[:3]
            self._node.get_logger().info(
                f"Door handle detected at {np.round(detected, 3).tolist()} {BASE_FRAME}: segmenting there"
            )
        handles, rejections, frames = [], Counter(), []
        for _ in range(wanted):
            depth = self._fresh_depth(FRESH_FRAME_TIMEOUT)
            if depth is None:
                rejections["no fresh depth frame"] += 1
                continue
            image, frame_id = depth
            try:
                cam_to_base = self._lookup(BASE_FRAME, frame_id, TF_TIMEOUT)
            except TransformException:
                rejections[f"no TF {frame_id} -> {BASE_FRAME}"] += 1
                continue
            image = np.where(np.isfinite(image) & (image > 0), image, np.nan)
            frames.append((image, cam_to_base))
            points = deproject(
                image, intrinsics, cam_to_base, float(self._param("depth_scale"))
            )
            try:
                handles.append(
                    detect_handle(
                        points,
                        image,
                        params,
                        detected=detected,
                        origin=cam_to_base[:3, 3],
                    )
                )
            except HandleNotFound as why:
                # Count frames per reason, not pieces.
                rejections[str(why).split(" (x")[0]] += 1
        self._dump(frames, intrinsics)
        response.frames_used = wanted
        response.frames_found = len(handles)
        if not handles:
            reason, count = (
                rejections.most_common(1)[0] if rejections else ("no frame", 0)
            )
            return self._fail(
                response,
                f"no door handle in {wanted} frame(s): {reason} ({count}/{wanted})",
            )
        self._fill(response, handles)
        self._node.get_logger().info(response.message)
        return response

    def _fill(self, response, handles) -> None:
        """Per-field median of the frames that found the handle."""
        stamp = self._node.get_clock().now().to_msg()
        center = np.median([h.center for h in handles], axis=0)
        normal = np.median([h.normal for h in handles], axis=0)
        normal /= np.linalg.norm(normal)
        door_point = np.median([h.door_point for h in handles], axis=0)
        for msg, value in (
            (response.center, center),
            (response.door_point, door_point),
        ):
            msg.header.frame_id, msg.header.stamp = BASE_FRAME, stamp
            msg.point.x, msg.point.y, msg.point.z = map(float, value)
        response.door_normal.x, response.door_normal.y, response.door_normal.z = map(
            float, normal
        )
        for name in ("length", "thickness", "standoff", "residual"):
            setattr(
                response, name, float(np.median([getattr(h, name) for h in handles]))
            )
        response.success = True
        yaw = np.degrees(np.arctan2(-normal[1], -normal[0]))
        response.message = (
            f"door handle at ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}) {BASE_FRAME}: length {response.length * 1000:.0f} mm, "
            f"thickness {response.thickness * 1000:.0f} mm, stand-off {response.standoff * 1000:.0f} mm, door yaw {yaw:.1f} deg, "
            f"fit {response.residual * 1000:.1f} mm, {response.frames_found}/{response.frames_used} frames"
        )

    def _dump(self, frames, intrinsics) -> None:
        path = self._param("dump_path")
        if not path or not frames:
            return
        np.savez_compressed(
            path,
            K=np.array([intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy]),
            **{f"depth_{i}": image for i, (image, _) in enumerate(frames)},
            **{f"cam_to_base_{i}": matrix for i, (_, matrix) in enumerate(frames)},
        )
        self._node.get_logger().info(f"Dumped {len(frames)} door frame(s) to {path}")

    def _lookup(self, target: str, source: str, timeout: float) -> np.ndarray:
        transform = self._tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(), timeout=Duration(seconds=timeout)
        ).transform
        r, t = transform.rotation, transform.translation
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix

    def _handle_params(self) -> HandleParams:
        defaults = asdict(HandleParams())
        return HandleParams(
            **{name: type(defaults[name])(self._param(name)) for name in HANDLE_PARAMS}
        )

    def _fail(self, response, message: str):
        self._node.get_logger().warning(message)
        response.success = False
        response.message = message
        return response
