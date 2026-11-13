#!/usr/bin/env python3
"""Semantic map: where every object the robot has seen is, in the map frame.

The ROS side of the map. Fusion, geometry and queries live in the Semantic-Map
library (third_party/semantic_map); this node feeds it detections, embeds their
crops through finals_commands and answers queries.

Detections only arrive while the detector runs, so the node forgets nothing when
the robot looks away: an object is dropped only after being looked at, with the
camera still, and not detected (see `_report_unseen`).
"""

import json
import math
import os
import sys
import threading
from pathlib import Path

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import utils
from ament_index_python.packages import get_package_share_directory
from frida_constants.navigation_constants import AREAS_SERVICE
from frida_constants.vision_constants import (
    DETECTIONS_TOPIC,
    EMBED_CROPS_SERVICE,
    EMBED_TEXT_SERVICE,
    SEMANTIC_GET_MISPLACED_SERVICE,
    SEMANTIC_MARKERS_TOPIC,
    SEMANTIC_QUERY_OBJECTS_SERVICE,
    SEMANTIC_RESET_SERVICE,
    SEMANTIC_RESOLVE_COMMAND_SERVICE,
)
from frida_constants.hri_constants import LLM_WRAPPER_SERVICE
from frida_interfaces.msg import ObjectDetectionArray, SemanticObject
from frida_interfaces.srv import EmbedCrops, GetMisplaced, QueryObjects, ResolveCommand
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from vision_runtime import VisionRuntime, spin

# The library's modules have generic names (core, ports, adapters), so they live in
# their own directory and only this node puts it on the path.
sys.path.insert(0, str(Path(utils.__file__).parent.parent / "semantic_map_lib"))

from adapters.files import load_snapshot, save_snapshot  # noqa: E402
from adapters.ros import RosClipEncoder, RosClock, RosLLM, load_areas  # noqa: E402
from core.config import SemanticMapConfig  # noqa: E402
from core.graph import SemanticGraph  # noqa: E402
from core.types import Observation, Point  # noqa: E402
from core.visibility import should_be_visible  # noqa: E402

MAP_FRAME = "map"
EMBED_TIMEOUT_S = 2.0
TF_TIMEOUT_S = 0.2
SNAPSHOT_NAME = "snapshot.json"
# The object detector prefixes labels of the category it was told to look for.
CATEGORY_PREFIX = "trash/"


def apply_transform(transform, point):
    """A (x, y, z) point through a geometry_msgs/TransformStamped."""
    t, q = transform.transform.translation, transform.transform.rotation
    p = np.asarray(point, dtype=float)
    u = np.array([q.x, q.y, q.z])
    rotated = p + 2.0 * np.cross(u, np.cross(u, p) + q.w * p)
    return float(rotated[0] + t.x), float(rotated[1] + t.y), float(rotated[2] + t.z)


class SemanticMapNode(VisionRuntime):
    def __init__(self):
        super().__init__(
            "semantic_map_node",
            use_depth=True,
            use_camera_info=True,
            use_tf=True,
            track_rotation=True,
        )
        self.map_name = os.environ.get("MAP_NAME", "").replace(".db", "")
        self.max_period_s = 1.0 / self._param("max_rate_hz", 2.0)
        self.still_threshold_m = self._param("still_threshold_m", 0.02)
        self.max_range_m = self._param("max_range_m", 2.0)
        self.min_height_m = self._param("min_height_m", -0.1)
        self.max_height_m = self._param("max_height_m", 2.5)
        self.config = self._build_config()
        self.snapshot_dir = self._resolve_snapshot_dir(self._param("snapshot_dir", ""))

        self.object_to_category = self._load_categories()
        self.graph = None
        self.graph_lock = threading.Lock()
        self.dirty = False
        self.last_batch_time = 0.0
        self.last_camera_pose = None

        self.embed_crops_client = self.create_client(
            EmbedCrops, EMBED_CROPS_SERVICE, callback_group=self.callback_group
        )
        # One batch at a time: while one waits on CLIP, newer ones are simply dropped.
        self.create_subscription(
            ObjectDetectionArray,
            DETECTIONS_TOPIC,
            self.detections_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_service(
            QueryObjects,
            SEMANTIC_QUERY_OBJECTS_SERVICE,
            self.query_objects_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            ResolveCommand,
            SEMANTIC_RESOLVE_COMMAND_SERVICE,
            self.resolve_command_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            GetMisplaced,
            SEMANTIC_GET_MISPLACED_SERVICE,
            self.get_misplaced_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            SEMANTIC_RESET_SERVICE,
            self.reset_callback,
            callback_group=self.callback_group,
        )
        self.marker_pub = self.create_publisher(MarkerArray, SEMANTIC_MARKERS_TOPIC, 1)

        self.create_timer(
            self._param("snapshot_period_s", 10.0),
            self._snapshot_timer,
            callback_group=self.callback_group,
        )
        self.create_timer(
            self._param("marker_period_s", 1.0),
            self._marker_timer,
            callback_group=self.callback_group,
        )
        # Not in __init__: services only answer once the executor is spinning.
        self._startup_timer = self.create_timer(
            self._param("startup_retry_s", 5.0),
            self._startup,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.get_logger().info(f"Semantic map up (map='{self.map_name or 'unset'}')")

    # ------------------------------------------------------------------- setup

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def _build_config(self):
        config = SemanticMapConfig(
            confidence_min=self._param("confidence_min", 0.4),
            merge_dist_m=self._param("merge_dist_m", 0.25),
            merge_sim_min=self._param("merge_sim_min", 0.8),
            confirm_min_observations=self._param("confirm_min_observations", 3),
            miss_limit=self._param("miss_limit", 4),
            surface_standoff_m=self._param("surface_standoff_m", 0.8),
            room_margin_m=self._param("room_margin_m", 0.6),
            max_sublocation_distance_m=self._param("max_sublocation_distance_m", 1.5),
            sim_min=self._param("sim_min", 0.24),
            name_weight=self._param("name_weight", 0.5),
        )
        rules = self._param("category_to_furniture_json", "")
        if rules:
            config.category_to_furniture = json.loads(rules)
        return config

    def _resolve_snapshot_dir(self, configured):
        """First writable of: the parameter, /workspace/log, ~/.ros. One directory per
        map, because the embeddings are saved next to the snapshot."""
        candidates = [configured] if configured else []
        candidates += [
            "/workspace/log/semantic_map",
            os.path.expanduser("~/.ros/semantic_map"),
        ]
        for base in candidates:
            directory = os.path.join(base, self.map_name or "default")
            try:
                os.makedirs(directory, exist_ok=True)
                if os.access(directory, os.W_OK):
                    return directory
            except OSError:
                continue
        self.get_logger().error("No writable snapshot directory; running memory-only")
        return ""

    def _load_categories(self):
        path = os.path.join(
            get_package_share_directory("frida_constants"), "data/objects.json"
        )
        try:
            with open(path) as handle:
                return json.load(handle).get("object_to_category", {})
        except (OSError, ValueError) as error:
            self.get_logger().error(
                f"No object categories, misplaced is disabled: {error}"
            )
            return {}

    def _startup(self):
        """Load the active map and any snapshot of it, retrying until both are possible."""
        try:
            areas = load_areas(self, AREAS_SERVICE)
            graph = self._load_graph()
        except (TimeoutError, RuntimeError) as error:
            self.get_logger().warn(f"Waiting for the map and CLIP: {error}")
            return
        except ValueError as error:
            self.get_logger().error(f"{AREAS_SERVICE} returned invalid JSON: {error}")
            return
        graph.load_layout(areas)
        graph.map_name = self.map_name
        rooms = [room.name for room in graph.rooms.values() if room.polygon]
        if not rooms:
            self.get_logger().warn(f"Waiting for tagged rooms in map '{self.map_name}'")
            return
        with self.graph_lock:
            self.graph = graph
        self._startup_timer.cancel()
        self.get_logger().info(
            f"Map loaded: rooms {rooms}, {len(graph.objects)} objects restored"
        )

    def _load_graph(self):
        """A graph restored from this map's snapshot, or an empty one.

        TimeoutError from CLIP propagates, so startup retries instead of throwing
        the snapshot away. A broken file only costs the old objects.
        """
        adapters = dict(
            config=self.config,
            encoder=RosClipEncoder(self, EMBED_TEXT_SERVICE),
            llm=RosLLM(self, LLM_WRAPPER_SERVICE),
            clock=RosClock(self),
        )
        path = (
            os.path.join(self.snapshot_dir, SNAPSHOT_NAME) if self.snapshot_dir else ""
        )
        if path and os.path.exists(path):
            try:
                snapshot = load_snapshot(path, self.config)
                if snapshot.map_name == self.map_name:
                    return SemanticGraph.from_snapshot(snapshot, **adapters)
                self.get_logger().warn(
                    f"Ignoring snapshot of map '{snapshot.map_name}'"
                )
            except (ValueError, KeyError, TypeError, OSError) as error:
                self.get_logger().warn(f"Ignoring unreadable snapshot: {error}")
        return SemanticGraph(**adapters)

    # ------------------------------------------------------------- image input

    def _image_callback(self, msg):
        """Crops are embedded from finals_commands' buffer: no frame is needed here."""

    # -------------------------------------------------------------- detections

    def detections_callback(self, msg):
        if self.graph is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        # A clock that jumped back (sim restart) must not stop the map forever.
        if 0.0 <= now - self.last_batch_time < self.max_period_s:
            return
        self.last_batch_time = now

        detections = [d for d in msg.detections if self._is_usable(d)]
        embeddings = self._embed_crops(msg.header.stamp, detections)

        camera_frame = self._camera_frame(msg, detections)
        to_map = (
            self._lookup(MAP_FRAME, camera_frame, msg.header.stamp)
            if camera_frame
            else None
        )
        if camera_frame and to_map is None:
            self.get_logger().warn(
                f"No TF {camera_frame} -> {MAP_FRAME}; batch dropped",
                throttle_duration_sec=5.0,
            )
            return

        seen = set()
        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        for detection, embedding in zip(detections, embeddings):
            p = detection.point3d.point
            position = apply_transform(to_map, (p.x, p.y, p.z))
            if not self.min_height_m <= position[2] <= self.max_height_m:
                continue
            label = detection.label_text.removeprefix(CATEGORY_PREFIX)
            observation = Observation(
                label=label,
                position=Point(*position),
                stamp=stamp,
                confidence=float(detection.score),
                category=self.object_to_category.get(label.strip().lower()),
                embedding=embedding,
            )
            try:
                with self.graph_lock:
                    object_id = self.graph.observe(observation)
            except (TimeoutError, RuntimeError) as error:
                self.get_logger().warn(
                    f"Skipping '{label}': {error}", throttle_duration_sec=5.0
                )
                continue
            if object_id:
                seen.add(object_id)
                self.dirty = True

        self._report_unseen(msg.header.stamp, seen)

    @staticmethod
    def _is_usable(detection):
        """Without depth the detector leaves the point at the camera's origin."""
        p = detection.point3d.point
        return (
            bool(detection.point3d.header.frame_id)
            and math.isfinite(p.x + p.y + p.z)
            and (p.x, p.y, p.z) != (0.0, 0.0, 0.0)
            and detection.xmax > detection.xmin
            and detection.ymax > detection.ymin
        )

    def _embed_crops(self, stamp, detections):
        """One unit vector per detection; all None if CLIP cannot embed this batch.

        The map still works then: objects fall back to their label's text.
        """
        missing = [None] * len(detections)
        if not detections or not self.embed_crops_client.service_is_ready():
            return missing
        request = EmbedCrops.Request(stamp=stamp, detections=detections)
        response = self.call_service(self.embed_crops_client, request, EMBED_TIMEOUT_S)
        if response is None or not response.success or response.dim <= 0:
            return missing
        vectors = np.asarray(response.embeddings, np.float32).reshape(-1, response.dim)
        return list(vectors) if len(vectors) == len(detections) else missing

    def _camera_frame(self, msg, detections):
        """The optical frame of this batch; an empty one takes it from the camera info."""
        if detections:
            return detections[0].point3d.header.frame_id
        return self.camera_info.header.frame_id if self.camera_info is not None else ""

    def _lookup(self, target, source, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT_S),
            )
        except Exception:  # tf2 raises several unrelated types
            return None

    def _report_unseen(self, stamp, seen):
        """Count a miss for confirmed objects the camera should have detected and did not.

        Only with the camera still: motion blur also makes the detector miss, and the
        depth image is not the batch's frame. An object is not gone because the robot
        drove past it.
        """
        info, depth = self.camera_info, self.depth_image
        if info is None or depth is None or self.rotation != 0:
            return
        if depth.shape[:2] != (info.height, info.width):
            return
        from_map = self._lookup(info.header.frame_id, MAP_FRAME, stamp)
        if from_map is None:
            return
        pose = (
            from_map.transform.translation.x,
            from_map.transform.translation.y,
            from_map.transform.translation.z,
            from_map.transform.rotation.x,
            from_map.transform.rotation.y,
            from_map.transform.rotation.z,
            from_map.transform.rotation.w,
        )
        previous, self.last_camera_pose = self.last_camera_pose, pose
        if previous is None or math.dist(previous, pose) > self.still_threshold_m:
            return

        with self.graph_lock:
            unseen = []
            for obj in self.graph.known_objects():
                if obj.id in seen:
                    continue
                point = apply_transform(
                    from_map, (obj.position.x, obj.position.y, obj.position.z)
                )
                if should_be_visible(
                    point,
                    fx=info.k[0],
                    fy=info.k[4],
                    cx=info.k[2],
                    cy=info.k[5],
                    width=info.width,
                    height=info.height,
                    depth=depth,
                    max_range_m=self.max_range_m,
                ):
                    unseen.append(obj.id)
            forgotten = self.graph.report_unseen(unseen)
        if unseen:
            self.dirty = True
        if forgotten:
            self.get_logger().info(f"Forgot {forgotten}: not there anymore")

    # ---------------------------------------------------------------- services

    def _to_msg(self, obj):
        position = PointStamped()
        position.header.frame_id = MAP_FRAME
        position.point.x = float(obj.position.x)
        position.point.y = float(obj.position.y)
        position.point.z = float(obj.position.z)
        return SemanticObject(
            id=obj.id,
            label=obj.label,
            category=obj.category or "",
            position=position,
            area=obj.area,
            sublocation=obj.sublocation,
            confidence=float(obj.confidence),
            last_seen=float(obj.last_seen),
            observations=int(obj.observation_count),
        )

    def query_objects_callback(self, request, response):
        if self.graph is None:
            response.error = "map not loaded yet"
            return response
        now = self.get_clock().now().nanoseconds / 1e9
        with self.graph_lock:
            response.objects = [
                self._to_msg(o)
                for o in self.graph.known_objects()
                if request.label in ("", o.label)
                and request.category in ("", o.category or "")
                and request.area in ("", o.area)
                and request.sublocation in ("", o.sublocation)
                and (request.max_age_s <= 0.0 or now - o.last_seen <= request.max_age_s)
                and o.confidence >= request.min_confidence
            ]
        response.success = True
        return response

    def resolve_command_callback(self, request, response):
        if self.graph is None:
            response.error = "map not loaded yet"
            return response
        try:
            # Outside the lock: the LLM takes seconds and detections must keep flowing.
            parsed = self.graph.parse(request.text)
            with self.graph_lock:
                answer = self.graph.answer(request.text, parsed)
                response.objects = [self._to_msg(m.obj) for m in answer.matches]
            response.status = answer.status
            response.success = True
        except (TimeoutError, RuntimeError) as error:
            response.error = str(error)
            self.get_logger().error(f"Resolve_command -> {error}")
        return response

    def get_misplaced_callback(self, request, response):
        if self.graph is None:
            response.error = "map not loaded yet"
            return response
        with self.graph_lock:
            for found in self.graph.misplaced():
                response.objects.append(
                    self._to_msg(self.graph.objects[found.object_id])
                )
                response.expected_areas.append(found.expected_area)
                response.expected_sublocations.append(found.expected_sublocation)
        response.success = True
        return response

    def reset_callback(self, request, response):
        if self.graph is None:
            response.message = "map not loaded yet"
            return response
        with self.graph_lock:
            self.graph.reset_objects()
            self.dirty = True
        response.success = True
        self.get_logger().info("Map reset")
        return response

    # ------------------------------------------------------------------ timers

    def _snapshot_timer(self):
        if self.dirty:
            self.save()

    def save(self):
        if self.graph is None or not self.snapshot_dir:
            return
        try:
            with self.graph_lock:
                self.dirty = False
                save_snapshot(
                    self.graph.snapshot(),
                    os.path.join(self.snapshot_dir, SNAPSHOT_NAME),
                )
        except (OSError, ValueError, TimeoutError, RuntimeError) as error:
            self.dirty = True
            self.get_logger().warn(f"Could not save the snapshot: {error}")

    def _marker_timer(self):
        if self.graph is None or self.marker_pub.get_subscription_count() == 0:
            return
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        with self.graph_lock:
            for index, obj in enumerate(self.graph.known_objects()):
                markers.markers.extend(self._object_markers(index, obj, stamp))
        self.marker_pub.publish(markers)

    @staticmethod
    def _object_markers(index, obj, stamp):
        sphere = Marker()
        sphere.header.frame_id = MAP_FRAME
        sphere.header.stamp = stamp
        sphere.ns = "objects"
        sphere.id = index
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(obj.position.x)
        sphere.pose.position.y = float(obj.position.y)
        sphere.pose.position.z = float(obj.position.z)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.1
        sphere.color.a = 1.0
        sphere.color.g = 0.8
        sphere.color.b = 0.2

        label = Marker()
        label.header = sphere.header
        label.ns = "labels"
        label.id = index
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = sphere.pose.position.x
        label.pose.position.y = sphere.pose.position.y
        label.pose.position.z = sphere.pose.position.z + 0.15
        label.pose.orientation.w = 1.0
        label.scale.z = 0.1
        label.color.a = label.color.r = label.color.g = label.color.b = 1.0
        place = "/".join(part for part in (obj.area, obj.sublocation) if part)
        label.text = f"{obj.label} [{place}] x{obj.observation_count}"
        return [sphere, label]


def main(args=None):
    rclpy.init(args=args)
    spin(SemanticMapNode())


if __name__ == "__main__":
    main()
