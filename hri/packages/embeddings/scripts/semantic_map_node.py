#!/usr/bin/env python3
"""Semantic map node.

Listens to vision's object detections, transforms each detection's 3D point
from camera frame into the `map` frame (via the existing PointTransformer
service — no new TF code needed here), tags it with the area it falls in
(reusing frida_constants/map_areas/areas.json, same polygons the nav stack
already tags rooms with), and folds it into the `semantic_objects` table in
Postgres via PostgresAdapter's label+distance dedup.

Also exposes a QuerySemanticMap service to read the map back, and periodically
expires entries that haven't been re-observed in a while.
"""

import json
import os

import rclpy
import rclpy.duration
from ament_index_python.packages import get_package_share_directory
from embeddings.postgres_adapter import PostgresAdapter
from frida_interfaces.msg import (
    ObjectDetectionArray,
    SemanticObject as SemanticObjectMsg,
)
from frida_interfaces.srv import PointTransformation, QuerySemanticMap
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from frida_constants.hri_constants import (
    QUERY_SEMANTIC_MAP_SERVICE,
    SEMANTIC_MAP_MARKERS_TOPIC,
)
from frida_constants.integration_constants import POINT_TRANSFORMER_TOPIC
from frida_constants.vision_constants import DETECTIONS_TOPIC

MAP_FRAME = "map"
AREAS_JSON_RELATIVE_PATH = "map_areas/areas.json"


def point_in_polygon(x: float, y: float, polygon: list) -> bool:
    """Ray-casting point-in-polygon test. Same algorithm
    task_manager/scripts/misc/point_transformer.py uses to resolve the
    robot's own area, applied here to an arbitrary detected point instead of
    only the robot's pose."""
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xinters = (y - y1) * (x2 - x1) / (y2 - y1 + 1e-10) + x1
            if x < xinters:
                inside = not inside
    return inside


class SemanticMapNode(Node):
    def __init__(self):
        super().__init__("semantic_map_node")

        self.declare_parameter("detections_topic", DETECTIONS_TOPIC)
        self.declare_parameter("match_radius", 0.4)
        self.declare_parameter("min_confidence", 0.4)
        # Only fold in every Nth message batch — continuous detections would
        # otherwise hit the DB at camera rate for objects that aren't moving.
        self.declare_parameter("process_every_n", 5)
        # Expiration: how long an object can go un-reobserved before it's
        # dropped, and how often we check for that.
        self.declare_parameter("max_object_age_seconds", 3600.0)
        self.declare_parameter("cleanup_interval_seconds", 300.0)
        # Debug visualization: republish the whole table as RViz markers.
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("markers_publish_interval_seconds", 1.0)

        detections_topic = self.get_parameter("detections_topic").value
        self.match_radius = self.get_parameter("match_radius").value
        self.min_confidence = self.get_parameter("min_confidence").value
        self.process_every_n = self.get_parameter("process_every_n").value
        self.max_object_age_seconds = self.get_parameter("max_object_age_seconds").value
        cleanup_interval = self.get_parameter("cleanup_interval_seconds").value
        self.publish_markers = self.get_parameter("publish_markers").value
        markers_interval = self.get_parameter("markers_publish_interval_seconds").value
        self._msg_count = 0

        self.get_logger().info("Initializing SemanticMapNode...")
        # No embeddings needed: matching is label + euclidean distance.
        self.pg = PostgresAdapter(load_embeddings=False)

        self.areas = self._load_areas()

        self.transform_client = self.create_client(
            PointTransformation, POINT_TRANSFORMER_TOPIC
        )

        self.create_subscription(
            ObjectDetectionArray,
            detections_topic,
            self.detections_callback,
            5,
        )

        self.create_service(
            QuerySemanticMap,
            QUERY_SEMANTIC_MAP_SERVICE,
            self.query_semantic_map_callback,
        )

        self.create_timer(cleanup_interval, self.cleanup_callback)

        if self.publish_markers:
            self.pub_markers = self.create_publisher(
                MarkerArray, SEMANTIC_MAP_MARKERS_TOPIC, 5
            )
            self.create_timer(markers_interval, self.publish_markers_callback)

        self.get_logger().info(
            f"SemanticMapNode ready, listening on '{detections_topic}'"
        )

    def _load_areas(self) -> dict:
        """Load the same areas.json the nav stack tags rooms with, so
        detected objects get an `area` for free instead of raw coordinates."""
        try:
            package_share_directory = get_package_share_directory("frida_constants")
            file_path = os.path.join(package_share_directory, AREAS_JSON_RELATIVE_PATH)
            with open(file_path, "r") as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().warn(
                f"Could not load areas.json, objects won't be area-tagged: {e}"
            )
            return {}

    def _resolve_area(self, x: float, y: float) -> str:
        for area, content in self.areas.items():
            polygon = content.get("polygon", [])
            if polygon and point_in_polygon(x, y, polygon):
                return area
        return ""

    def detections_callback(self, msg: ObjectDetectionArray):
        self._msg_count += 1
        if self._msg_count % self.process_every_n != 0:
            return

        if not msg.detections:
            return

        if not self.transform_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "PointTransformer service not available, skipping this batch"
            )
            return

        for det in msg.detections:
            if det.score < self.min_confidence:
                continue
            self._process_detection(det)

    def _process_detection(self, det):
        transformed = self._transform_to_map(det.point3d)
        if transformed is None:
            return

        x, y, z = transformed.point.x, transformed.point.y, transformed.point.z
        area = self._resolve_area(x, y) or None

        try:
            obj = self.pg.upsert_semantic_object(
                label=det.label_text,
                x=x,
                y=y,
                z=z,
                confidence=float(det.score),
                frame_id=MAP_FRAME,
                area=area,
                match_radius=self.match_radius,
            )
            self.get_logger().debug(
                f"semantic_objects: '{obj.label}' @ "
                f"({obj.x:.2f}, {obj.y:.2f}, {obj.z:.2f}) "
                f"area={obj.area} obs={obj.observations}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to upsert semantic object: {e}")

    def query_semantic_map_callback(self, request, response):
        try:
            label = request.label or None
            area = request.area or None
            objects = self.pg.get_semantic_objects(label=label, area=area)
            response.objects = [
                SemanticObjectMsg(
                    label=o.label,
                    x=o.x,
                    y=o.y,
                    z=o.z,
                    frame_id=o.frame_id,
                    confidence=o.confidence,
                    area=o.area or "",
                    observations=o.observations,
                )
                for o in objects
            ]
            response.success = True
            response.message = f"Found {len(objects)} object(s)"
        except Exception as e:
            self.get_logger().error(f"QuerySemanticMap error: {e}")
            response.success = False
            response.message = str(e)
            response.objects = []
        return response

    def cleanup_callback(self):
        try:
            deleted = self.pg.expire_stale_objects(self.max_object_age_seconds)
            if deleted:
                self.get_logger().info(
                    f"Expired {deleted} stale semantic-map object(s)"
                )
        except Exception as e:
            self.get_logger().error(f"Semantic map cleanup failed: {e}")

    def publish_markers_callback(self):
        """Debug view: one sphere + one text label per known object, in
        RViz's 'map' frame. Add a MarkerArray display on
        SEMANTIC_MAP_MARKERS_TOPIC (/hri/embeddings/semantic_map_markers) to
        an existing rviz config (e.g. navigation/packages/map_context/config/
        rviz/simulation_map.rviz) to see it alongside the nav map."""
        try:
            objects = self.pg.get_semantic_objects()
        except Exception as e:
            self.get_logger().error(
                f"Failed to fetch semantic objects for markers: {e}"
            )
            return

        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        lifetime = rclpy.duration.Duration(
            seconds=self.get_parameter("markers_publish_interval_seconds").value * 2
        ).to_msg()

        for i, obj in enumerate(objects):
            sphere = Marker()
            sphere.header.frame_id = obj.frame_id
            sphere.header.stamp = stamp
            sphere.ns = "semantic_map_objects"
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = obj.x
            sphere.pose.position.y = obj.y
            sphere.pose.position.z = obj.z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
            sphere.color.a = 1.0
            sphere.color.r = 0.1
            sphere.color.g = 0.8
            sphere.color.b = 0.2
            sphere.lifetime = lifetime
            markers.markers.append(sphere)

            label = Marker()
            label.header.frame_id = obj.frame_id
            label.header.stamp = stamp
            label.ns = "semantic_map_labels"
            label.id = i * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = obj.x
            label.pose.position.y = obj.y
            label.pose.position.z = obj.z + 0.2
            label.pose.orientation.w = 1.0
            label.scale.z = 0.15
            label.color.a = 1.0
            label.color.r = label.color.g = label.color.b = 1.0
            area_suffix = f" [{obj.area}]" if obj.area else ""
            label.text = f"{obj.label}{area_suffix} ({obj.observations})"
            label.lifetime = lifetime
            markers.markers.append(label)

        self.pub_markers.publish(markers)

    def _transform_to_map(self, point_stamped):
        request = PointTransformation.Request()
        request.point = point_stamped
        request.target_frame = MAP_FRAME

        future = self.transform_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        result = future.result()

        if result is None:
            self.get_logger().warn("PointTransformer call timed out")
            return None
        if not result.success:
            self.get_logger().warn(f"PointTransformer failed: {result.error_message}")
            return None

        return result.transformed_point


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
