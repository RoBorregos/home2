#!/usr/bin/env python3

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer

from vision_general.utils.calculations import point2d_to_ros_point_stamped


def point_in_polygon(point, polygon):
    """Check if a 2D point (x, y) is inside a polygon (list of [x, y] points)"""
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y + 1e-9) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


def is_point_in_room(point_stamped: PointStamped, room_name: str, areas_json) -> bool:
    """
    Check if a PointStamped is inside a specific room by name.
    Args:
        point_stamped: geometry_msgs.msg.PointStamped
        room_name: str, e.g. 'living_room'
    Returns:
        bool
    """
    if room_name not in areas_json or not areas_json.get(room_name, {}).get("polygon"):
        return False
    polygon = areas_json[room_name]["polygon"]
    point = (point_stamped.point.x, point_stamped.point.y)
    return point_in_polygon(point, polygon)


def filter_class(
    frame,
    detections: list,
    class_ids: list[str],
    rooms: list[str] | None,
    camera_info,
    depth_image,
    tf_buffer: Buffer,
    areas_json: dict,
    camera_frame: str,
    rotation: int = 0,
) -> list:
    """
    Return only detections of class_name whose bbox center is inside room_name.

    If spatial data (camera_info / depth_image / tf_buffer / areas_json) is
    unavailable, detections matching class_name pass through without room filtering.
    """
    h, w = frame.shape[:2]
    filtered = []

    spatial_ready = (
        camera_info is not None
        and depth_image is not None
        and tf_buffer is not None
        and areas_json is not None
    )

    for det in detections:
        if det["class_id"] not in class_ids:
            continue

        if not spatial_ready or rooms is None:
            filtered.append(det)
            continue

        x1, y1, x2, y2 = det["bbox"]
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        if not (0 <= cx < w and 0 <= cy < h):
            continue

        point_stamped = point2d_to_ros_point_stamped(
            camera_info,
            depth_image,
            (cx, cy),
            camera_frame,
            Time(sec=0, nanosec=0),
            rotation=rotation,
        )

        try:
            transform = tf_buffer.lookup_transform(
                "map",
                point_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            point_map = do_transform_point(point_stamped, transform)
        except Exception:
            continue

        for room in rooms:
            if is_point_in_room(point_map, room, areas_json):
                filtered.append(det)

    return filtered

