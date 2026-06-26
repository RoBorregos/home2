#!/usr/bin/env python3

import json

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer

from frida_interfaces.srv import MapAreas

from vision_general.utils.calculations import point2d_to_ros_point_stamped


def fetch_map_areas(
    node,
    areas_client,
    logger,
    service_timeout: float = 2.0,
    call_timeout: float = 5.0,
):
    """
    Fetch the active map's areas from nav_central's AREAS_SERVICE.

    Returns the areas dict, or None if the service is unavailable / returns no
    data (callers should treat None as "skip the house filter").

    Spins ``node`` while waiting so the response is processed even when called
    from within another callback on a single-threaded executor.
    """
    if not areas_client.wait_for_service(timeout_sec=service_timeout):
        logger.warn("Areas service not available; skipping house filter.")
        return None

    future = areas_client.call_async(MapAreas.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=call_timeout)
    result = future.result() if future.done() else None
    if result is None or not result.areas:
        logger.warn("Areas service returned no data; skipping house filter.")
        return None

    areas = json.loads(result.areas)
    logger.info(f"Loaded areas for rooms: {list(areas.keys())}")
    return areas


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


def is_point_in_house(point_stamped: PointStamped, areas_json) -> bool:
    return any(
        is_point_in_room(point_stamped, room, areas_json)
        for room in areas_json
        if areas_json.get(room, {}).get("polygon")
    )


def filter_detections_in_house(
    frame,
    detections: list,
    class_ids: list[int],
    rooms: list[str] | None,
    camera_info,
    depth_image,
    tf_buffer: Buffer,
    areas_json: dict,
    camera_frame: str,
    rotation: int = 0,
) -> list:
    """
    Filter detections by class ID and spatial location.

    If rooms is provided, only detections inside one of those rooms are kept.
    If rooms is None, detections inside any room of the hosue will be kept (as long as they have a polygon)
    If spatial data is unavailable, detections matching class_ids pass through unfiltered.
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

        if not spatial_ready:
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

        if rooms:
            if any(is_point_in_room(point_map, room, areas_json) for room in rooms):
                filtered.append(det)
        else:
            if is_point_in_house(point_map, areas_json):
                filtered.append(det)

    return filtered
