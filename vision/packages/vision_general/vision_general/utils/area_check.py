#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped


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
