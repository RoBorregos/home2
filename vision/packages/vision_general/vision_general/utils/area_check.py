#!/usr/bin/env python3

import os
import json
from geometry_msgs.msg import PointStamped
from ament_index_python.packages import get_package_share_directory


def load_areas_json():
    """Load the areas.json file from frida_constants/map_areas/areas.json"""
    package_share_directory = get_package_share_directory("frida_constants")
    file_path = os.path.join(package_share_directory, "map_areas/areas.json")
    with open(file_path, "r") as file:
        data = json.load(file)
    return data


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


def is_point_in_room(point_stamped: PointStamped, room_name: str) -> bool:
    """
    Check if a PointStamped is inside a specific room by name.
    Args:
        point_stamped: geometry_msgs.msg.PointStamped
        room_name: str, e.g. 'living_room'
    Returns:
        bool
    """
    areas = load_areas_json()
    if room_name not in areas or "polygon" not in areas[room_name]:
        return False
    polygon = areas[room_name]["polygon"]
    point = (point_stamped.point.x, point_stamped.point.y)
    return point_in_polygon(point, polygon)


def is_point_in_house(point_stamped: PointStamped) -> bool:
    """
    Check if a PointStamped is inside any room in the house.
    Args:
        point_stamped: geometry_msgs.msg.PointStamped
    Returns:
        bool
    """
    areas = load_areas_json()
    point = (point_stamped.point.x, point_stamped.point.y)
    for room, data in areas.items():
        if "polygon" in data and point_in_polygon(point, data["polygon"]):
            return True
    return False
