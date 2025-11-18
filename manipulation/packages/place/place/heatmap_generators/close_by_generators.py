import numpy as np
from frida_constants.manipulation_constants import (
    CLOSE_BY_MAX_DISTANCE,
)


def generate_close_to_heatmap(
    binary_map,
    close_point,
    min_x_mm=0,
    min_y_mm=0,
    grid_size_mm=10,
    max_distance=CLOSE_BY_MAX_DISTANCE,
):
    """
    Generate a heatmap based on proximity to a close point.
    """
    heatmap = np.zeros_like(binary_map, dtype=float)
    rows, cols = binary_map.shape
    for i in range(rows):
        for j in range(cols):
            if binary_map[i, j] == 1:
                x = (min_x_mm + (i + 0.5) * grid_size_mm) / 1000.0
                y = (min_y_mm + (j + 0.5) * grid_size_mm) / 1000.0
                distance = np.linalg.norm(np.array([x, y]) - np.array(close_point))
                if distance <= max_distance:
                    heatmap[i, j] = max(0, 1 - (distance / max_distance))
    return heatmap


def generate_directional_heatmap(
    binary_map,
    close_point,
    direction,
    min_x_mm=0,
    min_y_mm=0,
    grid_size_mm=10,
    max_distance=CLOSE_BY_MAX_DISTANCE,
):
    """
    Generate a directional heatmap based on proximity to a close point and a specified direction.
    directions: "front", "back", "left", "right"
    close_point: (x, y) coordinates of the close point in meters
    """
    heatmap = np.zeros_like(binary_map, dtype=float)
    rows, cols = binary_map.shape
    direction_vectors = {
        "front": np.array([1, 0]),
        "back": np.array([-1, 0]),
        "left": np.array([0, -1]),
        "right": np.array([0, 1]),
    }
    if direction not in direction_vectors:
        return None
    direction_vector = direction_vectors[direction]
    for i in range(rows):
        for j in range(cols):
            if binary_map[i, j] == 1:
                x = (min_x_mm + (i + 0.5) * grid_size_mm) / 1000.0
                y = (min_y_mm + (j + 0.5) * grid_size_mm) / 1000.0
                distance = np.linalg.norm(np.array([x, y]) - np.array(close_point))
                if distance <= max_distance:
                    point_vector = np.array([x, y]) - np.array(close_point)
                    if np.linalg.norm(point_vector) == 0:
                        angle = 0
                    else:
                        angle = np.arccos(
                            np.clip(
                                np.dot(point_vector, direction_vector)
                                / (
                                    np.linalg.norm(point_vector)
                                    * np.linalg.norm(direction_vector)
                                ),
                                -1.0,
                                1.0,
                            )
                        )
                    heatmap[i, j] = max(
                        0, 1 - (distance / max_distance) * (1 + np.cos(angle))
                    )
    return heatmap
