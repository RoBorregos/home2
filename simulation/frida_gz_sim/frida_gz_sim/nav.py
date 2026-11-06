"""Shared constants for the simulated navigation arena."""

# Arena world generated from the robocup2026_1 occupancy grid, so the Gazebo world
# frame and the navigation map frame are the same and areas_<map>.json applies as is
MAP_NAME = "robocup2026_1"
ARENA_WORLD = f"arena_{MAP_NAME}.sdf"

# start_location/safe_place from areas_robocup2026_1.json, as (x, y, yaw)
SPAWN_POSE = (4.1776, -11.6974, -1.9890)

# Well-clear goals from the same areas file, used by sim_nav_task_manager.py
NAV_ROUTE = [
    ("living_room", "sofa"),
    ("kitchen", "dinner_table"),
    ("laundry", "washing_machine"),
    ("bedroom", "bed"),
    ("start_location", "safe_place"),
]

# Furniture for the base-placement/docking tests (sim_dock_test.py spawns and removes
# it; the arena world itself stays the map). The lidar sits ~0.16 m above the floor,
# so the island is solid down to the floor and the chair shows as four legs.
# Boxes: centre (x, y), yaw, size (sx, sy), height h. Discs: centre, radius r.
TEST_FURNITURE = {
    # Living room, the most open part of the arena (~2 m from any wall)
    "island_table": {
        "type": "box",
        "x": -0.8,
        "y": -9.6,
        "yaw": 0.0,
        "sx": 1.2,
        "sy": 0.8,
        "h": 0.75,
    },
    "chair_leg_1": {
        "type": "box",
        "x": -0.99,
        "y": -8.80,
        "yaw": 0.0,
        "sx": 0.04,
        "sy": 0.04,
        "h": 0.45,
    },
    "chair_leg_2": {
        "type": "box",
        "x": -0.61,
        "y": -8.80,
        "yaw": 0.0,
        "sx": 0.04,
        "sy": 0.04,
        "h": 0.45,
    },
    "chair_leg_3": {
        "type": "box",
        "x": -0.99,
        "y": -9.18,
        "yaw": 0.0,
        "sx": 0.04,
        "sy": 0.04,
        "h": 0.45,
    },
    "chair_leg_4": {
        "type": "box",
        "x": -0.61,
        "y": -9.18,
        "yaw": 0.0,
        "sx": 0.04,
        "sy": 0.04,
        "h": 0.45,
    },
}
# Visual-only seat drawn above the chair legs (above the lidar plane, like a real seat)
TEST_CHAIR_SEAT = {"x": -0.8, "y": -8.99, "sx": 0.44, "sy": 0.44, "z": 0.45}
# Object to pick on the island, right behind the chair
TEST_TARGET = (-0.8, -9.45)

# Round dinner table already in the arena (fit to the map's occupied cells)
ROUND_TABLE = {"x": -0.40, "y": -14.075, "r": 0.44}
