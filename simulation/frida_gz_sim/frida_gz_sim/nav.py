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
