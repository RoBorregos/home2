"""Shelf worlds for checking the model-free shelf detection in Gazebo.

Each scene is a cabinet in front of the static robot (base_link at the world
origin, facing +x) at the stand-off navigation leaves it, 15-20 cm between the
robot front and the shelf. The furniture differs between scenes on purpose (level
count, spacing, depth, walls, top board) so a detector that secretly relies on one
shelf fails here. Every scene comes in a with-objects and an empty variant.

``world_sdf(scene, objects)`` returns the SDF text; ``SCENES[name].surfaces`` is the
ground truth (top of every board, base_link Z) the check compares against.
"""

import math
from dataclasses import dataclass

BOARD = 0.02  # board thickness, m
WALL = 0.018


@dataclass(frozen=True)
class ShelfScene:
    name: str
    boards: tuple  # board TOP surface heights, ascending
    front_x: float = 0.50  # board front edge in base_link
    depth: float = 0.38
    width: float = 0.80
    side_walls: bool = True
    back_wall: bool = True
    top: float = 0.0  # underside of a closed top board above the last level (0 = open)
    yaw_deg: float = 0.0  # cabinet rotation about its front centre: imperfect docking
    y_offset: float = 0.0  # lateral misalignment

    @property
    def surfaces(self) -> list:
        """Every surface an object can rest on, base_link Z (what the scan must find)."""
        tops = list(self.boards)
        if self.top:
            tops.append(self.top + BOARD)
        return tops


SCENES = {
    # The heights of the competition cabinet in shelf_levels.json, with an open top.
    "competition": ShelfScene("competition", boards=(0.095, 0.39, 0.68, 1.05)),
    # Another cabinet: three levels, closed top, deeper and wider, docked further away.
    "closed_three": ShelfScene(
        "closed_three",
        boards=(0.30, 0.62, 0.95),
        top=1.25,
        depth=0.42,
        width=0.90,
        front_x=0.55,
    ),
    # Open bookshelf: no side walls, close and slightly rotated (imperfect dock).
    "open_rotated": ShelfScene(
        "open_rotated",
        boards=(0.12, 0.45, 0.80, 1.15),
        side_walls=False,
        front_x=0.45,
        yaw_deg=4.0,
        y_offset=0.05,
    ),
    # Competition cabinet from the robot's photo: closed tall bookcase, four boards at
    # the shelf_levels.json heights and a closed top ~1.40 m (underside), taller
    # than the camera can look over. Width from the photo's proportions; the depth is
    # ASSUMED (not visible in the photo).
    "competition_tall": ShelfScene(
        "competition_tall",
        boards=(0.095, 0.39, 0.69, 1.05),
        top=1.40,
        depth=0.35,
        width=0.92,
    ),
    # The other competition cabinet: shelves at 0.90 m and ~1.30 m (from the team).
    # Width, depth and the open top are ASSUMED; fix them once measured.
    "competition_high": ShelfScene(
        "competition_high",
        boards=(0.90, 1.30),
        depth=0.35,
        width=0.80,
    ),
}

# Objects per scene: (shape, level index, x from board front, y, size...). Boxes are
# (sx, sy, sz); cylinders are (radius, length). Chosen to stress the detector: tall
# and short objects, a wide flat tray that is a horizontal surface itself, and
# objects near the front edge that hide the board behind them.
OBJECTS = {
    "competition": [
        ("box", 0, 0.10, 0.15, (0.07, 0.19, 0.25)),  # cereal box on the floor level
        ("cylinder", 1, 0.08, -0.20, (0.033, 0.12)),  # can
        ("cylinder", 1, 0.20, 0.05, (0.035, 0.22)),  # bottle
        ("box", 1, 0.12, 0.25, (0.10, 0.10, 0.10)),  # small box
        ("box", 2, 0.15, -0.05, (0.20, 0.30, 0.03)),  # flat tray
        ("cylinder", 2, 0.06, 0.28, (0.045, 0.10)),  # mug-sized
        ("box", 3, 0.10, -0.25, (0.07, 0.19, 0.28)),  # cereal box up high
    ],
    "closed_three": [
        ("box", 0, 0.12, -0.25, (0.08, 0.20, 0.24)),
        ("cylinder", 0, 0.08, 0.10, (0.033, 0.12)),
        ("box", 1, 0.20, 0.00, (0.30, 0.40, 0.02)),  # wide plate stack
        ("cylinder", 2, 0.10, 0.30, (0.035, 0.20)),
        ("box", 2, 0.06, -0.10, (0.12, 0.12, 0.12)),
    ],
    "competition_tall": [
        ("box", 0, 0.10, 0.15, (0.07, 0.19, 0.25)),
        ("cylinder", 1, 0.08, -0.20, (0.033, 0.12)),
        ("box", 2, 0.15, -0.05, (0.20, 0.30, 0.03)),
        ("box", 3, 0.10, -0.25, (0.07, 0.19, 0.24)),
        ("cylinder", 3, 0.20, 0.20, (0.035, 0.20)),
        ("box", 1, 0.12, 0.25, (0.10, 0.10, 0.10)),
    ],
    "competition_high": [
        ("cylinder", 0, 0.08, -0.22, (0.033, 0.12)),  # can
        ("box", 0, 0.15, 0.24, (0.07, 0.19, 0.25)),  # cereal box
        ("box", 1, 0.10, -0.20, (0.10, 0.10, 0.10)),  # small box
        ("cylinder", 1, 0.20, 0.22, (0.045, 0.10)),  # mug-sized
    ],
    "open_rotated": [
        ("cylinder", 0, 0.10, -0.15, (0.035, 0.22)),
        ("box", 1, 0.05, 0.10, (0.07, 0.19, 0.25)),  # right at the front edge
        ("cylinder", 2, 0.15, -0.20, (0.033, 0.12)),
        ("box", 3, 0.10, 0.20, (0.15, 0.25, 0.04)),
    ],
}


def _box_link(name, pose, size, colour):
    x, y, z = pose
    sx, sy, sz = size
    return f"""
      <collision name="{name}_c"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>
      <visual name="{name}_v"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>"""


def _cabinet(scene: ShelfScene) -> str:
    """Static cabinet model; its frame is the centre of the front edge at the floor."""
    wood = "0.55 0.40 0.25 1"
    parts = []
    d, w = scene.depth, scene.width
    for i, top in enumerate(scene.boards):
        parts.append(
            _box_link(f"board_{i}", (d / 2, 0.0, top - BOARD / 2), (d, w, BOARD), wood)
        )
    height = (scene.top + BOARD) if scene.top else scene.boards[-1]
    if scene.top:
        parts.append(
            _box_link("top", (d / 2, 0.0, scene.top + BOARD / 2), (d, w, BOARD), wood)
        )
    if scene.side_walls:
        for side, sign in (("left", 1), ("right", -1)):
            parts.append(
                _box_link(
                    f"wall_{side}",
                    (d / 2, sign * (w / 2 + WALL / 2), height / 2),
                    (d, WALL, height),
                    wood,
                )
            )
    else:  # an open bookshelf still stands on four thin posts
        for i, (px, py) in enumerate(
            (
                (0.015, w / 2 + 0.015),
                (0.015, -w / 2 - 0.015),
                (d - 0.015, w / 2 + 0.015),
                (d - 0.015, -w / 2 - 0.015),
            )
        ):
            parts.append(
                _box_link(f"post_{i}", (px, py, height / 2), (0.03, 0.03, height), wood)
            )
    if scene.back_wall:
        parts.append(
            _box_link(
                "wall_back",
                (d + WALL / 2, 0.0, height / 2),
                (WALL, w + 2 * WALL, height),
                wood,
            )
        )
    yaw = math.radians(scene.yaw_deg)
    return f"""
    <model name="shelf">
      <static>true</static>
      <pose>{scene.front_x:.4f} {scene.y_offset:.4f} 0 0 0 {yaw:.5f}</pose>
      <link name="link">{"".join(parts)}
      </link>
    </model>"""


def _objects(scene: ShelfScene) -> str:
    yaw = math.radians(scene.yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    colours = ["0.8 0.2 0.2 1", "0.2 0.6 0.9 1", "0.9 0.8 0.2 1", "0.3 0.8 0.3 1"]
    models = []
    for i, (shape, level, dx, dy, size) in enumerate(OBJECTS[scene.name]):
        z0 = scene.boards[level]
        x = scene.front_x + c * dx - s * dy
        y = scene.y_offset + s * dx + c * dy
        colour = colours[i % len(colours)]
        if shape == "box":
            half = size[2] / 2
            geometry = f"<box><size>{size[0]} {size[1]} {size[2]}</size></box>"
        else:
            half = size[1] / 2
            geometry = f"<cylinder><radius>{size[0]}</radius><length>{size[1]}</length></cylinder>"
        models.append(
            f"""
    <model name="shelf_object_{i}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z0 + half + 0.0005:.4f} 0 0 {yaw:.5f}</pose>
      <link name="link">
        <collision name="c"><geometry>{geometry}</geometry></collision>
        <visual name="v"><geometry>{geometry}</geometry>
          <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>
      </link>
    </model>"""
        )
    return "".join(models)


# Graspable object for the shelf pick check: a real (dynamic) model from
# objects.GRASPABLE_OBJECTS, so grasp assist can weld it and COCO YOLO sees a "bottle".
PICK_MODEL = ("mustard_bottle", "sim_mustard_bottle")
PICK_LEVEL = 1  # default: index into the boards, the second shelf
PICK_DX = 0.10  # object centre from the board front
# Object footprint (the mustard box, yaw 0: 9 cm deep, 6 cm wide) plus what the open
# fingers sweep beside it.
PICK_HALF_DEPTH = 0.045
PICK_HALF_SWEEP = 0.03 + 0.06
PICK_WALL_MARGIN = 0.12
PICK_MIN_GAP = 0.03  # free space beside the open fingers


def _clutter_footprint(item) -> tuple:
    """(x_min, x_max, y_min, y_max) of one clutter object, shelf frame."""
    shape, _, dx, dy, size = item
    hx, hy = (size[0] / 2, size[1] / 2) if shape == "box" else (size[0], size[0])
    return dx - hx, dx + hx, dy - hy, dy + hy


def pick_dy(scene: ShelfScene, level: int, objects: bool = True) -> float:
    """Most central lateral spot on ``level`` with PICK_MIN_GAP to its clutter.

    Falls back to the spot with the most clearance when none has enough.
    """
    clutter = [
        _clutter_footprint(item)
        for item in OBJECTS.get(scene.name, [])
        if objects and item[1] == level
    ]
    limit = scene.width / 2 - PICK_WALL_MARGIN
    best, best_gap = 0.0, -math.inf
    for step in range(int(limit / 0.02) + 1):
        for dy in (step * 0.02, -step * 0.02):
            gap = math.inf  # lateral clearance to the nearest clutter beside it
            for x_min, x_max, y_min, y_max in clutter:
                if (
                    x_max < PICK_DX - PICK_HALF_DEPTH
                    or x_min > PICK_DX + PICK_HALF_DEPTH
                ):
                    continue  # fully in front of or behind the object
                side = max(
                    y_min - (dy + PICK_HALF_SWEEP), (dy - PICK_HALF_SWEEP) - y_max
                )
                gap = min(gap, side)
            if gap >= PICK_MIN_GAP:
                return dy  # candidates come centre first
            if gap > best_gap + 1e-9:
                best, best_gap = dy, gap
    return best


def pick_object_pose(
    scene: ShelfScene, level: int = PICK_LEVEL, objects: bool = True
) -> tuple:
    """World (x, y, z) where the graspable object starts on ``level`` (board index)."""
    yaw = math.radians(scene.yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    dy = pick_dy(scene, level, objects)
    x = scene.front_x + c * PICK_DX - s * dy
    y = scene.y_offset + s * PICK_DX + c * dy
    return x, y, scene.boards[level] + 0.001


def _pick_object(scene: ShelfScene, level: int, objects: bool) -> str:
    name, model = PICK_MODEL
    x, y, z = pick_object_pose(scene, level, objects)
    yaw = math.radians(scene.yaw_deg)
    return f"""
    <include>
      <name>{name}</name>
      <uri>model://{model}</uri>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}</pose>
    </include>"""


def world_sdf(
    scene: ShelfScene, objects: bool, pick: bool = False, pick_level: int = PICK_LEVEL
) -> str:
    """Full SDF world: lights, floor, overview camera, the cabinet and its objects.

    The clutter objects are static: they test perception, and static models keep
    physics from shuffling them between runs. ``pick`` adds one dynamic graspable
    object on board ``pick_level`` (index), clear of that level's clutter, and names
    the world like objects.WORLD_NAME, which grasp assist needs.
    """
    world = "pnp_table" if pick else "pnp_shelf"
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{world}">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.85 0.9 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.7 0.7 0.68 1</ambient><diffuse>0.75 0.75 0.72 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="overview_camera">
      <static>true</static>
      <pose>0.30 -1.70 1.25 0 0.20 1.5708</pose>
      <link name="link">
        <sensor name="overview" type="camera">
          <always_on>true</always_on>
          <update_rate>2</update_rate>
          <topic>overview_camera/image</topic>
          <camera>
            <horizontal_fov>1.1</horizontal_fov>
            <image><width>800</width><height>600</height></image>
            <clip><near>0.05</near><far>20</far></clip>
          </camera>
        </sensor>
      </link>
    </model>{_cabinet(scene)}{_objects(scene) if objects else ""}{_pick_object(scene, pick_level, objects) if pick else ""}
  </world>
</sdf>
"""
