"""Rulebook objects as primitives: shape, size (m), mass (kg) and colour."""

import json
import os

from ament_index_python.packages import get_package_share_directory

# name -> (shape, size, mass, rgb); cylinder size = (radius, height), box size = (x, y, height)
OBJECTS = {
    "cup": ("cylinder", (0.04, 0.10), 0.20, (0.95, 0.95, 0.95)),
    "bowl": ("cylinder", (0.07, 0.07), 0.25, (0.30, 0.55, 0.90)),
    "plate": ("cylinder", (0.11, 0.02), 0.30, (0.90, 0.90, 0.80)),
    "spoon": ("box", (0.15, 0.03, 0.015), 0.04, (0.75, 0.75, 0.78)),
    "fork": ("box", (0.17, 0.025, 0.015), 0.04, (0.70, 0.70, 0.74)),
    "knife": ("box", (0.20, 0.02, 0.012), 0.05, (0.65, 0.65, 0.70)),
    "apple": ("cylinder", (0.04, 0.075), 0.15, (0.85, 0.10, 0.10)),
    "peach": ("cylinder", (0.035, 0.065), 0.12, (1.00, 0.60, 0.45)),
    "lemon": ("cylinder", (0.03, 0.06), 0.10, (0.95, 0.90, 0.15)),
    "pringles": ("cylinder", (0.038, 0.23), 0.20, (0.80, 0.10, 0.15)),
    "seaweed": ("box", (0.10, 0.04, 0.20), 0.08, (0.10, 0.35, 0.15)),
    "instant_noodles": ("cylinder", (0.05, 0.10), 0.12, (0.95, 0.55, 0.10)),
    "cornflakes": ("box", (0.19, 0.06, 0.27), 0.40, (0.95, 0.75, 0.20)),
    "coke": ("cylinder", (0.033, 0.12), 0.35, (0.70, 0.05, 0.05)),
    "pepsi": ("cylinder", (0.033, 0.12), 0.35, (0.10, 0.20, 0.70)),
    "red_bull": ("cylinder", (0.027, 0.135), 0.27, (0.20, 0.30, 0.60)),
    "soju": ("cylinder", (0.032, 0.21), 0.50, (0.25, 0.60, 0.30)),
    "milk": ("box", (0.07, 0.07, 0.20), 0.60, (0.98, 0.98, 1.00)),
    "rubiks_cube": ("box", (0.057, 0.057, 0.057), 0.10, (0.95, 0.45, 0.05)),
    "sponge": ("box", (0.11, 0.07, 0.035), 0.03, (0.95, 0.85, 0.20)),
    "toothpaste": ("box", (0.19, 0.045, 0.035), 0.12, (0.20, 0.75, 0.80)),
}

# Props that are not rulebook objects
PROPS = {
    "bag": ("cylinder", (0.025, 0.14), 0.05, (0.60, 0.30, 0.10)),
}
ALL = {**OBJECTS, **PROPS}

CUTLERY = ("spoon", "fork", "knife")
# Worldwide-available objects placed on the extra surface
COMMON = ("coke", "pepsi", "pringles", "red_bull")


def categories() -> dict:
    """object name -> category from frida_constants/data/objects.json."""
    path = os.path.join(
        get_package_share_directory("frida_constants"), "data", "objects.json"
    )
    with open(path) as f:
        return json.load(f)["object_to_category"]


def height(name: str) -> float:
    return ALL[name][1][-1]


def radius(name: str) -> float:
    """Half of the narrowest horizontal extent (what the fingers close on)."""
    shape, size, _, _ = ALL[name]
    return size[0] if shape == "cylinder" else min(size[0], size[1]) / 2.0


def footprint(name: str) -> float:
    """Radius of the circle that contains the object seen from above."""
    shape, size, _, _ = ALL[name]
    return size[0] if shape == "cylinder" else (size[0] ** 2 + size[1] ** 2) ** 0.5 / 2


def model_sdf(model: str, name: str, pose) -> str:
    """Free-standing model with its origin on the base centre."""
    shape, size, mass, rgb = ALL[name]
    h = size[-1]
    if shape == "cylinder":
        geometry = (
            f"<cylinder><radius>{size[0]}</radius><length>{h}</length></cylinder>"
        )
        ixx = mass * (3 * size[0] ** 2 + h**2) / 12
        izz = mass * size[0] ** 2 / 2
        iyy = ixx
    else:
        geometry = f"<box><size>{size[0]} {size[1]} {h}</size></box>"
        ixx = mass * (size[1] ** 2 + h**2) / 12
        iyy = mass * (size[0] ** 2 + h**2) / 12
        izz = mass * (size[0] ** 2 + size[1] ** 2) / 12
    colour = f"{rgb[0]} {rgb[1]} {rgb[2]} 1"
    x, y, z, yaw = pose
    return f"""    <model name="{model}">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <inertial>
          <pose>0 0 {h / 2} 0 0 0</pose>
          <mass>{mass}</mass>
          <inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <pose>0 0 {h / 2} 0 0 0</pose>
          <geometry>{geometry}</geometry>
          <surface>
            <friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction>
            <contact><ode><kp>1000000</kp><kd>100</kd><min_depth>0.001</min_depth><max_vel>0.1</max_vel></ode></contact>
          </surface>
        </collision>
        <visual name="visual">
          <pose>0 0 {h / 2} 0 0 0</pose>
          <geometry>{geometry}</geometry>
          <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse><specular>0.1 0.1 0.1 1</specular></material>
        </visual>
      </link>
    </model>
"""
