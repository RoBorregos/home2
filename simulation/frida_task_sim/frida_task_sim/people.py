"""People and seats as visual-only primitives: the lidar and cameras see visuals,
and a hand that the gripper reaches for must give way like a real one."""

COLOURS = {
    "red": (0.85, 0.10, 0.10),
    "blue": (0.10, 0.25, 0.85),
    "green": (0.10, 0.65, 0.20),
    "yellow": (0.95, 0.85, 0.10),
}
HEAD_Z = 1.55
# Open hand of a standing person, in the person frame (x forward)
HAND = (0.55, 0.0, 1.00)
# How far a seated person sinks so the head sits at seated height
SEATED_DROP = 0.35
SEAT_HEIGHT = 0.45


def _visual(name: str, pose: str, geometry: str, rgb) -> str:
    colour = f"{rgb[0]} {rgb[1]} {rgb[2]} 1"
    return f"""        <visual name="{name}">
          <pose>{pose}</pose>
          <geometry>{geometry}</geometry>
          <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material>
        </visual>
"""


def person_sdf(person: dict) -> str:
    """Static figure: shirt-coloured body, head and one arm stretched forward."""
    x, y, z, yaw = person["pose"]
    skin = (0.90, 0.72, 0.60)
    hx, hy, hz = HAND
    parts = (
        _visual(
            "body",
            "0 0 0.70 0 0 0",
            "<cylinder><radius>0.17</radius><length>1.40</length></cylinder>",
            COLOURS[person["colour"]],
        )
        + _visual(
            "head",
            f"0 0 {HEAD_Z} 0 0 0",
            "<sphere><radius>0.11</radius></sphere>",
            skin,
        )
        + _visual(
            "arm",
            f"{hx / 2} {hy} {hz - 0.05} 0 0 0",
            f"<box><size>{hx} 0.07 0.07</size></box>",
            skin,
        )
    )
    return f"""    <model name="{person["id"]}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
{parts}      </link>
    </model>
"""


def seat_sdf(name: str, x: float, y: float) -> str:
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="link">
{_visual("seat", f"0 0 {SEAT_HEIGHT / 2} 0 0 0", f"<box><size>0.5 0.5 {SEAT_HEIGHT}</size></box>", (0.35, 0.30, 0.45))}      </link>
    </model>
"""
