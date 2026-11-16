#!/usr/bin/env python3
"""Write a shelf world for the shelf checks.

    ros2 run frida_gz_sim shelf_world.py competition 1 /tmp/shelf.sdf
    ros2 run frida_gz_sim shelf_world.py competition_tall 1 /tmp/shelf.sdf pick:3

Scenes live in frida_gz_sim/shelf_scenes.py; the second argument toggles the clutter
objects. A trailing ``pick`` adds a graspable bottle on the second shelf, ``pick:N``
on shelf N (1 = lowest board), placed clear of that shelf's clutter.
"""

import sys
from pathlib import Path

from frida_gz_sim.shelf_scenes import PICK_LEVEL, SCENES, pick_object_pose, world_sdf


def main():
    if len(sys.argv) not in (4, 5) or sys.argv[1] not in SCENES:
        print(
            f"usage: shelf_world.py <{'|'.join(SCENES)}> <0|1 objects> <out.sdf> "
            "[pick|pick:N]"
        )
        raise SystemExit(2)
    scene = SCENES[sys.argv[1]]
    objects = sys.argv[2] == "1"
    pick_arg = sys.argv[4] if len(sys.argv) == 5 else ""
    pick = pick_arg.startswith("pick")
    level = int(pick_arg.split(":")[1]) - 1 if ":" in pick_arg else PICK_LEVEL
    if pick and not 0 <= level < len(scene.boards):
        print(f"{scene.name} has shelves 1-{len(scene.boards)}")
        raise SystemExit(2)
    Path(sys.argv[3]).write_text(
        world_sdf(scene, objects=objects, pick=pick, pick_level=level)
    )
    print(f"{sys.argv[3]}: {scene.name}, surfaces {scene.surfaces}")
    if pick:
        x, y, z = pick_object_pose(scene, level, objects)
        print(f"pick object on shelf {level + 1} at ({x:.3f}, {y:.3f}, {z:.3f})")


if __name__ == "__main__":
    main()
