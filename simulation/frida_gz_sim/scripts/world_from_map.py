#!/usr/bin/env python3
"""Turns a nav2 occupancy grid (.pgm + .yaml) into a Gazebo world of wall boxes.

The world is emitted in MAP coordinates, so the Gazebo world frame and the
navigation map frame coincide and the poses in areas_<map>.json apply unchanged.

    ros2 run frida_gz_sim world_from_map.py \
        <map_context>/maps/robocup2026_1.yaml worlds/arena_robocup2026_1.sdf
"""

import argparse
import os
import sys

from frida_gz_sim.world import build_world


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_yaml", help="nav2 map metadata YAML")
    parser.add_argument("output", help="SDF world file to write")
    parser.add_argument("--wall-height", type=float, default=1.2)
    parser.add_argument(
        "--collisions",
        action="store_true",
        help="Give the walls collision geometry (off by default, see the world comment)",
    )
    parser.add_argument("--world-name", default=None)
    args = parser.parse_args()

    name = args.world_name or os.path.splitext(os.path.basename(args.output))[0]
    world = build_world(args.map_yaml, name, args.wall_height, args.collisions)
    with open(args.output, "w") as f:
        f.write(world)
    print(
        f"Wrote {args.output} ({world.count('_visual"')} wall boxes)", file=sys.stderr
    )


if __name__ == "__main__":
    main()
