#!/usr/bin/env python3

"""Generate the clutter scenes: objects per table cell, near-to-far pick order and method order."""

import argparse
import json
import random

SOLIDS = [
    "banana",
    "soup_bar",
    "coca_cola_can",
    "apple",
    "cornflakes",
    "orange",
    "cup",
]
# Cells are ordered near to far; the operator keeps 3 to 5 cm between neighbours.
CELLS = [
    "near-left",
    "near-center",
    "near-right",
    "far-left",
    "far-center",
    "far-right",
]


def generate(num_scenes, seed):
    rng = random.Random(seed)
    # Balanced slots so every object appears about equally often.
    slots = []
    while len(slots) < num_scenes * 5:
        slots += rng.sample(SOLIDS, len(SOLIDS))
    # Each method goes first in half of the scenes.
    first_methods = [("geometric", "gpd"), ("gpd", "geometric")] * num_scenes
    first_methods = first_methods[:num_scenes]
    rng.shuffle(first_methods)
    scenes = []
    for scene_id in range(1, num_scenes + 1):
        objects = []
        for obj in slots:
            if obj not in objects:
                objects.append(obj)
            if len(objects) == 5:
                break
        for obj in objects:
            slots.remove(obj)
        cells = sorted(rng.sample(range(len(CELLS)), 5))
        methods = list(first_methods[scene_id - 1])
        scenes.append(
            {
                "id": scene_id,
                "objects": objects,
                "cells": [CELLS[i] for i in cells],
                "method_order": methods,
            }
        )
    return {"seed": seed, "scenes": scenes}


def main():
    parser = argparse.ArgumentParser(description="Generate clutter scenes")
    parser.add_argument("--scenes", type=int, default=10)
    parser.add_argument("--seed", type=int, default=2027)
    parser.add_argument("--out", default="/workspace/src/bags/scenes.json")
    args = parser.parse_args()
    data = generate(args.scenes, args.seed)
    with open(args.out, "w") as f:
        json.dump(data, f, indent=2)
    for scene in data["scenes"]:
        print(scene["id"], scene["method_order"][0], "first:")
        for obj, cell in zip(scene["objects"], scene["cells"]):
            print(f"  {cell}: {obj}")


if __name__ == "__main__":
    main()
