#!/usr/bin/env python3
"""Asymmetric sphere inflation for the FRIDA spherized URDF.

Why: FOAM's output undercovers thin links (link5/link6 get only a few spheres)
because the volume heuristic caps branch_value low for low-volume meshes. The
union of those spheres is a strict inner approximation of the actual mesh, so
VAMP can plan paths the real robot mesh hits — what we found chasing Tier 2.5.

Inflating spheres globally would fix that but also inflate the gripper and
fingers, breaking close-approach picking. This script only inflates spheres on
links we tag as "body" (link_base, link1..6, base_link), leaving gripper,
right_finger, left_finger, zed_camera_link, intel_realsense, laser untouched.

Usage:
    inflate_body_spheres.py <input.urdf> <output.urdf> [--delta METERS]
"""

from __future__ import annotations
import argparse
import sys
import xml.etree.ElementTree as ET

BODY_LINKS = {
    "link_base",
    "link1",
    "link2",
    "link3",
    "link4",
    "link5",
    "link6",
    "base_link",
}


def inflate(in_path: str, out_path: str, delta: float) -> None:
    tree = ET.parse(in_path)
    root = tree.getroot()
    inflated = {}
    for link in root.findall("link"):
        name = link.get("name")
        if name not in BODY_LINKS:
            continue
        n_changed = 0
        for collision in link.findall("collision"):
            for sphere in collision.findall("geometry/sphere"):
                r = float(sphere.get("radius"))
                sphere.set("radius", f"{r + delta:.6f}")
                n_changed += 1
        if n_changed:
            inflated[name] = n_changed

    tree.write(out_path, xml_declaration=True, encoding="utf-8")
    total = sum(inflated.values())
    print(
        f"Inflated {total} spheres across {len(inflated)} body links (Δ={delta * 1000:.1f} mm):"
    )
    for link, n in sorted(inflated.items()):
        print(f"  {link}: +{n}")
    untouched = sorted(
        {
            "gripper",
            "right_finger",
            "left_finger",
            "zed_camera_link",
            "intel_realsense",
            "laser",
        }
    )
    print(f"Left untouched (precision-critical or non-arm): {', '.join(untouched)}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", help="Input spherized URDF")
    p.add_argument("output", help="Output URDF with inflated body spheres")
    p.add_argument(
        "--delta",
        type=float,
        default=0.005,
        help="Meters to add to each body-link sphere radius (default 0.005)",
    )
    args = p.parse_args()
    inflate(args.input, args.output, args.delta)
    return 0


if __name__ == "__main__":
    sys.exit(main())
