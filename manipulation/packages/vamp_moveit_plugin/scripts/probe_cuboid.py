#!/usr/bin/env python3
"""Decisive diagnostic: at MoveIt-flagged-invalid states, list every robot
sphere's world position and check whether it actually enters the box volume."""

import vamp

START = [
    -2.3183422088623047,
    0.6329726576805115,
    -1.7279555797576904,
    1.9347864389419556,
    0.8104520440101624,
    -1.370638370513916,
]
GOAL = [
    -0.8735470771789551,
    0.5281401872634888,
    -1.2419078350067139,
    0.8877889513969421,
    -0.9701063632965088,
    0.25012558698654175,
]

BOX_CENTER = [0.58, 0.0, 1.07]
BOX_HALF = [0.12, 0.12, 0.12]
BOX_MIN = [BOX_CENTER[i] - BOX_HALF[i] for i in range(3)]
BOX_MAX = [BOX_CENTER[i] + BOX_HALF[i] for i in range(3)]

MOVEIT_N = 87
MOVEIT_INVALID = list(range(27, 77))


def lerp(t):
    return [(1.0 - t) * s + t * g for s, g in zip(START, GOAL)]


def sphere_box_clearance(sx, sy, sz, sr):
    """Signed distance from sphere surface to box AABB. Negative = overlap."""

    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    cx = clamp(sx, BOX_MIN[0], BOX_MAX[0])
    cy = clamp(sy, BOX_MIN[1], BOX_MAX[1])
    cz = clamp(sz, BOX_MIN[2], BOX_MAX[2])
    d = ((sx - cx) ** 2 + (sy - cy) ** 2 + (sz - cz) ** 2) ** 0.5
    return d - sr  # positive = clearance, negative = penetration


print(
    f"Box AABB:  x∈[{BOX_MIN[0]:.3f},{BOX_MAX[0]:.3f}]  y∈[{BOX_MIN[1]:.3f},{BOX_MAX[1]:.3f}]  z∈[{BOX_MIN[2]:.3f},{BOX_MAX[2]:.3f}]"
)

env_box = vamp.Environment()
env_box.add_cuboid(vamp.Cuboid(BOX_CENTER, [0, 0, 0], BOX_HALF))

print("\n=== Sphere FK at MoveIt-flagged states ===")
print(
    "For each state: shows closest sphere to box, and how many spheres VAMP thinks penetrate.\n"
)

worst_overall = (float("inf"), -1, -1)  # (clearance, state_idx, sphere_idx)
for i in [27, 35, 45, 55, 65, 70, 76]:
    t = i / (MOVEIT_N - 1)
    q = lerp(t)
    spheres = vamp.frida_real.fk(q)
    vamp_invalid = not vamp.frida_real.validate(q, env_box)
    best_clear = float("inf")
    best_idx = -1
    n_penetrating = 0
    for s_idx, s in enumerate(spheres):
        c = sphere_box_clearance(s.x, s.y, s.z, s.r)
        if c < best_clear:
            best_clear = c
            best_idx = s_idx
        if c < 0:
            n_penetrating += 1
    sx, sy, sz, sr = (
        spheres[best_idx].x,
        spheres[best_idx].y,
        spheres[best_idx].z,
        spheres[best_idx].r,
    )
    if best_clear < worst_overall[0]:
        worst_overall = (best_clear, i, best_idx)
    status = "VAMP-INVALID" if vamp_invalid else "VAMP-VALID"
    print(
        f"  state {i:3d} t={t:.3f}  [{status}]"
        f"  closest sphere #{best_idx:2d} at ({sx:+.3f},{sy:+.3f},{sz:+.3f}) r={sr:.3f}"
        f"  clearance={best_clear * 1000:+.1f}mm  n_penetrating={n_penetrating}"
    )

print(
    f"\nOverall worst clearance across probed states: {worst_overall[0] * 1000:+.1f} mm"
    f" (state {worst_overall[1]}, sphere #{worst_overall[2]})"
)

# Where ARE the spheres at state 45 (middle of flagged range)?
print("\n=== All sphere positions at state 45 (sorted by z, descending) ===")
q = lerp(45 / (MOVEIT_N - 1))
spheres = vamp.frida_real.fk(q)
sorted_by_z = sorted(enumerate(spheres), key=lambda kv: -kv[1].z)
print(f"  {'idx':>4} {'x':>8} {'y':>8} {'z':>8} {'r':>7} {'clearance(mm)':>15}")
for idx, s in sorted_by_z[:25]:
    c = sphere_box_clearance(s.x, s.y, s.z, s.r) * 1000
    mark = " <- closest" if idx == worst_overall[2] else ""
    print(f"  {idx:4d} {s.x:+8.3f} {s.y:+8.3f} {s.z:+8.3f} {s.r:7.3f} {c:+15.1f}{mark}")
print(f"  ... ({len(spheres) - 25} more spheres below this)")
print(f"\nTotal spheres reported by vamp.frida_real.fk: {len(spheres)}")
