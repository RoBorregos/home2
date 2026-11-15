"""Seeded rulebook scenarios: a pure function of (task, seed, venue)."""

import json
import math
import random

from frida_task_sim import catalog, venue as venues

# Free space kept between two spawned objects (m)
SPACING = 0.02


def _place(rng, region, names, z, restarts=200, tries=200):
    """Rejection-sample non-overlapping poses for names inside a rectangle."""
    for _ in range(restarts):
        placed, poses = [], {}
        # Largest first: small objects fit in the gaps that are left
        for name in sorted(names, key=catalog.footprint, reverse=True):
            r = catalog.footprint(name)
            for _ in range(tries):
                x = rng.uniform(region[0] + r, max(region[0] + r, region[2] - r))
                y = rng.uniform(region[1] + r, max(region[1] + r, region[3] - r))
                if all(
                    math.hypot(x - px, y - py) >= r + pr + SPACING
                    for px, py, pr in placed
                ):
                    break
            else:
                break
            placed.append((x, y, r))
            yaw = round(rng.uniform(-3.14, 3.14), 2)
            poses[name] = [round(x, 3), round(y, 3), round(z + 0.002, 3), yaw]
        if len(poses) == len(names):
            return [poses[name] for name in names]
    raise RuntimeError(f"No free layout for {names} in {region}")


def _entity(ids, name, category, role, surface, pose, level=None):
    count = ids[name] = ids.get(name, 0) + 1
    return {
        "id": name if count == 1 else f"{name}_{count}",
        "name": name,
        "category": category,
        "role": role,
        "surface": surface,
        "level": level,
        "pose": pose,
    }


def ppc(seed: int, venue_name: str = "robocup2026_1") -> dict:
    """Pick and Place setup of the 2026 rulebook (section 5.2)."""
    rng = random.Random(seed)
    venue = venues.load(venue_name)
    roles, furniture = venue["ppc"], venue["furniture"]
    cats = catalog.categories()
    ids: dict[str, int] = {}
    objects = []

    trash_category = "fruit"
    trash_names = sorted(n for n in catalog.OBJECTS if cats[n] == trash_category)
    reserved = set(catalog.CUTLERY) | {"cup", "plate", "bowl", "milk", "cornflakes"}
    other_pool = sorted(
        n
        for n in catalog.OBJECTS
        if n not in reserved and cats[n] != trash_category and n not in catalog.COMMON
    )
    others = rng.sample(other_pool, 2)
    table_names = [
        rng.choice(catalog.CUTLERY),
        "cup",
        "plate",
        rng.choice(trash_names),
    ] + others
    rng.shuffle(table_names)
    table_roles = {
        **{n: "cutlery" for n in catalog.CUTLERY},
        "cup": "tableware",
        "plate": "tableware",
    }

    table = furniture[roles["table"]]
    for name, pose in zip(
        table_names, _place(rng, table["reach"], table_names, table["height"])
    ):
        role = table_roles.get(
            name, "trash" if cats[name] == trash_category else "other"
        )
        objects.append(_entity(ids, name, cats[name], role, roles["table"], pose))

    counter = furniture[roles["extra_surface"]]
    commons = rng.sample(catalog.COMMON, 2)
    for name, pose in zip(
        commons, _place(rng, counter["reach"], commons, counter["height"])
    ):
        objects.append(
            _entity(ids, name, cats[name], "common", roles["extra_surface"], pose)
        )

    source = furniture[roles["breakfast_surface"]]
    for name, pose in zip(
        ("bowl", "spoon"),
        _place(rng, source["reach"], ["bowl", "spoon"], source["height"]),
    ):
        objects.append(
            _entity(
                ids, name, cats[name], "breakfast", roles["breakfast_surface"], pose
            )
        )

    # Cabinet: one category per reachable level; milk and cereal next to their category
    cabinet = furniture[roles["cabinet"]]
    levels = [i for i, h in enumerate(cabinet["levels"]) if h >= 0.20]
    shelf_categories = ["drink", "food"] + rng.sample(
        sorted({cats[n] for n in others} | {"snack"}), 1
    )
    rng.shuffle(shelf_categories)
    shelves = {}
    cx, cy = cabinet["center"]
    width, depth = cabinet["size"]
    for level, category in zip(levels, shelf_categories):
        shelves[str(level)] = category
        group = [
            n
            for n in sorted(catalog.OBJECTS)
            if cats[n] == category and n not in table_names + commons
        ]
        breakfast = [n for n in ("milk", "cornflakes") if cats[n] == category]
        fillers = [n for n in group if n not in breakfast]
        names = breakfast + rng.sample(
            fillers, min(len(fillers), 2 - len(breakfast) + 1)
        )
        # Left half of the shelf holds the group, the right half stays free for placing
        region = [
            cx - width / 2 + 0.04,
            cy - depth / 2 + 0.04,
            cx + 0.05,
            cy + depth / 2 - 0.10,
        ]
        for name, pose in zip(
            names, _place(rng, region, names, cabinet["levels"][level])
        ):
            pose[3] = 0.0
            role = "breakfast" if name in breakfast else "cabinet"
            objects.append(
                _entity(ids, name, cats[name], role, roles["cabinet"], pose, level)
            )

    floor_name = rng.choice(trash_names)
    fx, fy = roles["floor_trash"]
    floor_pose = [
        round(fx + rng.uniform(-0.1, 0.1), 3),
        round(fy + rng.uniform(-0.1, 0.1), 3),
        0.002,
        0.0,
    ]
    objects.append(
        _entity(ids, floor_name, cats[floor_name], "floor_trash", "floor", floor_pose)
    )

    return {
        "task": "ppc",
        "seed": seed,
        "venue": venue_name,
        "time_limit_s": 420,
        "trash_category": trash_category,
        "shelves": shelves,
        "objects": objects,
        # Seeded perception noise of the ground-truth detector
        "detector": {"seed": seed, "miss_rate": 0.0, "mislabel_rate": 0.0},
    }


GENERATORS = {"ppc": ppc}


def generate(task: str, seed: int, venue_name: str = "robocup2026_1") -> dict:
    return GENERATORS[task](seed, venue_name)


def save(scenario: dict, path: str):
    with open(path, "w") as f:
        json.dump(scenario, f, indent=2)


def load(path: str) -> dict:
    with open(path) as f:
        return json.load(f)
