"""Score a run against its rulebook scoresheet from referee events and final poses."""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from frida_task_sim import catalog, venue as venues

DROP_LIMIT = 0.10
CLEAR_AREA = 0.05
NEXT_TO = {("bowl", "spoon"): 0.15, ("milk", "cornflakes"): 0.20}


def load_sheet(task: str) -> dict:
    share = get_package_share_directory("frida_task_sim")
    with open(os.path.join(share, "config", "scoresheets", f"{task}.yaml")) as f:
        return yaml.safe_load(f)


def _gap(a: dict, b: dict, names: dict) -> float:
    """Free space between two objects seen from above."""
    (ax, ay, _), (bx, by, _) = a["position"], b["position"]
    return (
        math.hypot(ax - bx, ay - by)
        - catalog.footprint(names[a["id"]])
        - catalog.footprint(names[b["id"]])
    )


def ppc_counts(scenario: dict, events: list, final: dict) -> dict:
    """Row id -> (count, details) for the Pick and Place scoresheet."""
    venue = venues.load(scenario["venue"])
    roles = venue["ppc"]
    objects = {o["id"]: o for o in scenario["objects"]}
    names = {oid: o["name"] for oid, o in objects.items()}
    target = {
        "cutlery": roles["dishwasher"],
        "tableware": roles["dishwasher"],
        "trash": roles["trash"],
        "floor_trash": roles["trash"],
        "other": roles["cabinet"],
        "common": roles["cabinet"],
        "breakfast": roles["table"],
    }
    counts = {}

    def add(row, detail):
        counts.setdefault(row, []).append(detail)

    if any(e["type"] == "at_table" for e in events):
        add("navigate_to_table", "reached the dining table")
    for e in events:
        if e["type"] == "recognize" and e["correct"]:
            add("recognize_object", e["object"])
        if e["type"] == "shelf_indicated" and e["correct"]:
            add("shelf_placement_indicated", f"{e['object']} -> shelf {e['level'] + 1}")

    picked = {}
    for e in events:
        if e["type"] == "pick" and e["object"] not in picked:
            picked[e["object"]] = e
    for oid, e in picked.items():
        add("pick", oid)
        if e["first"]:
            add("first_pick", oid)
        role = objects[oid]["role"]
        if role == "floor_trash":
            add("pick_floor", oid)
        if names[oid] in catalog.CUTLERY:
            add("pick_cutlery", oid)
        if names[oid] == "plate":
            add("pick_plate", oid)
        if role == "common":
            add("pick_common", oid)

    last_place = {}
    for e in events:
        if e["type"] == "place":
            last_place[e["object"]] = e
            if e["location"] == "floor":
                add("dropped_floor", e["object"])
            elif e["drop"] > DROP_LIMIT and e["location"] != roles["trash"]:
                add("thrown", f"{e['object']} fell {e['drop']:.2f} m")
        if e["type"] == "knocked":
            add("dropped_floor", f"{e['object']} knocked off {e['source']}")
    for oid, e in last_place.items():
        obj = objects[oid]
        if e["location"] != target.get(obj["role"]):
            continue
        add("place", f"{oid} -> {e['location']}")
        if obj["role"] == "common":
            add("place_common", oid)
        if (
            e["location"] == roles["cabinet"]
            and scenario["shelves"].get(str(e["level"])) == obj["category"]
        ):
            add("place_similar", f"{oid} on the {obj['category']} shelf")

    on_table = [
        {"id": oid, **f} for oid, f in final.items() if f["location"] == roles["table"]
    ]
    served = [
        o
        for o in on_table
        if objects[o["id"]]["role"] == "breakfast" and o["id"] in last_place
    ]
    for item in served:
        clutter = [
            o["id"]
            for o in on_table
            if objects[o["id"]]["role"] != "breakfast"
            and _gap(item, o, names) < CLEAR_AREA
        ]
        if clutter:
            add("breakfast_area_dirty", f"{item['id']} next to {clutter}")
    by_name = {names[o["id"]]: o for o in served}
    for (a, b), limit in NEXT_TO.items():
        if (
            a in by_name
            and b in by_name
            and _gap(by_name[a], by_name[b], names) > limit
        ):
            add("breakfast_not_typical", f"{a} is not next to {b}")
            break
    return counts


COUNTERS = {"ppc": ppc_counts}


def score(scenario: dict, events: list, final: dict) -> dict:
    """Scoresheet rows with points, plus totals."""
    sheet = load_sheet(scenario["task"])
    counts = COUNTERS[scenario["task"]](scenario, events, final)
    rows, total, max_sheet, max_sim = [], 0, 0, 0
    for row in sheet["rows"]:
        simulated = row.get("simulated", True)
        details = counts.get(row["id"], [])
        count = min(len(details), row["count"]) if simulated else 0
        points = count * row["points"]
        total += points
        if row["points"] > 0:
            max_sheet += row["points"] * row["count"]
            max_sim += row["points"] * row["count"] if simulated else 0
        rows.append(
            {
                **row,
                "simulated": simulated,
                "scored": count,
                "total": points,
                "details": details[: row["count"]],
            }
        )
    return {
        "task": scenario["task"],
        "seed": scenario["seed"],
        "total": total,
        "max_in_sim": max_sim,
        "max_rulebook": max_sheet,
        "rows": rows,
    }
