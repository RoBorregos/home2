#!/usr/bin/env python3

"""Offline checks for the FLAT_OBJECT_NAMES routing list and predicate.
Run with plain python3; no ROS, robot, Docker or build needed."""

import importlib.util
import os
import sys


def _load_constants():
    """Load frida_constants.manipulation_constants (pure-python, no ROS)."""
    # Preferred: direct import when frida_constants is on PYTHONPATH (container).
    try:
        from frida_constants import manipulation_constants as mc

        return mc
    except Exception:
        pass
    # Fallback: walk up the tree to find the source file (laptop, no build).
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        cand = os.path.join(
            d, "frida_constants", "frida_constants", "manipulation_constants.py"
        )
        if os.path.isfile(cand):
            spec = importlib.util.spec_from_file_location(
                "manipulation_constants", cand
            )
            mc = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mc)
            return mc
        d = os.path.dirname(d)
    raise RuntimeError("Could not locate frida_constants/manipulation_constants.py")


mc = _load_constants()
FLAT_OBJECT_NAMES = mc.FLAT_OBJECT_NAMES
CUTLERY_NAMES = mc.CUTLERY_NAMES
RIM_NAMES = mc.RIM_NAMES


def is_flat_grasp(name):
    """Mirror of PickManager.is_flat_grasp / pick_server is_flat / estimator target_classes."""
    return name is not None and name.lower() in FLAT_OBJECT_NAMES


_RESULTS = []


def check(name, got, expected):
    ok = got == expected
    _RESULTS.append(ok)
    print(
        f"  PASS  {name}"
        if ok
        else f"  FAIL  {name}: got={got!r} expected={expected!r}"
    )


def section(title):
    print(f"\n=== {title} ===")


def test_list_membership():
    section("FLAT_OBJECT_NAMES membership")
    # Cutlery must remain a subset -> no regression for existing flat picks.
    check("cutlery ⊆ flat", all(c in FLAT_OBJECT_NAMES for c in CUTLERY_NAMES), True)
    check("plate is flat", "plate" in FLAT_OBJECT_NAMES, True)
    check("red_plate is flat", "red_plate" in FLAT_OBJECT_NAMES, True)
    # Normal GPD-graspable objects must NOT be routed to the flat estimator.
    for n in ("apple", "coca_cola", "bowl", "cup", "mug"):
        check(f"{n} is NOT flat (normal GPD)", n in FLAT_OBJECT_NAMES, False)
    # Rim objects are handled by a different strategy -> must stay disjoint.
    check(
        "rim names disjoint from flat", set(RIM_NAMES) & set(FLAT_OBJECT_NAMES), set()
    )


def test_routing_predicate():
    section("is_flat_grasp routing predicate")
    check("fork -> flat", is_flat_grasp("fork"), True)
    check("spoon -> flat", is_flat_grasp("spoon"), True)
    check("red_plate -> flat", is_flat_grasp("red_plate"), True)
    check("RED_PLATE case-insensitive", is_flat_grasp("RED_PLATE"), True)
    check("apple -> not flat", is_flat_grasp("apple"), False)
    check("None -> not flat", is_flat_grasp(None), False)


def main():
    print("Flat-grasp routing tests")
    test_list_membership()
    test_routing_predicate()
    total = len(_RESULTS)
    passed = sum(_RESULTS)
    print(f"\n{'=' * 50}\nRESULT: {passed}/{total} checks passed.\n{'=' * 50}")
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
