#!/usr/bin/env python3

"""
Offline routing tests for the flat-grasp generalization.

A single ``FLAT_OBJECT_NAMES`` constant now drives every flat-grasp decision
point (the flat_grasp_estimator's ``target_classes``, PickManager's
``is_flat_object`` and pick_server's ``is_flat``). These tests verify the
list's contents and the routing predicate so the four call sites stay
consistent — without needing ROS, a robot, Docker or a built workspace.

Run from anywhere::

    python3 manipulation/packages/pick_and_place/pick_and_place/tests/test_flat_grasp_routing.py

Exit code 0 when every check passes, 1 otherwise.
"""

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
    print("Flat-grasp generalization — offline routing tests")
    test_list_membership()
    test_routing_predicate()
    total = len(_RESULTS)
    passed = sum(_RESULTS)
    print(f"\n{'=' * 50}\nRESULT: {passed}/{total} checks passed.\n{'=' * 50}")
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
