#!/usr/bin/env python3

"""
Test for PPC cabinet place flow. Mocks navigation, injects a fake grasped
object, and runs the real scan + place pipeline. Robot must be in front
of the cabinet with vision + manip containers running.

Usage:
    python3 test_ppc_cabinet_place.py
    python3 test_ppc_cabinet_place.py --shelf 2
    python3 test_ppc_cabinet_place.py --object apple
"""

import importlib.util
import json
import os
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from task_manager.utils.colored_logger import CLog
from task_manager.utils.subtask_manager import SubtaskManager, Task


def _load_task_manager_module():
    """Load pickandplace_task_manager.py as an importable module.

    The production script lives in scripts/ (installed to lib/task_manager/
    as an executable), not under the task_manager python package, so a
    regular `import pickandplace_task_manager` does not work from a test.
    We locate the file in the install dir and load it via importlib.
    """
    candidates = [
        # When running from the install tree
        "/workspace/install/task_manager/lib/task_manager/pickandplace_task_manager.py",
        # When running from the source tree
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "pickandplace_task_manager.py",
        ),
    ]
    for path in candidates:
        if os.path.isfile(path):
            spec = importlib.util.spec_from_file_location("pickandplace_task_manager", path)
            mod = importlib.util.module_from_spec(spec)
            sys.modules["pickandplace_task_manager"] = mod
            spec.loader.exec_module(mod)
            return mod
    raise RuntimeError("Could not locate pickandplace_task_manager.py in any known location")


_tm_mod = _load_task_manager_module()
PickAndPlaceTM = _tm_mod.PickAndPlaceTM
ObjectInfo = _tm_mod.ObjectInfo
ObjectCategory = _tm_mod.ObjectCategory
Location = _tm_mod.Location


class PPCTestCabinetPlace(PickAndPlaceTM):
    """Subclass of PickAndPlaceTM that skips cleanup and jumps to the cabinet place flow."""

    # When set, _get_shelf_height_for_object always returns this height,
    # bypassing categorization.  The scan still runs (for octomap), but the
    # shelf assignment is deterministic.
    _forced_shelf_height: float | None = None

    def _get_shelf_height_for_object(self, obj):
        if self._forced_shelf_height is not None:
            CLog.vision(
                self,
                "SHELF",
                f"Forced shelf height {self._forced_shelf_height}m for '{obj.name}'",
            )
            return self._forced_shelf_height
        return super()._get_shelf_height_for_object(obj)

    def __init__(self, object_name: str = "coca_cola"):
        # Call Node.__init__ directly; we will override subtask_manager below.
        Node.__init__(self, "ppc_test_cabinet_place")
        self.subtask_manager = SubtaskManager(
            self,
            task=Task.PICK_AND_PLACE,
            mock_areas=["navigation"],  # <-- key: only nav is mocked
        )

        # Same config as PickAndPlaceTM.__init__
        self.use_side_table = False
        self.trash_category = "napkin"
        self.use_dishwasher = False

        self.yolo_names = {
            "cereal": "blue_cereal_box",
            "milk": "chocomilk_box",
        }
        self.yolo_to_logical = {v: k for k, v in self.yolo_names.items()}

        self.shelf_level_heights = {1: 0.475, 2: 0.827, 3: 1.201}
        self.default_shelf_height = 0.475

        try:
            from ament_index_python.packages import get_package_share_directory

            objects_path = (
                Path(get_package_share_directory("frida_constants")) / "data" / "objects.json"
            )
            with open(objects_path) as f:
                self._object_to_category = json.load(f).get("object_to_category", {})
        except Exception:
            self._object_to_category = {}

        self.nav_locations = {
            Location.KITCHEN: ("kitchen", ""),
            Location.DINING_TABLE: ("kitchen", "dining_table"),
            Location.SIDE_TABLE: ("kitchen", "side_table"),
            Location.DISHWASHER: ("kitchen", "dishwasher"),
            Location.CABINET: ("kitchen", "cabinet"),
            Location.TRASH_BIN: ("kitchen", "trash_bin"),
            Location.BREAKFAST_SURFACE: ("kitchen", "breakfast_surface"),
        }

        self.detected_objects: list = []
        self.current_object_index = 0
        self.grasped_object: ObjectInfo = None
        self.first_pick = True
        self.max_cleanup_objects = 3
        self.breakfast_items: list = []
        self.current_breakfast_item = None
        self.bowl_placed = False
        self.dishwasher_open = False

        self.shelves = {}
        self.object_to_placing_shelf = defaultdict(list)
        self.shelf_scanned = False
        self._cabinet_scan_fresh = False
        self._shelf_fallback_heights: list[float] = []
        self._shelf_fallback_idx: int = 0
        self.shelf_level_threshold = 0.20
        self.shelf_level_down_threshold = 0.05

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_attempts = 0
        self.current_location: Location = Location.DINING_TABLE  # pretend we're at the table
        self.running_task = True

        self.state_start_time = None
        self.state_times: dict = {}
        self.total_start_time = datetime.now()
        self.previous_state = None

        # ---------------------------------------------------------------
        # Inject the fake grasped object and jump straight to NAVIGATE_TO_PLACEMENT
        # ---------------------------------------------------------------
        self.detected_objects = [
            ObjectInfo(
                name=object_name,
                category=ObjectCategory.OTHER,  # OTHER routes to CABINET
                bbox=None,
            )
        ]
        self.detected_objects[0].is_picked = True  # pretend it's already in the gripper
        self.grasped_object = self.detected_objects[0]
        self.grasped_object.placement_location = Location.CABINET

        self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT

        CLog.fsm(
            self,
            "TEST",
            f"Test harness ready. Object: {object_name}. "
            "Navigation is mocked; everything else is real.",
            level="success",
        )
        CLog.fsm(
            self,
            "TEST",
            "The FSM will loop so you can test retries. Ctrl+C to exit.",
        )


def main(args=None):
    import argparse

    parser = argparse.ArgumentParser(description="Test PPC cabinet place flow")
    parser.add_argument(
        "--shelf",
        type=int,
        choices=[1, 2, 3],
        default=None,
        help="Force placement on this shelf level (skip categorization). "
        "1=0.475m, 2=0.827m, 3=1.201m",
    )
    parser.add_argument("--object", type=str, default="coca_cola", help="Object name to place")
    parsed, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = PPCTestCabinetPlace(object_name=parsed.object)

    # If --shelf is given, override shelf assignment.  The scan still runs
    # (populates octomap) but when categorization fails (FindClosest not
    # available), the FSM falls back to default_shelf_height — so we just
    # set that to the forced value.  We also pre-populate
    # object_to_placing_shelf so that even if categorization succeeds, it
    # returns the forced shelf.
    if parsed.shelf is not None:
        forced_height = node.shelf_level_heights[parsed.shelf]
        node.default_shelf_height = forced_height
        node.object_to_placing_shelf[parsed.object] = [parsed.shelf - 1]
        CLog.fsm(
            node,
            "TEST",
            f"Forced shelf {parsed.shelf} at {forced_height}m (default overridden).",
            level="success",
        )

    # Stop conditions specific to this test: we only care about the cabinet
    # place flow, so we exit as soon as the FSM transitions OUT of the
    # cabinet-related states (either to CLEANUP_LOOP once the place is
    # resolved, or to any breakfast / end state).
    stop_states = {
        PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP,
        PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS,
        PickAndPlaceTM.TaskStates.END,
    }

    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
            if node.current_state in stop_states:
                CLog.fsm(
                    node,
                    "TEST",
                    f"Reached stop state {node.current_state}; test finished.",
                    level="success",
                )
                break
    except KeyboardInterrupt:
        CLog.fsm(node, "TEST", "Interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
