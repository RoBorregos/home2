#!/usr/bin/env python3

"""
Task manager to test ONLY the vision area for the HRIC task.

Runs the same vision subtask-manager calls the real HRIC FSM makes
(hric_task_manager.py), in the same order, without needing nav, HRI or
manipulation. Each step is wrapped so a failure (or an exception) is recorded
and the run continues, and the return value of every call is validated
against the (Status, ...) contract so broken returns are caught here instead
of mid-competition.

Usage (inside the integration container):
    ros2 run task_manager test_hric_vision.py
    ros2 run task_manager test_hric_vision.py --ros-args -p mock_data:=true
    ros2 run task_manager test_hric_vision.py --ros-args -p steps:="detect_person,find_seat"
"""

import time

import rclpy
from rclpy.node import Node

from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

# Steps in the order the HRIC task manager exercises vision.
ALL_STEPS = [
    "camera_orientation",
    "face_recognition_toggle",
    "detect_person",
    "follow_by_name_area",
    "follow_face_stream",
    "save_face_name",
    "describe_person",
    "get_person_name",
    "detect_hand",
    "find_seat",
    "find_seat_flipped",
    "find_drink",
    "moondream_query",
    "track_person",
]

# Outcomes that count as "the pipeline works" even if no target was in frame.
ACCEPTABLE = {Status.EXECUTION_SUCCESS, Status.TARGET_NOT_FOUND, Status.MOCKED}


class HRICVisionTest(Node):
    """Sequential test task manager for HRIC's vision surface."""

    def __init__(self):
        super().__init__("hric_vision_test_manager")
        self.declare_parameter("mock_data", False)
        self.declare_parameter("steps", "all")
        self.declare_parameter("guest_name", "TestGuest")
        self.declare_parameter("detect_person_timeout", 10.0)
        self.declare_parameter("describe_timeout", 30.0)

        self.mock_data = self.get_parameter("mock_data").value
        steps_param = self.get_parameter("steps").value
        self.guest_name = self.get_parameter("guest_name").value
        self.detect_person_timeout = self.get_parameter("detect_person_timeout").value
        self.describe_timeout = self.get_parameter("describe_timeout").value

        self.steps = (
            ALL_STEPS
            if steps_param == "all"
            else [s.strip() for s in steps_param.split(",") if s.strip() in ALL_STEPS]
        )
        self.results: list[tuple[str, str, float, str]] = []

        self.vision = VisionTasks(self, task=Task.HRIC, mock_data=self.mock_data)
        rclpy.spin_once(self, timeout_sec=1.0)
        Logger.info(self, f"HRICVisionTest started (mock_data={self.mock_data}).")
        Logger.info(self, f"Steps: {', '.join(self.steps)}")

    # ------------------------------------------------------------------ #
    # Harness                                                            #
    # ------------------------------------------------------------------ #

    def _record(self, step: str, outcome: str, elapsed: float, detail: str = ""):
        self.results.append((step, outcome, elapsed, detail))
        log = Logger.success if outcome == "PASS" else Logger.warn
        if outcome in ("EXCEPTION", "BAD_RETURN"):
            log = Logger.error
        log(self, f"[{step}] {outcome} ({elapsed:.2f}s) {detail}")

    def _check_status(self, step: str, status, elapsed: float, detail: str = ""):
        """Validate that `status` is a Status member and classify the outcome."""
        if not isinstance(status, Status):
            self._record(
                step,
                "BAD_RETURN",
                elapsed,
                f"expected Status enum, got {type(status).__name__}: {status!r}. {detail}",
            )
            return
        if status in ACCEPTABLE:
            outcome = "PASS" if status != Status.TARGET_NOT_FOUND else "NOT_FOUND"
            self._record(step, outcome, elapsed, detail)
        else:
            self._record(step, status.name, elapsed, detail)

    def _run_step(self, step: str, fn):
        Logger.info(self, f"=== [{step}] ===")
        start = time.time()
        try:
            fn()
        except Exception as e:
            self._record(step, "EXCEPTION", time.time() - start, repr(e))

    # ------------------------------------------------------------------ #
    # Steps (mirroring hric_task_manager.py's vision calls)              #
    # ------------------------------------------------------------------ #

    def step_camera_orientation(self):
        """START state: camera_upside_down(False); TAKE_BAG flips it."""
        start = time.time()
        self.vision.camera_upside_down(False)
        self.vision.camera_upside_down(True)
        self.vision.camera_upside_down(False)
        self._record("camera_orientation", "PASS", time.time() - start, "published 0/180/0")

    def step_face_recognition_toggle(self):
        """START/GREETING states: deactivate + activate face recognition."""
        start = time.time()
        self.vision.deactivate_face_recognition()
        self.vision.activate_face_recognition()
        self._record("face_recognition_toggle", "PASS", time.time() - start)

    def step_detect_person(self):
        """WAIT_FOR_GUEST state: DetectPerson action."""
        start = time.time()
        status = self.vision.detect_person(timeout=self.detect_person_timeout)
        self._check_status("detect_person", status, time.time() - start)

    def step_follow_by_name_area(self):
        """GREETING state: follow_by_name('area')."""
        start = time.time()
        status = self.vision.follow_by_name("area")
        self._check_status("follow_by_name_area", status, time.time() - start)

    def step_follow_face_stream(self):
        """GREETING state: the arm consumes get_follow_face() deltas."""
        start = time.time()
        updates = 0
        while time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            x, _y = self.vision.get_follow_face()
            if x is not None:
                updates += 1
        outcome = "PASS" if updates > 0 else "NOT_FOUND"
        self._record(
            "follow_face_stream",
            outcome,
            time.time() - start,
            f"{updates} face updates in 3s (needs a face in frame)",
        )

    def step_save_face_name(self):
        """SAVE_FACE state: save_face_name(guest name)."""
        start = time.time()
        status = self.vision.save_face_name(self.guest_name)
        self._check_status("save_face_name", status, time.time() - start)

    def step_describe_person(self):
        """SAVE_FACE state: describe_person() via async moondream queries."""
        start = time.time()
        done = {"status": None, "description": None}

        def cb(status, description):
            done["status"] = status
            done["description"] = description

        self.vision.describe_person(cb)
        while done["status"] is None and time.time() - start < self.describe_timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        elapsed = time.time() - start
        if done["status"] is None:
            self._record("describe_person", "TIMEOUT", elapsed, "no callback fired")
        else:
            self._check_status(
                "describe_person", done["status"], elapsed, f"desc='{done['description']}'"
            )

    def step_get_person_name(self):
        """INTRODUCTION state: get_person_name() + isPerson(name)."""
        start = time.time()
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        status, name = self.vision.get_person_name()
        known = self.vision.isPerson(self.guest_name)
        self._check_status(
            "get_person_name", status, time.time() - start, f"name={name!r} known={known}"
        )

    def step_detect_hand(self):
        """TAKE_BAG state: detect_hand() 3D point."""
        start = time.time()
        status, point = self.vision.detect_hand()
        detail = ""
        if status == Status.EXECUTION_SUCCESS and point is not None:
            p = point.point
            detail = f"point=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
        self._check_status("detect_hand", status, time.time() - start, detail)

    def step_find_seat(self):
        """FIND_SEAT state (normal camera)."""
        start = time.time()
        status, angle = self.vision.find_seat()
        self._check_status("find_seat", status, time.time() - start, f"angle={angle}")

    def step_find_seat_flipped(self):
        """FIND_SEAT state while carrying the bag (camera at 180)."""
        start = time.time()
        self.vision.camera_upside_down(True)
        try:
            status, angle = self.vision.find_seat()
        finally:
            self.vision.camera_upside_down(False)
        self._check_status("find_seat_flipped", status, time.time() - start, f"angle={angle}")

    def step_find_drink(self):
        """GREETING state: beverage location lookup."""
        start = time.time()
        status, location = self.vision.find_drink("water")
        self._check_status("find_drink", status, time.time() - start, f"location={location!r}")

    def step_moondream_query(self):
        """Moondream sanity query (used by describe/confirmation flows)."""
        start = time.time()
        status, result = self.vision.moondream_query("Briefly describe the scene.")
        self._check_status("moondream_query", status, time.time() - start, f"result={result!r}")

    def step_track_person(self):
        """FOLLOW_PERSON state: track_person on, then off."""
        start = time.time()
        status_on = self.vision.track_person(True)
        status_off = self.vision.track_person(False)
        detail = f"on={getattr(status_on, 'name', status_on)} off={getattr(status_off, 'name', status_off)}"
        self._check_status("track_person", status_on, time.time() - start, detail)

    # ------------------------------------------------------------------ #
    # Runner                                                             #
    # ------------------------------------------------------------------ #

    def run(self) -> int:
        for step in self.steps:
            self._run_step(step, getattr(self, f"step_{step}"))

        # Leave the stack in the state HRIC expects at task start.
        self.vision.deactivate_face_recognition()
        self.vision.camera_upside_down(False)

        Logger.info(self, "=" * 60)
        Logger.info(self, "HRIC VISION TEST SUMMARY")
        failures = 0
        for step, outcome, elapsed, detail in self.results:
            line = f"  {step:<24} {outcome:<12} {elapsed:6.2f}s  {detail}"
            if outcome == "PASS":
                Logger.success(self, line)
            elif outcome == "NOT_FOUND":
                Logger.warn(self, line)
            else:
                failures += 1
                Logger.error(self, line)
        Logger.info(self, "=" * 60)
        Logger.info(
            self,
            f"{len(self.results)} steps: "
            f"{sum(1 for r in self.results if r[1] == 'PASS')} pass, "
            f"{sum(1 for r in self.results if r[1] == 'NOT_FOUND')} target-not-found, "
            f"{failures} failing",
        )
        return failures


def main(args=None):
    rclpy.init(args=args)
    node = HRICVisionTest()
    failures = 0
    try:
        failures = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
