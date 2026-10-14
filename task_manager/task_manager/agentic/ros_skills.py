"""Skill registry bound to the robot's SubtaskManager.

Each skill wraps one or more subtask calls (same sequences the hand-written
task managers use) and returns a SkillResult whose error string is what the
supervisor reads, so keep errors descriptive (transient vs permanent).
"""

import time
from typing import Callable, Dict, List

import rclpy

from task_manager.utils.status import Status

from .events import SkillResult


def _status_ok(status) -> bool:
    return status == Status.EXECUTION_SUCCESS


def build_ros_skills(node, subtask_manager) -> Dict[str, Callable[..., SkillResult]]:
    sm = subtask_manager

    def spin(seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

    # ── HRI ──
    def wait_start_button(timeout_s: float = 600.0) -> SkillResult:
        sm.hri.reset_task_status()
        end = time.time() + timeout_s
        while not sm.hri.start_button_clicked:
            if time.time() > end:
                return SkillResult.fail("start button not pressed (timeout)")
            rclpy.spin_once(node, timeout_sec=0.1)
        sm.hri.reset_task_status()
        return SkillResult.ok()

    def say(text: str = "", wait: bool = True) -> SkillResult:
        status = sm.hri.say(text, wait=wait)
        return SkillResult.ok() if _status_ok(status) else SkillResult.fail("speech service failed")

    def take_order(retries: int = 3) -> SkillResult:
        status, orders = sm.hri.take_order(retries=retries)
        if _status_ok(status) and orders:
            return SkillResult.ok(orders=list(orders))
        return SkillResult.fail("order not understood (speech empty or no menu match)")

    def ask_to_type_order() -> SkillResult:
        # TODO: there is no typed-order input on the display yet; until then the
        # fallback asks the customer to speak close to the microphone, one item at a time.
        sm.hri.say(
            "Sorry, I could not understand you. Please come closer to me and say "
            "your order slowly, one item at a time."
        )
        status, orders = sm.hri.take_order(retries=1)
        if _status_ok(status) and orders:
            return SkillResult.ok(orders=list(orders))
        return SkillResult.fail("order still not understood after asking to repeat closer")

    def tell_barman(orders: List[str] = None) -> SkillResult:
        items = orders or []
        sm.hri.say(
            f"Hello barman. I have {len(items)} item{'s' if len(items) != 1 else ''} "
            f"for one table: {', '.join(items)}. Please help me prepare them."
        )
        return SkillResult.ok()

    # ── Navigation ──
    def wait_door_open() -> SkillResult:
        status, _ = sm.nav.check_door()
        if _status_ok(status):
            return SkillResult.ok()
        spin(1.0)
        return SkillResult.fail("door still closed")

    def navigate(location: str = "", sublocation: str = "") -> SkillResult:
        sm.manipulation.follow_face(False)
        sm.manipulation.move_to_position("nav_pose")
        status, error = sm.nav.move_to_location(location, sublocation)
        if _status_ok(status):
            return SkillResult.ok()
        return SkillResult.fail(f"navigation to {location} failed: {error}")

    def return_to_origin(yaw_offset_deg: float = None) -> SkillResult:
        result = sm.nav.return_to_origin(yaw_offset_deg=yaw_offset_deg)
        status = result[0] if isinstance(result, tuple) else result
        if _status_ok(status):
            return SkillResult.ok()
        return SkillResult.fail("return to origin failed")

    # ── Manipulation (same flow as restaurant_task_manager.pick_object) ──
    def pick(object: str = "") -> SkillResult:
        status, detections = sm.vision.detect_objects()
        if not _status_ok(status) or not detections:
            return SkillResult.fail("no objects detected on the surface")
        labels = sm.vision.get_labels(detections)
        status, closest = sm.hri.find_closest(labels, object)
        if not _status_ok(status) or not closest.results:
            return SkillResult.fail(f"'{object}' not detected among bar detections {labels}")
        status = sm.manipulation.pick_object(closest.results[0])
        if _status_ok(status):
            return SkillResult.ok(holding=object)
        return SkillResult.fail(f"pick of '{closest.results[0]}' failed (status {status})")

    def deus_pick(object: str = "") -> SkillResult:
        sm.manipulation.open_gripper()
        sm.hri.say(f"Could you please place the {object} in my gripper and say yes when done?")
        _, confirmation = sm.hri.confirm(
            "Have you placed the object in my gripper?",
            use_keyword=True,
            retries=3,
            wait_between_retries=5,
        )
        if confirmation == "yes":
            sm.manipulation.close_gripper()
            return SkillResult.ok(holding=object)
        return SkillResult.fail("human did not confirm placing the object")

    def place() -> SkillResult:
        status = sm.manipulation.place()
        if _status_ok(status):
            return SkillResult.ok(holding=None)
        return SkillResult.fail(f"place failed (status {status})")

    def deus_place() -> SkillResult:
        sm.hri.say("Please take the object from my gripper and say yes when done.")
        _, confirmation = sm.hri.confirm(
            "Have you grabbed the object from my gripper?",
            use_keyword=True,
            retries=3,
            wait_between_retries=5,
        )
        if confirmation == "yes":
            sm.manipulation.open_gripper()
            return SkillResult.ok(holding=None)
        return SkillResult.fail("human did not confirm taking the object")

    return {
        "wait_start_button": wait_start_button,
        "say": say,
        "take_order": take_order,
        "ask_to_type_order": ask_to_type_order,
        "tell_barman": tell_barman,
        "wait_door_open": wait_door_open,
        "navigate": navigate,
        "return_to_origin": return_to_origin,
        "pick": pick,
        "deus_pick": deus_pick,
        "place": place,
        "deus_place": deus_place,
    }
