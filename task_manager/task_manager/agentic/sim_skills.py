"""Simulated skills with injectable failures for offline A/B runs.

Each episode samples a "world" from a seed: which items are easy, hard
(grasp sometimes slips -> transient) or impossible (out of workspace / not on
the bar -> permanent). The error strings mimic what the real subtask managers
log, so a supervisor that reads them can tell transient from permanent
failures. Every skill advances the SimClock by a modelled duration.
"""

import random
from dataclasses import dataclass, field
from typing import Callable, Dict, List

from .events import SkillResult
from .executor import SimClock

MENU = ["coke", "orange juice", "water", "chips", "cookies", "apple", "banana", "milk"]


@dataclass
class SimWorld:
    seed: int
    orders: List[str]
    # object -> "easy" | "slippery" | "unreachable" | "missing"
    objects: Dict[str, str]
    nav_fail_p: float = 0.15
    order_fail_p: float = 0.3
    place_fail_p: float = 0.15
    log: List[str] = field(default_factory=list)


def sample_world(seed: int) -> SimWorld:
    rng = random.Random(seed)
    orders = rng.sample(MENU, rng.choice([2, 3]))
    kinds = ["easy", "easy", "slippery", "unreachable", "missing"]
    objects = {o: rng.choice(kinds) for o in orders}
    return SimWorld(seed=seed, orders=orders, objects=objects)


PICK_SUCCESS_P = {"easy": 0.8, "slippery": 0.4, "unreachable": 0.0, "missing": 0.0}
PICK_ERRORS = {
    "easy": "grasp slipped, object fell back on the bar",
    "slippery": "grasp slipped, object fell back on the bar",
    "unreachable": "no valid grasp: object out of arm workspace (behind other items)",
    "missing": "object not detected among bar detections",
}


def make_sim_skills(world: SimWorld, clock: SimClock) -> Dict[str, Callable[..., SkillResult]]:
    # Common random numbers: the n-th call of a skill with the same args gets
    # the same draw in every policy, so policies only differ by their decisions.
    calls: Dict[str, int] = {}

    def draw(skill: str, *args) -> random.Random:
        key = f"{skill}|{args}"
        calls[key] = calls.get(key, 0) + 1
        return random.Random(f"{world.seed}|{key}|{calls[key]}")

    def spend(rng: random.Random, seconds: float) -> None:
        clock.advance(seconds * rng.uniform(0.85, 1.15))

    def wait_start_button() -> SkillResult:
        spend(draw("wait_start_button"), 2)
        return SkillResult.ok()

    def say(text: str = "") -> SkillResult:
        clock.advance(0.5 + 0.3 * len(text.split()))
        world.log.append(f"SAY: {text}")
        return SkillResult.ok()

    def navigate(location: str = "") -> SkillResult:
        rng = draw("navigate", location)
        spend(rng, 20)
        if rng.random() < world.nav_fail_p:
            return SkillResult.fail("navigation aborted: path blocked by a person")
        return SkillResult.ok()

    def take_order() -> SkillResult:
        rng = draw("take_order")
        spend(rng, 15)
        if rng.random() < world.order_fail_p:
            return SkillResult.fail("speech not understood (STT empty / low confidence)")
        return SkillResult.ok(orders=list(world.orders))

    def ask_to_type_order() -> SkillResult:
        # deus ex machina: customer types the order on the screen
        spend(draw("ask_to_type_order"), 25)
        return SkillResult.ok(orders=list(world.orders))

    def tell_barman(orders=None) -> SkillResult:
        spend(draw("tell_barman"), 6)
        return SkillResult.ok()

    def pick(object: str = "") -> SkillResult:
        kind = world.objects.get(object, "missing")
        rng = draw("pick", object)
        spend(rng, 8 if kind == "missing" else 25)  # detection fails fast
        if rng.random() < PICK_SUCCESS_P[kind]:
            return SkillResult.ok(holding=object)
        return SkillResult.fail(PICK_ERRORS[kind])

    def deus_pick(object: str = "") -> SkillResult:
        rng = draw("deus_pick", object)
        spend(rng, 20)
        if rng.random() < 0.9:
            return SkillResult.ok(holding=object)
        return SkillResult.fail("barman did not confirm placing the object in the gripper")

    def place() -> SkillResult:
        rng = draw("place")
        spend(rng, 15)
        if rng.random() < world.place_fail_p:
            return SkillResult.fail("place pose in collision with table objects")
        return SkillResult.ok(holding=None)

    def deus_place() -> SkillResult:
        spend(draw("deus_place"), 15)
        return SkillResult.ok(holding=None)

    return {
        "wait_start_button": wait_start_button,
        "say": say,
        "navigate": navigate,
        "take_order": take_order,
        "ask_to_type_order": ask_to_type_order,
        "tell_barman": tell_barman,
        "pick": pick,
        "deus_pick": deus_pick,
        "place": place,
        "deus_place": deus_place,
    }
