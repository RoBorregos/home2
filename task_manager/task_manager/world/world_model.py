"""
Runtime picture of what the robot believes about itself and the arena.

The repo had no such thing. In-hand state lived in per-task booleans that die with the
process (``carrying_bag`` in HRIC, ``carrying`` in pick-and-place), the embeddings
``items``/``locations`` tables are static build-time seeds with no production writers,
and the only gripper reasoning was the GPSR merger's hypothetical scheduling state over
a candidate plan rather than runtime truth.

A selector cannot choose well from a vague state view, so this is deliberately explicit
and boring: predicates, a held object, a location, and what has been seen where.

ROS-free so the selector and its offline replay tests can use it.
"""

from dataclasses import dataclass, field
from typing import Any, Optional

from task_manager.skills.registry import ARM_FREE, REGISTRY

SUCCESS = "EXECUTION_SUCCESS"


@dataclass
class DetectedObject:
    """One object the robot has seen, and what it has done about it."""

    name: str
    category: str = ""
    surface: str = ""
    destination: str = ""
    picked: bool = False
    placed: bool = False
    skipped: bool = False

    @property
    def pending(self) -> bool:
        return not (self.placed or self.skipped)


@dataclass
class WorldModel:
    """Mutable belief state, updated after every skill call."""

    predicates: set = field(default_factory=lambda: {ARM_FREE})
    holding: Optional[str] = None
    location: str = ""
    sublocation: str = ""
    objects: dict = field(default_factory=dict)
    # objective id -> times completed, so `once` and `repeat_for` can be enforced
    completed: dict = field(default_factory=dict)
    # (objective id, target) -> attempts, so one awkward object does not retire an objective
    attempts: dict = field(default_factory=dict)
    # problem category -> times solved, for scoresheets that penalise repetition
    categories: dict = field(default_factory=dict)
    facts: dict = field(default_factory=dict)

    # ---------------- predicates ----------------

    def holds(self, predicate: str) -> bool:
        return predicate in self.predicates

    def satisfies(self, requirements) -> bool:
        """True when every requirement holds. Supports `not <predicate>`."""
        for requirement in requirements:
            negated = requirement.startswith("not ")
            name = requirement[4:].strip() if negated else requirement
            if self.holds(name) == negated:
                return False
        return True

    def apply(self, skill_name: str, status: Any) -> None:
        """Fold a skill outcome into the belief state using the registry's effects."""
        if _name_of(status) != SUCCESS:
            return
        skill = REGISTRY.get(skill_name)
        if skill is None:
            return
        self.predicates.update(skill.sets)
        self.predicates.difference_update(skill.clears)

    # ---------------- objects ----------------

    def observe(self, name: str, **fields: Any) -> DetectedObject:
        """Record or update a seen object."""
        existing = self.objects.get(name)
        if existing is None:
            existing = DetectedObject(name=name)
            self.objects[name] = existing
        for key, value in fields.items():
            if hasattr(existing, key):
                setattr(existing, key, value)
        return existing

    def pending_objects(self) -> list:
        return [obj for obj in self.objects.values() if obj.pending]

    def pick_up(self, name: str) -> None:
        self.holding = name
        self.predicates.discard(ARM_FREE)
        self.predicates.add("holding")
        if name in self.objects:
            self.objects[name].picked = True

    def put_down(self, placed: bool = True) -> None:
        if self.holding and self.holding in self.objects:
            self.objects[self.holding].placed = placed
        self.holding = None
        self.predicates.discard("holding")
        self.predicates.add(ARM_FREE)

    def arrive(self, location: str, sublocation: str = "") -> None:
        self.location = location
        self.sublocation = sublocation

    # ---------------- objectives ----------------

    def mark_done(self, objective_id: str) -> None:
        self.completed[objective_id] = self.completed.get(objective_id, 0) + 1

    def times_done(self, objective_id: str) -> int:
        return self.completed.get(objective_id, 0)

    def solve_category(self, category: str) -> None:
        """Record that a problem of this kind was solved, for repetition scoring."""
        if category:
            self.categories[category] = self.categories.get(category, 0) + 1

    def category_solves(self, category: str) -> int:
        return self.categories.get(category, 0) if category else 0

    def note_attempt(self, objective_id: str, target: Any = None) -> int:
        """
        Count an attempt against one (objective, target) pair.

        Per target, not per objective: a repeating objective that fails on one object
        must still be allowed to try the next one, and a global counter would retire
        the whole objective after the first couple of awkward items.
        """
        key = (objective_id, _target_key(target))
        self.attempts[key] = self.attempts.get(key, 0) + 1
        return self.attempts[key]

    def attempts_for(self, objective_id: str, target: Any = None) -> int:
        return self.attempts.get((objective_id, _target_key(target)), 0)

    # ---------------- reporting ----------------

    def snapshot(self) -> dict:
        """JSON-serializable view, for the MCP world tool and the run log."""
        return {
            "location": self.location,
            "sublocation": self.sublocation,
            "holding": self.holding,
            "predicates": sorted(self.predicates),
            "objects": [
                {
                    "name": obj.name,
                    "category": obj.category,
                    "surface": obj.surface,
                    "destination": obj.destination,
                    "picked": obj.picked,
                    "placed": obj.placed,
                    "skipped": obj.skipped,
                }
                for obj in self.objects.values()
            ],
            "completed": dict(self.completed),
            "facts": dict(self.facts),
        }


def _name_of(status: Any) -> str:
    return getattr(status, "name", str(status))


def _target_key(target: Any) -> str:
    """Bindings hold objects or bare strings; both need a stable key."""
    if target is None:
        return ""
    return str(getattr(target, "name", target))
