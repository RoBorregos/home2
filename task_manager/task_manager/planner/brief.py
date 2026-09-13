"""
Task briefs: what a task is worth, declared instead of coded.

In GPSR a person speaks the goal. In every other task the goal comes from the rulebook
months in advance — so it is written once, into a file, with the scoresheet open, rather
than parsed at runtime. A brief replaces a task manager's finite state machine; the
skills underneath it do not change.

ROS-free so briefs can be parsed and tested offline.
"""

import os
import re
from dataclasses import dataclass, field
from typing import Any, Optional

from task_manager.skills.registry import REGISTRY

# skill(arg) | skill(a, k=v) | skill()
_CALL = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*\((.*)\)\s*$", re.DOTALL)
_PLACEHOLDER = re.compile(r"\{([A-Za-z_][A-Za-z0-9_.]*)\}")


class BriefError(ValueError):
    """Raised when a brief is malformed — always with the offending text."""


@dataclass(frozen=True)
class Step:
    """One skill call in an objective's template."""

    skill: str
    args: dict = field(default_factory=dict)

    def bind(self, binding: dict) -> dict:
        """Resolve {placeholders} against a binding, e.g. {'obj': DetectedObject}."""
        return {key: _resolve(value, binding) for key, value in self.args.items()}


@dataclass(frozen=True)
class Objective:
    """A scoreable unit of work the selector can choose."""

    id: str
    points: float
    template: tuple = ()
    requires: tuple = ()
    repeat_for: Any = None
    once: bool = False
    guard: str = ""
    fallback: str = ""
    penalty_risk: float = 0.0
    max_attempts: int = 2
    at: str = ""  # location this objective happens at, for travel cost

    @property
    def repeats(self) -> bool:
        return self.repeat_for is not None


@dataclass(frozen=True)
class Trigger:
    """Event-driven entry point, for reactive tasks like HRIC and Restaurant."""

    on: str
    objective: str
    when: tuple = ()


@dataclass(frozen=True)
class Brief:
    task: str
    budget_s: float
    objectives: tuple
    risk_posture: str = "safe"
    triggers: tuple = ()
    on_start: tuple = ()
    on_deadline: tuple = ()

    def objective(self, objective_id: str) -> Optional[Objective]:
        for objective in self.objectives:
            if objective.id == objective_id:
                return objective
        return None


def parse_step(text: Any) -> Step:
    """Parse ``skill(a, k=v)`` or a ``{skill: ..., args: {...}}`` mapping."""
    if isinstance(text, dict):
        skill = text.get("skill")
        if not skill:
            raise BriefError(f"step mapping needs a 'skill' key: {text!r}")
        return Step(skill=skill, args=dict(text.get("args", {})))

    if not isinstance(text, str):
        raise BriefError(f"step must be a string or mapping, got {text!r}")

    match = _CALL.match(text)
    if not match:
        # bare skill name, no arguments
        name = text.strip()
        _check_skill(name, text)
        return Step(skill=name)

    name, raw_args = match.group(1), match.group(2).strip()
    _check_skill(name, text)

    args: dict = {}
    positional: list = []
    for piece in _split_args(raw_args):
        if "=" in piece and not piece.split("=", 1)[0].strip().startswith("{"):
            key, value = piece.split("=", 1)
            args[key.strip()] = _literal(value.strip())
        elif piece:
            positional.append(_literal(piece))

    # bind positionals to the skill's declared argument order
    declared = REGISTRY[name].args
    if len(positional) > len(declared):
        raise BriefError(f"'{text}' passes {len(positional)} args; {name} declares {len(declared)}")
    for arg, value in zip(declared, positional):
        if arg.name in args:
            raise BriefError(f"'{text}' sets {arg.name} both positionally and by name")
        args[arg.name] = value
    return Step(skill=name, args=args)


def _check_skill(name: str, text: str) -> None:
    if name not in REGISTRY:
        raise BriefError(f"unknown skill '{name}' in step '{text}'")


def _split_args(raw: str) -> list:
    """Split on commas that are not inside braces or quotes."""
    pieces, depth, current, quote = [], 0, [], ""
    for char in raw:
        if quote:
            current.append(char)
            if char == quote:
                quote = ""
            continue
        if char in "\"'":
            quote = char
            current.append(char)
        elif char in "{[(":
            depth += 1
            current.append(char)
        elif char in "}])":
            depth -= 1
            current.append(char)
        elif char == "," and depth == 0:
            pieces.append("".join(current).strip())
            current = []
        else:
            current.append(char)
    if current:
        pieces.append("".join(current).strip())
    return [piece for piece in pieces if piece]


def _literal(text: str) -> Any:
    """YAML-ish scalar conversion; leaves {placeholders} as strings."""
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in "\"'":
        return text[1:-1]
    lowered = text.lower()
    if lowered in {"true", "false"}:
        return lowered == "true"
    if lowered in {"none", "null"}:
        return None
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        pass
    return text


def _resolve(value: Any, binding: dict) -> Any:
    """Substitute {name} and {name.attr} from the binding."""
    if not isinstance(value, str):
        return value

    whole = _PLACEHOLDER.fullmatch(value)
    if whole:
        # a lone placeholder keeps the bound object's type
        return _lookup(whole.group(1), binding)

    def replace(match):
        return str(_lookup(match.group(1), binding))

    return _PLACEHOLDER.sub(replace, value)


def _lookup(path: str, binding: dict) -> Any:
    head, _, rest = path.partition(".")
    if head not in binding:
        raise BriefError(f"no binding for '{{{path}}}'")
    current = binding[head]
    for attribute in filter(None, rest.split(".")):
        current = getattr(current, attribute, None)
        if current is None:
            return ""
    return current


def parse_brief(data: dict) -> Brief:
    """Build a Brief from already-loaded YAML data."""
    if "task" not in data:
        raise BriefError("brief needs a 'task' key")
    if "budget_s" not in data:
        raise BriefError(f"brief '{data['task']}' needs a 'budget_s'")

    objectives = []
    seen = set()
    for raw in data.get("objectives", []):
        objective = _parse_objective(raw)
        if objective.id in seen:
            raise BriefError(f"duplicate objective id '{objective.id}'")
        seen.add(objective.id)
        objectives.append(objective)
    if not objectives:
        raise BriefError(f"brief '{data['task']}' declares no objectives")

    triggers = tuple(_parse_trigger(raw) for raw in data.get("triggers", []))
    for trigger in triggers:
        if trigger.objective not in seen:
            raise BriefError(f"trigger '{trigger.on}' targets unknown '{trigger.objective}'")

    posture = data.get("risk_posture", "safe")
    if posture not in {"safe", "aggressive"}:
        raise BriefError(f"risk_posture must be safe|aggressive, got '{posture}'")

    brief = Brief(
        task=data["task"],
        budget_s=float(data["budget_s"]),
        risk_posture=posture,
        objectives=tuple(objectives),
        triggers=triggers,
        on_start=tuple(parse_step(step) for step in data.get("on_start", [])),
        on_deadline=tuple(parse_step(step) for step in data.get("on_deadline", [])),
    )

    for objective in brief.objectives:
        if objective.fallback and objective.fallback not in seen:
            # a fallback may also name a bare skill
            if objective.fallback not in REGISTRY:
                raise BriefError(
                    f"objective '{objective.id}' has unknown fallback '{objective.fallback}'"
                )
    return brief


def _parse_trigger(raw: dict) -> Trigger:
    """
    Triggers name their event with ``event:``.

    Not ``on:`` — YAML 1.1 parses a bare ``on`` key as the boolean True, which silently
    produces a trigger nothing can ever fire. An unquoted ``on`` is rejected loudly here
    rather than left to fail on the arena floor.
    """
    if True in raw or "on" in raw:
        raise BriefError(
            f"trigger uses 'on:' — rename it to 'event:' (YAML reads bare 'on' as true): {raw!r}"
        )
    if "event" not in raw:
        raise BriefError(f"trigger needs an 'event' key: {raw!r}")
    if "objective" not in raw:
        raise BriefError(f"trigger '{raw['event']}' needs an 'objective' key")
    return Trigger(
        on=str(raw["event"]),
        objective=raw["objective"],
        when=tuple(_as_list(raw.get("when", []))),
    )


def _parse_objective(raw: dict) -> Objective:
    if "id" not in raw:
        raise BriefError(f"objective needs an 'id': {raw!r}")
    template = [parse_step(step) for step in _as_list(raw.get("template", []))]
    if "skill" in raw and not template:
        template = [parse_step(raw["skill"])]
    if not template:
        raise BriefError(f"objective '{raw['id']}' has no template or skill")
    return Objective(
        id=raw["id"],
        points=float(raw.get("points", 0)),
        template=tuple(template),
        requires=tuple(_as_list(raw.get("requires", []))),
        repeat_for=raw.get("repeat_for"),
        once=bool(raw.get("once", False)),
        guard=raw.get("guard", ""),
        fallback=raw.get("fallback", ""),
        penalty_risk=float(raw.get("penalty_risk", 0)),
        max_attempts=int(raw.get("max_attempts", 2)),
        at=raw.get("at", ""),
    )


def _as_list(value: Any) -> list:
    if value is None:
        return []
    return list(value) if isinstance(value, (list, tuple)) else [value]


def load_brief(path: str) -> Brief:
    """Load and validate a brief from a YAML file."""
    import yaml

    if not os.path.exists(path):
        raise BriefError(f"no brief at {path}")
    with open(path, encoding="utf-8") as handle:
        return parse_brief(yaml.safe_load(handle))


def briefs_dir() -> str:
    """Briefs beside the package in a source tree, or in the ament share dir once installed."""
    local = os.path.join(os.path.dirname(os.path.dirname(__file__)), "briefs")
    if os.path.isdir(local):
        return local
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory("task_manager"), "briefs")
    except Exception:  # noqa: BLE001 — outside a workspace the local path is all there is
        return local
