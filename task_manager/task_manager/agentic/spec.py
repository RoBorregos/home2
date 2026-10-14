"""Declarative task spec (one YAML per RoboCup@Home task).

Example:

    name: restaurant_lite
    time_limit_s: 600
    persona: "You are FRIDA, a friendly waiter robot..."
    rules: |
      Short rulebook summary the supervisor reads.
    steps:
      - id: take_order
        skill: take_order
        max_attempts: 3
        points: 100
        fallbacks: [ask_to_type_order]
      - foreach: orders        # blackboard list
        as: item
        steps:
          - id: pick
            skill: pick
            args: {object: "{item}"}
            points: 50
            fallbacks: [deus_pick]
            fallback_points: 25

String args are formatted with the blackboard, so "{item}" resolves to the
current loop element.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Union

import yaml

DEFAULT_MAX_ATTEMPTS = 3


@dataclass
class Step:
    id: str
    skill: str
    args: Dict[str, Any] = field(default_factory=dict)
    max_attempts: int = DEFAULT_MAX_ATTEMPTS
    points: float = 0.0
    fallbacks: List[str] = field(default_factory=list)
    fallback_points: Optional[float] = None
    # critical: if it finally fails the task cannot continue (e.g. start button)
    critical: bool = False
    # skippable: False forbids the LLM from skipping it (e.g. delivering while holding)
    skippable: bool = True
    description: str = ""


@dataclass
class Loop:
    foreach: str
    var: str
    steps: List["Node"]


Node = Union[Step, Loop]


@dataclass
class TaskSpec:
    name: str
    steps: List[Node]
    time_limit_s: float = 600.0
    persona: str = ""
    rules: str = ""
    blackboard: Dict[str, Any] = field(default_factory=dict)
    # error substrings that never go away by retrying (used by the rules+ policy)
    permanent_errors: List[str] = field(default_factory=list)

    def max_points(self, blackboard: Optional[Dict[str, Any]] = None) -> float:
        """Upper bound of points; loops count their current blackboard length."""
        bb = blackboard if blackboard is not None else self.blackboard
        return _max_points(self.steps, bb)


def _max_points(nodes: List[Node], bb: Dict[str, Any]) -> float:
    total = 0.0
    for node in nodes:
        if isinstance(node, Loop):
            total += len(bb.get(node.foreach, []) or []) * _max_points(node.steps, bb)
        else:
            total += node.points
    return total


def _parse_nodes(raw_steps: List[Dict[str, Any]]) -> List[Node]:
    nodes: List[Node] = []
    seen = set()
    for raw in raw_steps:
        if "foreach" in raw:
            nodes.append(
                Loop(
                    foreach=raw["foreach"],
                    var=raw.get("as", "item"),
                    steps=_parse_nodes(raw.get("steps", [])),
                )
            )
            continue
        if "skill" not in raw:
            raise ValueError(f"Step without 'skill': {raw}")
        step = Step(
            id=raw.get("id", raw["skill"]),
            skill=raw["skill"],
            args=raw.get("args", {}) or {},
            max_attempts=int(raw.get("max_attempts", DEFAULT_MAX_ATTEMPTS)),
            points=float(raw.get("points", 0.0)),
            fallbacks=list(raw.get("fallbacks", []) or []),
            fallback_points=raw.get("fallback_points"),
            critical=bool(raw.get("critical", False)),
            skippable=bool(raw.get("skippable", True)),
            description=raw.get("description", ""),
        )
        if step.id in seen:
            raise ValueError(f"Duplicate step id: {step.id}")
        seen.add(step.id)
        nodes.append(step)
    return nodes


def spec_from_dict(data: Dict[str, Any]) -> TaskSpec:
    return TaskSpec(
        name=data["name"],
        steps=_parse_nodes(data.get("steps", [])),
        time_limit_s=float(data.get("time_limit_s", 600.0)),
        persona=data.get("persona", ""),
        rules=data.get("rules", ""),
        blackboard=dict(data.get("blackboard", {}) or {}),
        permanent_errors=list(data.get("permanent_errors", []) or []),
    )


def load_spec(path: str) -> TaskSpec:
    with open(path, "r", encoding="utf-8") as f:
        return spec_from_dict(yaml.safe_load(f))


def format_args(args: Dict[str, Any], blackboard: Dict[str, Any]) -> Dict[str, Any]:
    """Resolve "{key}" placeholders in string args from the blackboard."""
    out = {}
    for key, value in args.items():
        if isinstance(value, str) and "{" in value:
            try:
                value = value.format(**blackboard)
            except (KeyError, IndexError):
                pass
        out[key] = value
    return out
