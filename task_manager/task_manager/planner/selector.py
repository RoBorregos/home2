"""
Score-aware objective selection.

The scoresheet is a reward function the technical committee hands out months early, and
the robot has ~420 s to harvest it. Deciding what to attempt is therefore arithmetic on
a table, not reasoning: expected points per second, under a deadline, given measured
capabilities. No LLM belongs in this loop — it runs after every completed action and has
to be deterministic enough to unit test.

Worked example from the Pick & Place sheet: a pick+place cycle is 90 pts in ~45 s, so
~2 pts/s. Opening the dishwasher door autonomously is worth 400 but takes ~65 s, and
asking the referee costs -0. So attempt it only if p*400 > 2*65 = 130, i.e. p > 33%.
At the measured p=0.30 the selector asks for help and spends the time on another object.

ROS-free: a pure function over (brief, world, manifest, remaining time).
"""

from dataclasses import dataclass
from typing import Any, Callable, Optional

from task_manager.planner.brief import Brief, Objective
from task_manager.planner.manifest import Manifest
from task_manager.world.world_model import WorldModel

# minimum joint success probability an objective needs to be worth starting
POSTURE_MIN_P = {"safe": 0.35, "aggressive": 0.15}
# a risky objective (one that can score negative) needs this much confidence
POSTURE_MIN_P_RISKY = {"safe": 0.70, "aggressive": 0.40}
# fraction of an objective's expected cost that must fit in the remaining budget
DEADLINE_SLACK = 1.0


@dataclass
class Candidate:
    """One concrete thing the robot could do next."""

    objective: Objective
    binding: dict
    points: float
    p: float
    expected_s: float
    travel_s: float
    use_fallback: bool = False

    @property
    def total_s(self) -> float:
        return self.expected_s + self.travel_s

    @property
    def ev(self) -> float:
        """Expected points per second — the whole ranking rule."""
        return (self.p * self.points) / self.total_s if self.total_s > 0 else 0.0

    @property
    def label(self) -> str:
        target = self.binding.get("obj") or self.binding.get("item")
        name = getattr(target, "name", target)
        return f"{self.objective.id}({name})" if name else self.objective.id


def rank(
    brief: Brief,
    world: WorldModel,
    manifest: Manifest,
    remaining_s: float,
    travel_fn: Optional[Callable[[str, str], float]] = None,
    guards: Optional[dict] = None,
) -> list:
    """All currently-eligible candidates, best expected value first."""
    candidates = []
    for objective in brief.objectives:
        for binding in _bindings(objective, world):
            candidate = _evaluate(objective, binding, world, manifest, travel_fn, brief)
            if candidate is None:
                continue
            if not _admissible(candidate, brief, remaining_s, world, guards):
                continue
            candidates.append(candidate)
    candidates.sort(key=lambda c: c.ev, reverse=True)
    return candidates


def choose(
    brief: Brief,
    world: WorldModel,
    manifest: Manifest,
    remaining_s: float,
    travel_fn: Optional[Callable[[str, str], float]] = None,
    guards: Optional[dict] = None,
) -> Optional[Candidate]:
    """Best next objective, or None when nothing is worth doing in the time left."""
    candidates = rank(brief, world, manifest, remaining_s, travel_fn, guards)
    return candidates[0] if candidates else None


def _bindings(objective: Objective, world: WorldModel) -> list:
    """Expand a repeating objective into one binding per target."""
    if not objective.repeats:
        return [{}]

    source = objective.repeat_for
    if isinstance(source, (list, tuple)):
        return [{"item": item} for item in source]
    if source == "table_objects":
        return [{"obj": obj} for obj in world.pending_objects()]
    # any other name resolves against world.facts, letting a brief bind arena data
    return [{"item": item} for item in world.facts.get(str(source), [])]


def _cost_of(
    steps,
    binding: dict,
    manifest: Manifest,
    at: str = "",
) -> Optional[tuple]:
    """Joint success probability, expected seconds and destination for a step list."""
    joint_p = 1.0
    expected_s = 0.0
    destination = at
    for step in steps:
        try:
            args = step.bind(binding)
        except Exception:  # noqa: BLE001 — an unbindable step just isn't a candidate
            return None
        capability = manifest.lookup(step.skill, _context_for(step.skill, args))
        joint_p *= capability.p
        expected_s += capability.p50
        if step.skill == "go_to" and not destination:
            destination = str(args.get("location", ""))
    return joint_p, expected_s, destination


def _evaluate(
    objective: Objective,
    binding: dict,
    world: WorldModel,
    manifest: Manifest,
    travel_fn: Optional[Callable[[str, str], float]],
    brief: Brief,
) -> Optional[Candidate]:
    """
    Cost and success probability of achieving this objective once.

    An objective with a declared fallback has two paths to the same world state. When the
    autonomous path is too unreliable to be worth attempting, the candidate is rebuilt on
    the fallback path: it still reaches the state, but forfeits the bonus points, so its
    expected value drops and it naturally sinks below real scoring work. It gets picked
    only once nothing better is admissible — which is exactly when the state change is
    needed and the cheap route is the right one.
    """
    auto = _cost_of(objective.template, binding, manifest, objective.at)
    if auto is None:
        return None
    joint_p, expected_s, destination = auto

    use_fallback = False
    points = objective.points
    bar = _confidence_bar(objective, brief.risk_posture)

    if objective.fallback and joint_p < bar:
        fallback_steps = _fallback_steps(objective.fallback, brief)
        if fallback_steps is not None:
            fallback = _cost_of(fallback_steps, binding, manifest, objective.at)
            if fallback is not None:
                joint_p, expected_s, destination = fallback
                # the assist is priced at -0 on the scoresheet: no penalty, no bonus
                points = 0.0
                use_fallback = True

    travel_s = 0.0
    if travel_fn is not None and destination:
        travel_s = travel_fn(world.location, destination)

    return Candidate(
        objective=objective,
        binding=binding,
        points=points,
        p=joint_p,
        expected_s=expected_s,
        travel_s=travel_s,
        use_fallback=use_fallback,
    )


def _fallback_steps(fallback: str, brief: Brief):
    """A fallback names either another objective or a bare skill."""
    objective = brief.objective(fallback)
    if objective is not None:
        return objective.template
    from task_manager.planner.brief import Step
    from task_manager.skills.registry import REGISTRY

    if fallback in REGISTRY:
        return (Step(skill=fallback),)
    return None


def _confidence_bar(objective: Objective, posture: str) -> float:
    if objective.penalty_risk:
        return POSTURE_MIN_P_RISKY.get(posture, 0.7)
    return POSTURE_MIN_P.get(posture, 0.35)


def _context_for(skill: str, args: dict) -> dict:
    """Manifest buckets are keyed by the same tags @measured records."""
    context = {}
    if "object_name" in args:
        context["object"] = args["object_name"]
    if "pour_object_name" in args:
        context["object"] = args["pour_object_name"]
    if "location" in args:
        context["location"] = args["location"]
    if "label" in args:
        context["label"] = args["label"]
    return context


def _admissible(
    candidate: Candidate,
    brief: Brief,
    remaining_s: float,
    world: WorldModel,
    guards: Optional[dict],
) -> bool:
    objective = candidate.objective

    if objective.once and world.times_done(objective.id) > 0:
        return False
    if not objective.repeats and world.times_done(objective.id) >= objective.max_attempts:
        return False
    if not world.satisfies(objective.requires):
        return False
    if objective.guard and guards:
        check = guards.get(objective.guard)
        if check is not None and not check(candidate.binding, world):
            return False

    # never start work that cannot finish inside the budget
    if candidate.total_s > remaining_s * DEADLINE_SLACK:
        return False

    return candidate.p >= _confidence_bar(objective, brief.risk_posture)


def constant_travel(seconds: float = 31.0) -> Callable[[str, str], float]:
    """Flat per-leg cost. Measured arena average is ~31 s; free when already there."""

    def travel(origin: str, destination: str) -> float:
        if not destination or origin == destination:
            return 0.0
        return seconds

    return travel


def distance_travel(
    path_distance: Callable[[str, str], Optional[float]], speed_mps: float = 0.35
) -> Callable[[str, str], float]:
    """
    Travel cost from real Nav2 path distance.

    ``path_distance`` should wrap ``nav.get_path_info``, which returns metres between two
    named areas without moving the robot. Falls back to the flat estimate when the query
    fails, so a nav hiccup never stalls planning.
    """
    fallback = constant_travel()

    def travel(origin: str, destination: str) -> float:
        if not destination or origin == destination:
            return 0.0
        try:
            metres = path_distance(origin, destination)
        except Exception:  # noqa: BLE001
            metres = None
        if metres is None:
            return fallback(origin, destination)
        return float(metres) / speed_mps

    return travel


def explain(candidates: list, limit: int = 6) -> str:
    """Human-readable ranking, for the run log and for debugging a bad choice."""
    lines = [f"{'objective':32} {'ev':>7} {'pts':>6} {'p':>6} {'sec':>7}"]
    for candidate in candidates[:limit]:
        lines.append(
            f"{candidate.label:32} {candidate.ev:7.2f} {candidate.points:6.0f} "
            f"{candidate.p:6.2f} {candidate.total_s:7.1f}"
        )
    return "\n".join(lines)


def as_dict(candidate: Candidate) -> dict:
    """Run-log friendly view of a decision."""
    return {
        "objective": candidate.objective.id,
        "target": _target_name(candidate.binding),
        "ev": round(candidate.ev, 3),
        "points": candidate.points,
        "p": round(candidate.p, 3),
        "expected_s": round(candidate.expected_s, 1),
        "travel_s": round(candidate.travel_s, 1),
        "fallback": candidate.use_fallback,
    }


def _target_name(binding: dict) -> Any:
    target = binding.get("obj") or binding.get("item")
    return getattr(target, "name", target)
