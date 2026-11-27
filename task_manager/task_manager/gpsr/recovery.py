"""Command-local recovery for the GPSR behaviour tree."""

from dataclasses import dataclass, field, replace
from enum import Enum
from time import monotonic
from types import SimpleNamespace
from typing import Callable, Optional, Sequence

import py_trees

from task_manager.gpsr.leaf_behaviours import _dispatch, _resolve_method, fallback_order
from task_manager.gpsr.merger import (
    InfeasibleResidual,
    PlanAction,
    RoomFn,
    _action_effects,
    _kind,
    _norm_location,
    _places_overlap,
)
from task_manager.gpsr.status import Status
from task_manager.gpsr.timeouts import timeout_for


class RecoveryPolicy(str, Enum):
    SEQUENTIAL_FALLBACK = "sequential-fallback"
    SKIP_ONLY = "skip-only"
    SKIP_DEFER = "skip-defer"


@dataclass
class ExecutionHooks:
    """Adapter for the clock and budgeted I/O. Gripper state comes from skill results.

    All deadlines are absolute values in clock() units. estimate_deferred
    prices the supplied sequence from the current pose and includes return.
    estimate_deferred(()) prices return alone. Use None for an unknown cost.
    """

    clock: Callable[[], float] = monotonic
    deadline: Optional[float] = None
    set_action_deadline: Optional[Callable[[Optional[float]], None]] = None
    estimate_deferred: Optional[Callable[[Sequence[PlanAction]], Optional[float]]] = None
    return_to_start: Optional[Callable[[float], bool]] = None
    report_failure: Optional[Callable[[PlanAction, Status, float], bool]] = None
    failure_report_s: float = 0.0
    on_phase: Optional[Callable[[str], None]] = None
    room_of: Optional[RoomFn] = None
    replan: Optional[
        Callable[[Sequence[Sequence[PlanAction]], Optional[int]], Sequence[PlanAction]]
    ] = None


OK = Status.EXECUTION_SUCCESS
FIND = {"find_person", "find_person_by_name"}
ESCORT = {"guide_person_to", "follow_person_until"}
REVISITABLE = FIND | {"count", "get_visual_info", "get_person_info"}
INFORMATION = REVISITABLE | {"answer_question"}
NEEDS_PERSON = ESCORT | {"get_person_info", "give_object", "answer_question"}


def _key(pa):
    return pa.source_cmd, pa.source_idx


def _goto(place, cmd):
    return PlanAction(
        SimpleNamespace(action="go_to", location_to_go=place),
        cmd,
        -1,
        place,
        False,
        False,
        purpose="revisit",
    )


@dataclass
class _Prerequisites:
    place: Optional[str]
    navigation: Optional[PlanAction]
    dependencies: list = field(default_factory=list)
    target: Optional[PlanAction] = None
    pick: Optional[PlanAction] = None


class _CommandRequirements:
    def __init__(self, actions, start):
        self.by_action = {}
        self.dependents = {}
        self.nav_goals = set()
        self.rooms = {}
        self.start = start
        for command in actions:
            self._add_command(command)

    def _add_command(self, command):
        place, nav = self.start, None
        pick, target = None, None
        inputs = []
        for pa in command:
            kind = _kind(pa.action)
            if kind == "go_to":
                place, nav = getattr(pa.action, "location_to_go", None), pa
                continue
            needs = _Prerequisites(place, nav, pick=pick)
            if pick is not None:
                needs.dependencies.append(pick)
            if kind in NEEDS_PERSON and target is not None:
                needs.target = target
                needs.dependencies.append(target)
            if kind == "say_with_context":
                needs.dependencies.extend(inputs)
                inputs = []
            else:
                inputs.append(pa)
            for dep in needs.dependencies:
                self.dependents.setdefault(_key(dep), {})[_key(pa)] = pa
            self.by_action[_key(pa)] = needs

            if kind in FIND:
                target = pa
            if kind in ESCORT:
                place, nav = _action_effects(pa.action, place)[2], pa
            if pa.requires_gripper:
                pick = pa
            if pa.releases_gripper:
                pick = None

        for i, pa in enumerate(command):
            if (
                _kind(pa.action) == "go_to"
                and pa.source_idx >= 0
                and (i + 1 == len(command) or _kind(command[i + 1].action) == "go_to")
            ):
                self.nav_goals.add(_key(pa))


@dataclass
class RecoveryFailure:
    action: PlanAction
    status: Status
    phase: str
    write_index: int
    deferred: Optional[str] = None
    recovery_status: Optional[Status] = None
    reported: bool = False


class _Phase(py_trees.behaviour.Behaviour):
    """Advance a phase by one synchronous skill attempt per tick."""

    def __init__(self, name, run):
        super().__init__(name)
        self._run = run

    def initialise(self):
        self._steps = iter(self._run())

    def update(self):
        try:
            next(self._steps)
        except StopIteration as end:
            return (
                py_trees.common.Status.FAILURE
                if end.value is False
                else py_trees.common.Status.SUCCESS
            )
        return py_trees.common.Status.RUNNING


class ExecutionTree(py_trees.composites.Sequence):
    """Run the merged order, resolve failures, and return on one clock."""

    def __init__(
        self,
        plan,
        handlers,
        on_complete,
        retry_count,
        budget,
        on_fallback,
        is_completed,
        on_start,
        policy,
        hooks,
    ):
        super().__init__(name=f"gpsr_root ({policy.value})", memory=True)
        self._plan = plan
        self._handlers = handlers
        self._on_complete = on_complete
        self._on_start = on_start
        self._on_fallback = on_fallback
        self._is_completed = is_completed
        self._attempts = max(1, retry_count)
        self._budget = budget
        self.policy = policy
        self.hooks = hooks

        commands = plan.fallback
        if not commands:
            by_cmd = {}
            for pa in plan.actions:
                by_cmd.setdefault(pa.source_cmd, []).append(pa)
            commands = [
                sorted(actions, key=lambda pa: pa.source_idx) for actions in by_cmd.values()
            ]
        start = plan.origin_name
        if start is None:
            start = next(
                (
                    pa.action.location_to_go
                    for cmd in commands
                    for pa in cmd
                    if pa.source_idx == -1 and _kind(pa.action) == "go_to"
                ),
                None,
            )
        self._requirements = _CommandRequirements(commands, start)
        self.add_children(
            [
                _Phase("interleaved", self._interleaved),
                _Phase("sequential_fallback", self._fallback),
                _Phase("defer_information", self._defer_information),
                _Phase("return", self._return),
                _Phase("report_failures", self._report_failures),
            ]
        )

    def initialise(self):
        self.deadline = self.hooks.deadline
        if self.deadline is None:
            self.deadline = self.hooks.clock() + self._budget
        self.failures = []
        self.completed = set()
        self.unreachable = set()
        self.replans = []
        self.skipped = {}
        self._writes = []
        self._place = _norm_location(self._requirements.start)
        self._holder = None  # key of the pick whose object is held, per skill results
        self.returned = None
        self._finished = False
        self._pending_information = []

    def _remaining(self):
        return self.deadline - self.hooks.clock()

    def _set_phase(self, phase):
        self._phase = phase
        if self.hooks.on_phase:
            self.hooks.on_phase(phase)

    def _done(self, pa):
        return _key(pa) in self.completed or (
            self._is_completed is not None and self._is_completed(pa)
        )

    def _skip_reason(self, pa, check_place=True):
        needs = self._requirements.by_action.get(_key(pa))
        if needs and any(not self._done(dep) for dep in needs.dependencies):
            return "prerequisite unavailable"
        if check_place and needs and needs.place and _norm_location(needs.place) != self._place:
            return "required location unavailable"
        if pa.requires_gripper and self._holder is not None:
            return "gripper is occupied"
        if pa.releases_gripper and (
            not needs or not needs.pick or self._holder != _key(needs.pick)
        ):
            return "no picked object for this release"
        return None

    def _nav_needed(self, nav, following, resume):
        if (nav.navigation_goals or _key(nav) in self._requirements.nav_goals) and not (
            resume and self._done(nav)
        ):
            return True
        destination = _norm_location(getattr(nav.action, "location_to_go", None))
        for pa in following:
            kind = _kind(pa.action)
            if kind == "go_to":
                break
            needs = self._requirements.by_action.get(_key(pa))
            if needs and needs.place and _norm_location(needs.place) != destination:
                continue
            if resume and self._done(pa):
                continue
            if not self._skip_reason(pa, check_place=False):
                return True
        return False

    def _attempt(self, pa):
        if self._remaining() <= 0:
            return Status.TIMEOUT
        kind = _kind(pa.action)
        method = _resolve_method(kind, self._handlers)
        limit = min(self.deadline, self.hooks.clock() + timeout_for(kind))
        if self.hooks.set_action_deadline:
            self.hooks.set_action_deadline(limit)
        try:
            status = _dispatch(
                pa,
                method,
                self._on_complete,
                self.logger,
                self._on_start,
                is_late=lambda: self.hooks.clock() > limit,
            )
        finally:
            if self.hooks.set_action_deadline:
                self.hooks.set_action_deadline(None)

        _, writes, next_place = _action_effects(pa.action, self._place)
        self._writes.extend(writes)
        if kind == "go_to" or kind in ESCORT:
            self._place = next_place if status == OK else None
        # A failed pick leaves the gripper empty; a failed release leaves it held.
        if status == OK and (pa.requires_gripper or pa.releases_gripper):
            self._holder = _key(pa) if pa.requires_gripper else None
        if status == OK and pa.purpose in ("plan", "deferred", "operator-return"):
            self.completed.add(_key(pa))
            self.skipped.pop(_key(pa), None)
        yield
        return status

    def _remaining_commands(self, tail):
        """Restore segment entry drives after shared navigation was collapsed."""
        count = len(self._plan.fallback)
        remaining = [[] for _ in range(count)]
        places = [None] * count
        blocked = {_key(f.action) for f in self.failures} | set(self.skipped)
        # A failed release can strand an object. Keep independent information
        # work in the residual optimization even when no pick is possible.
        stranded = self._holder is not None and not any(
            pa.releases_gripper
            and (pick := self._requirements.by_action[_key(pa)].pick)
            and _key(pick) == self._holder
            and _key(pa) not in blocked
            for pa in tail
        )
        for pa in tail:
            cmd = pa.source_cmd
            key = _key(pa)
            kind = _kind(pa.action)
            if kind == "go_to":
                # A collapsed drive can satisfy multiple standalone navigation goals.
                if _norm_location(pa.location) in self.unreachable:
                    continue
                goals = pa.navigation_goals
                if not goals and key in self._requirements.nav_goals:
                    goals = (key,)
                for owner, idx in goals:
                    remaining[owner].append(
                        replace(
                            pa, source_cmd=owner, source_idx=idx, navigation_goals=((owner, idx),)
                        )
                    )
                    places[owner] = pa.location
                continue
            needs = self._requirements.by_action[key]
            if (
                key in blocked
                or (stranded and (pa.requires_gripper or pa.releases_gripper))
                or _norm_location(needs.place) in self.unreachable
                or any(_key(dep) in blocked for dep in needs.dependencies)
                or (
                    needs.navigation
                    and _key(needs.navigation) in blocked
                    and _kind(needs.navigation.action) in ESCORT
                )
            ):
                self.skipped[key] = "unavailable after failure"
                blocked.add(key)
                continue
            if self._done(pa):
                continue
            if needs.place and needs.place != places[cmd]:
                drive = needs.navigation
                if drive is None or _kind(drive.action) != "go_to":
                    drive = replace(_goto(needs.place, cmd), purpose="replan-drive")
                remaining[cmd].append(drive)
                places[cmd] = needs.place
            remaining[cmd].append(pa)
            if kind in ESCORT:
                places[cmd] = _action_effects(pa.action, needs.place)[2]
        return remaining

    def _run_order(self, actions, resume=False):
        sequential = self.policy == RecoveryPolicy.SEQUENTIAL_FALLBACK
        for i, pa in enumerate(actions):
            if self._remaining() <= 0:
                return False
            kind = _kind(pa.action)
            if kind == "go_to":
                if _norm_location(pa.location) in self.unreachable:
                    self.skipped[_key(pa)] = "destination unreachable"
                    continue
                if (
                    resume or (not sequential and (self.failures or self.skipped))
                ) and not self._nav_needed(pa, actions[i + 1 :], resume):
                    self.skipped[_key(pa)] = "no pending work at destination"
                    continue
            else:
                if (
                    resume
                    and self._done(pa)
                    and (self._is_completed or pa.requires_gripper or pa.releases_gripper)
                ):
                    continue
                reason = self._skip_reason(pa)
                if reason:
                    self.skipped[_key(pa)] = reason
                    continue

            attempts = 1
            previously_exhausted = any(_key(f.action) == _key(pa) for f in self.failures)
            if kind in INFORMATION and not (resume and previously_exhausted):
                attempts = 2
            elif not resume and (sequential or kind == "go_to"):
                attempts = self._attempts
            for _ in range(attempts):
                status = yield from self._attempt(pa)
                if status == OK or self._remaining() <= 0:
                    break
                if kind != "go_to" and self._skip_reason(pa):
                    break
            if status == OK:
                continue
            self.failures.append(RecoveryFailure(pa, status, self._phase, len(self._writes)))
            if (
                not sequential
                and not resume
                and self.hooks.replan
                and self._plan.fallback
                and self._remaining() > 0
            ):
                if kind == "go_to":
                    self.unreachable.add(_norm_location(pa.location))
                holder = self._holder[0] if self._holder else None
                pending = self._remaining_commands(actions[i + 1 :])
                try:
                    revised = self.hooks.replan(pending, holder)
                except InfeasibleResidual:
                    # Existing dependency and gripper guards still apply to the original tail.
                    continue
                self.replans.append(
                    {
                        "after": _key(pa),
                        "holder": holder,
                        "remaining_s": self._remaining(),
                        "order": [_key(step) for step in revised],
                    }
                )
                return (yield from self._run_order(revised))
            if not resume and (sequential or kind == "go_to"):
                return False
        return True

    def _fallback_order(self):
        commands = self._plan.fallback
        if self._holder is None:
            return commands
        return [commands[i] for i in fallback_order(commands, self._done, holder=self._holder[0])]

    def _restoration(self, failure):
        pa = failure.action
        key = _key(pa)
        needs = self._requirements.by_action.get(key)
        if (
            _kind(pa.action) not in REVISITABLE
            or not needs
            or not needs.place
            or needs.pick
            or (needs.navigation and _kind(needs.navigation.action) != "go_to")
        ):
            return None

        actions = [pa]
        if _kind(pa.action) == "get_person_info":
            target = needs.target
            if not target or not self._done(target):
                return None
            target_needs = self._requirements.by_action[_key(target)]
            if target_needs.pick or _norm_location(target_needs.place) != _norm_location(
                needs.place
            ):
                return None
            actions.insert(0, replace(target, purpose="reacquire"))

        reports = {}
        for dependent in self._requirements.dependents.get(key, {}).values():
            dep_kind = _kind(dependent.action)
            dep_key = _key(dependent)
            if self._done(dependent):
                continue
            if dep_kind == "say_with_context":
                reports[dep_key] = dependent
            elif dep_kind == "get_person_info" and dep_key in self.skipped:
                dep_needs = self._requirements.by_action[dep_key]
                if _norm_location(dep_needs.place) != _norm_location(needs.place):
                    return None
                actions.append(dependent)
                for report_key, report in self._requirements.dependents.get(dep_key, {}).items():
                    if _kind(report.action) != "say_with_context":
                        return None
                    if not self._done(report):
                        reports[report_key] = report
            else:
                return None
        restored = {_key(action) for action in actions}
        if any(
            not self._done(dep) and _key(dep) not in restored
            for report in reports.values()
            for dep in self._requirements.by_action[_key(report)].dependencies
        ):
            return None

        # A successful reacquisition supplies fresh attention to the next
        # observation. Earlier writes still invalidate any *external* input,
        # such as a person having been moved away from the search location.
        restored_writes = set()
        for action in actions:
            a_needs = self._requirements.by_action[_key(action)]
            reads, writes, _ = _action_effects(action.action, _norm_location(a_needs.place))
            if any(
                r[0] == w[0] and self._overlap(r[1], w[1])
                for r in reads - restored_writes
                for w in self._writes[failure.write_index :]
            ):
                return None
            restored_writes.update(writes)

        steps = [_goto(needs.place, pa.source_cmd)]
        steps.extend(
            replace(a, purpose="deferred")
            if a.purpose == "plan" and (reports or i < len(actions) - 1)
            else a
            for i, a in enumerate(actions)
        )
        place = needs.place
        for report in sorted(reports.values(), key=lambda p: p.source_idx):
            report_place = self._requirements.by_action[_key(report)].place
            if not report_place:
                return None
            if _norm_location(report_place) != _norm_location(place):
                steps.append(_goto(report_place, pa.source_cmd))
                place = report_place
            steps.append(report)
        return steps

    def _overlap(self, a, b):
        if self.hooks.room_of is None:
            return True
        rooms = self._requirements.rooms
        for place in (a, b):
            if place is not None and place not in rooms:
                rooms[place] = _norm_location(self.hooks.room_of(place))
        return _places_overlap(a, b, rooms)

    def _fits(self, steps, extra=0.0):
        if self.hooks.estimate_deferred is None:
            return False
        estimate = self.hooks.estimate_deferred(steps)
        return estimate is not None and 0 <= estimate + extra <= self._remaining()

    def _defer_information(self):
        by_action = {}
        for failure in self.failures:
            if _kind(failure.action.action) in INFORMATION and not self._done(failure.action):
                by_action.setdefault(_key(failure.action), failure)
        self._pending_information = list(by_action.values())
        if self.policy == RecoveryPolicy.SKIP_DEFER:
            self._set_phase("deferred")
            for failure in self._pending_information:
                steps = self._restoration(failure)
                if steps is not None and any(
                    _norm_location(step.location) in self.unreachable
                    for step in steps
                    if _kind(step.action) == "go_to"
                ):
                    steps = None
                if steps is None:
                    failure.deferred = "ineligible"
                    continue
                if not self._fits(steps, self.hooks.failure_report_s):
                    failure.deferred = "insufficient_time"
                    continue
                failure.deferred = "failed"
                for pa in steps:
                    status = yield from self._attempt(pa)
                    if status != OK:
                        failure.recovery_status = status
                        break
                else:
                    failure.deferred = "recovered"

    def _report_failures(self):
        if self.hooks.report_failure and self.returned is not False:
            self._set_phase("failure_report")
            for failure in self._pending_information:
                unresolved = not self._done(failure.action) or failure.deferred == "failed"
                if unresolved and self._fits((), self.hooks.failure_report_s):
                    failure.reported = (
                        self.hooks.report_failure(
                            failure.action, failure.recovery_status or failure.status, self.deadline
                        )
                        and self._remaining() >= 0
                    )
                    yield
        return self._finished

    def _interleaved(self):
        self._set_phase("interleaved")
        self._finished = yield from self._run_order(self._plan.actions)

    def _fallback(self):
        if not self._finished and self._plan.fallback and self._remaining() > 0:
            self._set_phase("sequential_fallback")
            if self._on_fallback:
                try:
                    self._on_fallback()
                except Exception:
                    self.logger.error("on_fallback_entry hook raised")
            for actions in self._fallback_order():
                self._finished = yield from self._run_order(actions, resume=True)

    def _return(self):
        if self.hooks.return_to_start:
            self._set_phase("return")
            self.returned = (
                self._remaining() >= 0
                and self.hooks.return_to_start(self.deadline)
                and self._remaining() >= 0
            )
            self._finished = self._finished and self.returned
            yield
