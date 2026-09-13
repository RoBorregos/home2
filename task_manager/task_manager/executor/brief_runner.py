"""
Execute a task brief: select, run, update, repeat, until the budget is spent.

This is what replaces a hand-written task manager. The 1759-line pick-and-place FSM
encoded one fixed order (clean the table, then breakfast) and never looked at the clock,
so it ran ~14 minutes against a 420 s limit and the 400-point pour cluster almost never
happened. Here the order is a consequence of the scoresheet, the measured capabilities
and the time left, recomputed after every completed objective.

The task-specific part is the YAML brief. This file is the same for every task.
"""

from typing import Any, Callable, Optional

import py_trees
import rclpy

from task_manager.executor.objective_tree import build_objective_tree
from task_manager.gpsr.skill_runner import SkillRunner
from task_manager.planner.brief import Brief, Step
from task_manager.planner.manifest import Manifest, load_manifest
from task_manager.planner.selector import (
    Candidate,
    as_dict,
    choose,
    distance_travel,
    explain,
    rank,
)
from task_manager.utils.colored_logger import CLog
from task_manager.utils.run_log import RunLog
from task_manager.utils.status import Status
from task_manager.utils.task_runner import TaskRunner
from task_manager.world.world_model import WorldModel


class States:
    WAIT_START = "wait_start"
    ON_START = "on_start"
    SELECT = "select"
    EXECUTE = "execute"
    WRAP_UP = "wrap_up"
    DONE = "done"


# leave enough budget to stow the arm and say something before the buzzer
WRAP_UP_RESERVE_S = 15.0


class BriefRunner(TaskRunner):
    """Task-agnostic executor driven by a brief."""

    def __init__(
        self,
        node_name: str,
        task: Any,
        brief: Brief,
        manifest_path: str = "",
        mock_areas: Optional[list] = None,
        guards: Optional[dict] = None,
    ):
        super().__init__(
            node_name=node_name,
            task=task,
            run_name=brief.task,
            budget_s=brief.budget_s,
            mock_areas=mock_areas,
        )
        self.brief = brief
        self.manifest: Manifest = load_manifest(manifest_path)
        self.world = WorldModel()
        self.guards = dict(guards or {})
        self.runner = SkillRunner()
        self.current_state = States.WAIT_START

        self._tree: Optional[py_trees.trees.BehaviourTree] = None
        self._candidate = None
        self._pending: list = []
        self._score_projection = 0.0
        self._fired_triggers: list = []

    # ---------------- triggers ----------------

    def fire_trigger(self, event: str) -> None:
        """
        Record that a reactive event happened (doorbell, waving customer).

        Deliberately does not preempt the objective in flight: interrupting a carry
        mid-arena drops the object, which is worth more negative points than a few
        seconds of delay. The event is honoured at the next selection instead.
        """
        if event not in {trigger.on for trigger in self.brief.triggers}:
            self.get_logger().warning(f"no trigger declared for event '{event}'")
            return
        if event not in self._fired_triggers:
            self._fired_triggers.append(event)
            RunLog.note("trigger_fired", event=event)
            CLog.fsm(self, "PLAN", f"Trigger '{event}' fired.")

    def _triggered_objective(self):
        """First fired trigger whose conditions hold, as an objective to run now."""
        for event in list(self._fired_triggers):
            for trigger in self.brief.triggers:
                if trigger.on != event or not self.world.satisfies(trigger.when):
                    continue
                objective = self.brief.objective(trigger.objective)
                if objective is None:
                    continue
                if objective.once and self.world.times_done(objective.id) > 0:
                    self._fired_triggers.remove(event)
                    break
                return event, objective
        return None, None

    # ---------------- travel cost ----------------

    def _travel_fn(self) -> Callable[[str, str], float]:
        """Real Nav2 distance when available, flat estimate when the query fails."""

        def path_distance(origin: str, destination: str) -> Optional[float]:
            status, payload = self.subtask_manager.nav.get_path_info(destination, "", origin, "")
            if status != Status.EXECUTION_SUCCESS or not isinstance(payload, dict):
                return None
            return payload.get("distance")

        return distance_travel(path_distance)

    # ---------------- main step ----------------

    def run(self) -> None:
        if self.current_state == States.WAIT_START:
            self.wait_for_start()
            self._pending = list(self.brief.on_start)
            self.set_state(States.ON_START)

        elif self.current_state == States.ON_START:
            if self._run_pending("on_start"):
                self.set_state(States.SELECT)

        elif self.current_state == States.SELECT:
            self._select()

        elif self.current_state == States.EXECUTE:
            self._tick_objective()

        elif self.current_state == States.WRAP_UP:
            if self._run_pending("on_deadline"):
                self.set_state(States.DONE)

        elif self.current_state == States.DONE:
            CLog.fsm(
                self,
                "STATE",
                f"Brief complete. Projected {self._score_projection:.0f} pts.",
                level="success",
            )
            self.runner.shutdown()
            self.finish()

    # ---------------- selection ----------------

    def _select(self) -> None:
        if self.out_of_time(WRAP_UP_RESERVE_S):
            CLog.fsm(self, "STATE", "Budget spent; wrapping up.", level="warn")
            RunLog.note("deadline_hit", elapsed_s=round(self.budget.elapsed(), 1))
            self._start_wrap_up()
            return

        remaining = self.budget.remaining() - WRAP_UP_RESERVE_S

        # a fired trigger is an obligation to the person in front of us, not an option,
        # so it jumps the expected-value queue
        event, triggered = self._triggered_objective()
        if triggered is not None:
            self._fired_triggers.remove(event)
            candidate = _forced_candidate(triggered)
            CLog.fsm(self, "PLAN", f"Answering trigger '{event}' with {triggered.id}")
        else:
            ranked = rank(
                self.brief, self.world, self.manifest, remaining, self._travel_fn(), self.guards
            )
            if ranked:
                CLog.fsm(self, "PLAN", "\n" + explain(ranked))
            candidate = choose(
                self.brief, self.world, self.manifest, remaining, self._travel_fn(), self.guards
            )

        if candidate is None:
            CLog.fsm(self, "STATE", "Nothing left worth doing.", level="warn")
            self._start_wrap_up()
            return

        self._candidate = candidate
        RunLog.note("objective_selected", **as_dict(candidate))
        CLog.fsm(self, "PLAN", f"Chose {candidate.label} (ev={candidate.ev:.2f})")

        steps = self._steps_for(candidate)
        root = build_objective_tree(
            steps=steps,
            binding=candidate.binding,
            subtask_manager=self.subtask_manager,
            runner=self.runner,
            on_complete=self._on_skill_complete,
            name=candidate.objective.id,
        )
        self._tree = py_trees.trees.BehaviourTree(root)
        try:
            self._tree.setup(timeout=15.0)
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f"tree setup raised: {error}")
        self.set_state(States.EXECUTE)

    def _steps_for(self, candidate) -> list:
        """The autonomous template, or the fallback's when the selector chose that path."""
        objective = candidate.objective
        if not candidate.use_fallback:
            return list(objective.template)
        fallback = self.brief.objective(objective.fallback)
        if fallback is not None:
            CLog.fsm(self, "PLAN", f"Using free fallback '{fallback.id}' instead.")
            return list(fallback.template)
        return [Step(skill=objective.fallback)]

    # ---------------- execution ----------------

    def _tick_objective(self) -> None:
        if self._tree is None:
            self.set_state(States.SELECT)
            return

        self._tree.tick()
        status = self._tree.root.status
        if status == py_trees.common.Status.RUNNING:
            # a hard overrun still has to stop: the arena removes an idle robot
            if self.out_of_time():
                self._tree.root.stop(py_trees.common.Status.INVALID)
                RunLog.note("objective_preempted", objective=self._candidate.objective.id)
                self._start_wrap_up()
            return

        objective = self._candidate.objective
        succeeded = status == py_trees.common.Status.SUCCESS
        target = self._candidate.binding.get("obj") or self._candidate.binding.get("item")
        self.world.note_attempt(objective.id, target)
        if succeeded:
            self.world.mark_done(objective.id)
            self.world.solve_category(objective.category)
            self._score_projection += self._candidate.p * self._candidate.points
        else:
            self._penalize(objective, target)

        RunLog.note(
            "objective_finished",
            objective=objective.id,
            success=succeeded,
            elapsed_s=round(self.budget.elapsed(), 1),
        )
        CLog.fsm(
            self,
            "STATE",
            f"{objective.id} {'succeeded' if succeeded else 'failed'} "
            f"at {self.budget.elapsed():.0f}s",
            level="success" if succeeded else "warn",
        )
        self._tree = None
        self._candidate = None
        self.set_state(States.SELECT)

    def _penalize(self, objective, target) -> None:
        """A repeating objective that failed must not be retried forever on the same target."""
        if target is None or not hasattr(target, "skipped"):
            return
        if self.world.attempts_for(objective.id, target) >= objective.max_attempts:
            target.skipped = True
            CLog.fsm(self, "STATE", f"Giving up on {target.name}.", level="warn")

    def _on_skill_complete(self, skill: str, args: dict, status: Any, result: Any) -> None:
        """Keep the world model honest after every individual skill call."""
        self.world.apply(skill, status)
        if status != Status.EXECUTION_SUCCESS:
            return
        if skill == "pick_object":
            self.world.pick_up(str(args.get("object_name", "")))
        elif skill in {"place", "place_on_shelf", "place_on_floor"}:
            self.world.put_down()
        elif skill == "go_to":
            self.world.arrive(str(args.get("location", "")), str(args.get("sublocation", "")))
        elif skill == "detect_objects":
            self._record_detections(result)

    def _record_detections(self, result: Any) -> None:
        """Fold a detection result into the world model, tolerating its shape."""
        if not result:
            return
        for detection in result:
            label = getattr(detection, "label", None) or getattr(detection, "name", None)
            if not label:
                continue
            self.world.observe(
                str(label),
                surface=self.world.sublocation or self.world.location,
                destination=self._destination_for(str(label)),
            )
        self.world.predicates.add("objects_known")

    def _destination_for(self, label: str) -> str:
        """Where this object belongs. Overridden per task where the brief needs more."""
        return "cabinet"

    # ---------------- shared helpers ----------------

    def _start_wrap_up(self) -> None:
        self._pending = list(self.brief.on_deadline)
        self._tree = None
        self.set_state(States.WRAP_UP)

    def _run_pending(self, label: str) -> bool:
        """Run queued steps one per tick. Returns True when the queue is empty."""
        if not self._pending:
            return True
        step = self._pending.pop(0)
        try:
            from task_manager.skills.registry import call_skill

            call_skill(self.subtask_manager, step.skill, **step.bind({}))
        except Exception as error:  # noqa: BLE001 — wrap-up must always finish
            self.get_logger().warning(f"{label} step {step.skill} failed: {error}")
        return not self._pending


def _forced_candidate(objective) -> Candidate:
    """Wrap a triggered objective as a candidate without consulting expected value."""
    return Candidate(
        objective=objective,
        binding={},
        points=objective.points,
        p=1.0,
        expected_s=0.0,
        travel_s=0.0,
    )


def spin_brief(node_factory, args=None) -> None:
    """Entry point mirroring utils.task_runner.spin_task, with a clean shutdown."""
    rclpy.init(args=args)
    node = node_factory()
    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.runner.shutdown()
        node.destroy_node()
        rclpy.shutdown()
