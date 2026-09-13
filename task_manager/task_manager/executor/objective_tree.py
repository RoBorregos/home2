"""
Turn a selected objective into a behaviour tree of registry skill calls.

The GPSR leaves dispatch a single BAML command object into a handler method. Brief steps
are the other shape: a registered skill name plus keyword arguments. Same asynchronous
contract underneath — submit to the SkillRunner, tick RUNNING, so Timeout and Deadline
still work.
"""

from typing import Any, Callable, Optional, Sequence

import py_trees

from task_manager.gpsr.skill_runner import SkillRunner
from task_manager.gpsr.timeouts import timeout_for
from task_manager.planner.brief import Step
from task_manager.skills.registry import call_skill
from task_manager.utils.status import Status


class SkillLeaf(py_trees.behaviour.Behaviour):
    """Leaf that runs one registry skill with bound keyword arguments."""

    def __init__(
        self,
        skill: str,
        args: dict,
        subtask_manager: Any,
        runner: SkillRunner,
        on_complete: Optional[Callable[[str, dict, Any, Any], None]] = None,
        name: Optional[str] = None,
    ):
        super().__init__(name=name or skill)
        self._skill = skill
        self._args = args
        self._subtask_manager = subtask_manager
        self._runner = runner
        self._on_complete = on_complete
        self._future = None

    def _call(self):
        return call_skill(self._subtask_manager, self._skill, **self._args)

    def initialise(self) -> None:
        self._future = None

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            self._future = self._runner.submit(self._call)
            return py_trees.common.Status.RUNNING
        if not self._future.done():
            return py_trees.common.Status.RUNNING

        try:
            outcome = self._future.result()
        except Exception as error:  # noqa: BLE001 — a bad call fails the leaf, not the run
            self.logger.error(f"{self._skill} raised: {error}")
            self._notify(None, None)
            return py_trees.common.Status.FAILURE

        status, result = _split(outcome)
        self._notify(status, result)
        if status == Status.EXECUTION_SUCCESS:
            return py_trees.common.Status.SUCCESS
        self.logger.warning(f"{self._skill} returned {status}")
        return py_trees.common.Status.FAILURE

    def _notify(self, status: Any, result: Any) -> None:
        if self._on_complete is None:
            return
        try:
            self._on_complete(self._skill, self._args, status, result)
        except Exception:  # noqa: BLE001
            self.logger.error("on_complete hook raised")

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status == py_trees.common.Status.INVALID:
            self._runner.abandon(self._future)


def _split(outcome: Any) -> tuple:
    if isinstance(outcome, tuple) and outcome:
        return outcome[0], outcome[1] if len(outcome) > 1 else None
    return outcome, None


def build_objective_tree(
    steps: Sequence[Step],
    binding: dict,
    subtask_manager: Any,
    runner: SkillRunner,
    on_complete: Optional[Callable[[str, dict, Any, Any], None]] = None,
    retry_count: int = 1,
    name: str = "objective",
) -> py_trees.behaviour.Behaviour:
    """
    Sequence of the objective's steps, each with its own timeout and retries.

    Memory=True so a re-tick resumes at the step that was running rather than
    restarting the objective and repeating a pick.
    """
    sequence = py_trees.composites.Sequence(name=name, memory=True)
    for index, step in enumerate(steps):
        leaf = SkillLeaf(
            skill=step.skill,
            args=step.bind(binding),
            subtask_manager=subtask_manager,
            runner=runner,
            on_complete=on_complete,
            name=f"{step.skill}#{index}",
        )
        guarded = py_trees.decorators.Timeout(
            name=f"to({step.skill})",
            child=leaf,
            duration=timeout_for(step.skill),
        )
        sequence.add_child(
            py_trees.decorators.Retry(
                name=f"retry({step.skill})",
                child=guarded,
                num_failures=retry_count,
            )
        )
    return sequence
