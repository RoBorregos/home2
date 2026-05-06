"""py_trees leaf behaviours that wrap GPSR subtask-manager methods.

Each leaf calls the method that matches its action's ``.action`` field
(e.g. ``"go_to"`` → ``go_to(command)``) on the first handler in
``subtask_handlers`` that defines it — the same dispatch model used by
``gpsr_task_manager.search_command``.

The leaves are synchronous: ``update()`` blocks for the duration of the
underlying subtask call, then returns SUCCESS/FAILURE. py_trees Timeout
decorators only check between ticks, so the *primary* timeout enforcement
lives inside the subtask managers themselves; the decorators are a
between-tick safety net.
"""

from typing import Any, Callable, Optional, Sequence

import py_trees

from task_manager.gpsr.merger import PlanAction
from task_manager.utils.status import Status


def _resolve_method(action_name: str, handlers: Sequence[Any]) -> Optional[Callable]:
    for h in handlers:
        method = getattr(h, action_name, None)
        if callable(method):
            return method
    return None


class ActionLeaf(py_trees.behaviour.Behaviour):
    """Leaf that runs one ``PlanAction`` to completion."""

    def __init__(
        self,
        plan_action: PlanAction,
        subtask_handlers: Sequence[Any],
        on_complete: Optional[Callable[[PlanAction, Any, Any], None]] = None,
        name: Optional[str] = None,
    ):
        kind = getattr(plan_action.action, "action", "?")
        super().__init__(name=name or f"{kind}#{plan_action.source_cmd}.{plan_action.source_idx}")
        self._plan_action = plan_action
        self._handlers = subtask_handlers
        self._on_complete = on_complete

    def update(self) -> py_trees.common.Status:
        action = self._plan_action.action
        kind = getattr(action, "action", "")
        method = _resolve_method(kind, self._handlers)
        if method is None:
            self.logger.error(f"No handler for action '{kind}'")
            return py_trees.common.Status.FAILURE
        try:
            outcome = method(action)
        except Exception as exc:  # noqa: BLE001 — we want the BT to handle it
            self.logger.error(f"{kind} raised: {exc}")
            return py_trees.common.Status.FAILURE

        # Subtask methods canonically return (status, result). Tolerate
        # methods that return only a status.
        if isinstance(outcome, tuple) and len(outcome) >= 1:
            status, result = outcome[0], outcome[1] if len(outcome) > 1 else None
        else:
            status, result = outcome, None

        if self._on_complete is not None:
            try:
                self._on_complete(self._plan_action, status, result)
            except Exception as exc:  # noqa: BLE001
                self.logger.warning(f"on_complete hook raised: {exc}")

        if status == Status.EXECUTION_SUCCESS:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class SequentialFallbackLeaf(py_trees.behaviour.Behaviour):
    """Leaf that runs a single command's fallback sequence to completion.

    Used inside the sequential-fallback branch when interleaving fails.
    Each leaf executes one source command's actions in original order,
    ignoring within-command failures (they are logged) so that subsequent
    commands still get a chance to run.
    """

    def __init__(
        self,
        per_command_actions: Sequence[PlanAction],
        subtask_handlers: Sequence[Any],
        on_complete: Optional[Callable[[PlanAction, Any, Any], None]] = None,
        name: Optional[str] = None,
    ):
        super().__init__(name=name or f"fallback_cmd#{per_command_actions[0].source_cmd}")
        self._actions = list(per_command_actions)
        self._handlers = subtask_handlers
        self._on_complete = on_complete

    def update(self) -> py_trees.common.Status:
        for pa in self._actions:
            kind = getattr(pa.action, "action", "")
            method = _resolve_method(kind, self._handlers)
            if method is None:
                self.logger.error(f"No handler for action '{kind}'")
                continue
            try:
                outcome = method(pa.action)
            except Exception as exc:  # noqa: BLE001
                self.logger.error(f"{kind} raised: {exc}")
                continue
            if isinstance(outcome, tuple) and len(outcome) >= 1:
                status = outcome[0]
                result = outcome[1] if len(outcome) > 1 else None
            else:
                status, result = outcome, None
            if self._on_complete is not None:
                try:
                    self._on_complete(pa, status, result)
                except Exception as exc:  # noqa: BLE001
                    self.logger.warning(f"on_complete hook raised: {exc}")
        # The fallback branch always reports SUCCESS — its job is to give
        # every command its chance, not to gate on per-action outcomes.
        return py_trees.common.Status.SUCCESS
