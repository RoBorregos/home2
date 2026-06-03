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

from typing import Any, Callable, List, Optional, Sequence, Tuple

import py_trees

from task_manager.gpsr.merger import PlanAction
from task_manager.utils.status import Status


def _resolve_method(action_name: str, handlers: Sequence[Any]) -> Optional[Callable]:
    for h in handlers:
        method = getattr(h, action_name, None)
        if callable(method):
            return method
    return None


def _handler_names(handlers: Sequence[Any]) -> str:
    return ", ".join(type(h).__name__ for h in handlers) or "<none>"


def _unpack_outcome(outcome: Any) -> Tuple[Optional[Status], Any]:
    """Return ``(status, result)`` from a handler return value.

    Subtask methods canonically return ``(Status, result)``; we also
    tolerate methods that return a bare ``Status``. Anything else (wrong
    type, empty tuple, non-Status first element) yields ``(None, raw)``
    so the caller can warn instead of silently mapping to FAILURE.
    """
    if isinstance(outcome, tuple):
        status = outcome[0] if outcome else None
        result = outcome[1] if len(outcome) > 1 else None
    else:
        status, result = outcome, None
    if not isinstance(status, Status):
        return None, outcome
    return status, result


def _dispatch(
    plan_action: PlanAction,
    method: Callable,
    on_complete: Optional[Callable[[PlanAction, Any, Any], None]],
    logger: Any,
) -> Optional[Status]:
    """Call ``method(plan_action.action)``, fire ``on_complete``, return Status.

    Returns ``None`` if the handler raised or returned a non-Status value.
    """
    action = plan_action.action
    kind = getattr(action, "action", "")
    try:
        outcome = method(action)
    except Exception:  # noqa: BLE001 — we want the BT to handle it
        logger.error(f"{kind} raised")
        return None
    status, result = _unpack_outcome(outcome)
    if status is None:
        logger.warning(f"{kind} returned non-Status outcome {outcome!r}; treating as failure")
    if on_complete is not None:
        try:
            on_complete(plan_action, status, result)
        except Exception:  # noqa: BLE001
            logger.error("on_complete hook raised")
    return status


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
        self._method = _resolve_method(kind, subtask_handlers)
        self._on_complete = on_complete

    def update(self) -> py_trees.common.Status:
        if self._method is None:
            kind = getattr(self._plan_action.action, "action", "")
            self.logger.error(
                f"No handler for action '{kind}' in [{_handler_names(self._handlers)}]"
            )
            return py_trees.common.Status.FAILURE
        status = _dispatch(self._plan_action, self._method, self._on_complete, self.logger)
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
        if not per_command_actions:
            raise ValueError("SequentialFallbackLeaf requires at least one action")
        super().__init__(name=name or f"fallback_cmd#{per_command_actions[0].source_cmd}")
        self._handlers = subtask_handlers
        self._on_complete = on_complete
        # Resolve handlers once so a missing-method misconfiguration
        # surfaces here, in tree construction, rather than mid-tick.
        self._resolved: List[Tuple[PlanAction, Optional[Callable]]] = [
            (pa, _resolve_method(getattr(pa.action, "action", ""), subtask_handlers))
            for pa in per_command_actions
        ]
        self._started = False

    def update(self) -> py_trees.common.Status:
        if not self._started:
            self._started = True
            if self._resolved:
                cmd_idx = self._resolved[0][0].source_cmd
                self.logger.info(
                    f"fallback running cmd_idx={cmd_idx}, " f"{len(self._resolved)} actions"
                )
        for pa, method in self._resolved:
            if method is None:
                kind = getattr(pa.action, "action", "")
                self.logger.error(
                    f"No handler for action '{kind}' in [{_handler_names(self._handlers)}]"
                )
                continue
            _dispatch(pa, method, self._on_complete, self.logger)
        # The fallback branch always reports SUCCESS — its job is to give
        # every command its chance, not to gate on per-action outcomes.
        return py_trees.common.Status.SUCCESS


class OneShotCallbackLeaf(py_trees.behaviour.Behaviour):
    """Leaf that fires ``callback`` exactly once on its first update.

    Used at the head of the sequential-fallback branch so the operator gets
    a visible signal (on-screen text, log line, etc.) the moment the tree
    advances past the interleaved branch. Always returns SUCCESS so the
    enclosing Sequence advances unconditionally.
    """

    def __init__(self, callback: Callable[[], None], name: str = "one_shot"):
        super().__init__(name=name)
        self._callback = callback
        self._fired = False

    def update(self) -> py_trees.common.Status:
        if not self._fired:
            self._fired = True
            try:
                self._callback()
            except Exception:  # noqa: BLE001 — never let a debug hook tank the BT
                self.logger.error(f"{self.name} callback raised")
        return py_trees.common.Status.SUCCESS
