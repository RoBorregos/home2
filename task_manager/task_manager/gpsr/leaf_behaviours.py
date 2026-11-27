"""Skill dispatch and gripper-aware fallback ordering for the execution tree."""

from typing import Any, Callable, List, Optional, Sequence, Tuple

from task_manager.gpsr.merger import PlanAction
from task_manager.gpsr.status import Status


def _resolve_method(action_name: str, handlers: Sequence[Any]) -> Optional[Callable]:
    for h in handlers:
        method = getattr(h, action_name, None)
        if callable(method):
            return method
    return None


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
    method: Optional[Callable],
    on_complete: Optional[Callable[[PlanAction, Any, Any], None]],
    logger: Any,
    on_start: Optional[Callable[[PlanAction], None]] = None,
    is_late: Optional[Callable[[], bool]] = None,
) -> Status:
    """Call ``method(plan_action.action)``, fire ``on_complete``, return Status.

    Converts missing handlers, exceptions and invalid outcomes into failures.
    """
    if on_start is not None:
        try:
            on_start(plan_action)
        except Exception:  # noqa: BLE001
            logger.error("on_start hook raised")

    action = plan_action.action
    kind = getattr(action, "action", "")
    status, result = Status.EXECUTION_ERROR, None
    if method is None:
        logger.error(f"No handler for action '{kind}'")
    else:
        try:
            outcome = method(action)
        except Exception:  # noqa: BLE001 — we want the BT to handle it
            logger.error(f"{kind} raised")
        else:
            parsed_status, result = _unpack_outcome(outcome)
            if parsed_status is None:
                logger.warning(f"{kind} returned non-Status outcome; treating as failure")
            else:
                status = parsed_status
    if is_late is not None and is_late():
        status, result = Status.TIMEOUT, None
    if on_complete is not None:
        try:
            on_complete(plan_action, status, result)
        except Exception:  # noqa: BLE001
            logger.error("on_complete hook raised")
    return status


def fallback_order(
    fallback: Sequence[Sequence[PlanAction]],
    is_completed: Optional[Callable[[PlanAction], bool]],
    *,
    holder: Optional[int] = None,
) -> List[int]:
    """Issue order, except the command holding an object runs right before the
    first other command that still has to pick (one gripper)."""
    order = list(range(len(fallback)))
    if is_completed is None:
        return order
    if holder is None:
        for c, actions in enumerate(fallback):
            held = False
            for pa in actions:
                if pa.requires_gripper and is_completed(pa):
                    held = True
                if pa.releases_gripper and is_completed(pa):
                    held = False
            if held:
                holder = c
    if holder is None:
        return order
    pickers = [
        c
        for c in order
        if c != holder and any(pa.requires_gripper and not is_completed(pa) for pa in fallback[c])
    ]
    if pickers and pickers[0] < holder:
        order.remove(holder)
        order.insert(order.index(pickers[0]), holder)
    return order
