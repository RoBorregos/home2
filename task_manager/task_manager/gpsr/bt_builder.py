"""Build the behaviour tree for an interleaved GPSR batch."""

from typing import Any, Callable, Optional, Sequence

import py_trees

from task_manager.gpsr.merger import InterleavedPlan, PlanAction
from task_manager.gpsr.recovery import ExecutionHooks, ExecutionTree, RecoveryPolicy
from task_manager.gpsr.timeouts import GLOBAL_BUDGET_S


def build_tree(
    plan: InterleavedPlan,
    subtask_handlers: Sequence[Any],
    on_action_complete: Optional[Callable[[PlanAction, Any, Any], None]] = None,
    retry_count: int = 2,
    global_budget_s: float = GLOBAL_BUDGET_S,
    on_fallback_entry: Optional[Callable[[], None]] = None,
    is_completed: Optional[Callable[[PlanAction], bool]] = None,
    on_action_start: Optional[Callable[[PlanAction], None]] = None,
    *,
    recovery_policy: RecoveryPolicy = RecoveryPolicy.SEQUENTIAL_FALLBACK,
    execution_hooks: Optional[ExecutionHooks] = None,
) -> ExecutionTree:
    """Build a tickable tree for one batch.

    Action hooks fire around each actual attempt. The returned behaviour also
    records exhausted failures and their recovery outcomes by source command.
    An optional completion predicate supplies prior progress for fallback.
    Information always gets two attempts in the merged pass. Other retries
    use retry_count on the sequential policy, or for explicit navigation.

    Skills are synchronous. A time-aware adapter must enforce the absolute
    deadline received through ExecutionHooks inside each blocking skill.
    """
    return ExecutionTree(
        plan,
        subtask_handlers,
        on_action_complete,
        retry_count,
        global_budget_s,
        on_fallback_entry,
        is_completed,
        on_action_start,
        RecoveryPolicy(recovery_policy),
        execution_hooks or ExecutionHooks(),
    )


def render_tree_ascii(root: py_trees.behaviour.Behaviour) -> str:
    """Return a tree dump for the execution log."""
    return py_trees.display.unicode_tree(root, show_status=False)
