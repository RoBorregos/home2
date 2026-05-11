"""Build a py_trees behaviour tree for an interleaved GPSR plan.

Tree shape:

    Selector "gpsr_root"
      ├── Timeout(GLOBAL_BUDGET_S)
      │     └── Sequence "interleaved"
      │           ├── Retry(2) ▸ Timeout(action_t) ▸ ActionLeaf(a0)
      │           ├── ...
      │           └── Retry(2) ▸ Timeout(action_t) ▸ ActionLeaf(aN)
      └── Sequence "sequential_fallback"
            ├── SequentialFallbackLeaf(cmd0)
            ├── SequentialFallbackLeaf(cmd1)
            └── ...

Failure is absolute in the interleaved branch: any leaf — gripper or
gripper-neutral — that exhausts its Retry attempts (or the global
Timeout firing) causes the inner Sequence to return FAILURE, and the
Selector advances to ``sequential_fallback``. The fallback then runs
each source command's actions in original order (within-command
failures are logged but ignored so every command still gets a chance).
Per-action results are forwarded via ``on_action_complete`` so the
HRI command-history reflects partial progress regardless of branch.
"""

from typing import Any, Callable, Optional, Sequence

import py_trees

from task_manager.gpsr.leaf_behaviours import ActionLeaf, SequentialFallbackLeaf
from task_manager.gpsr.merger import InterleavedPlan, PlanAction
from task_manager.gpsr.timeouts import GLOBAL_BUDGET_S, timeout_for


def _wrap_action(
    plan_action: PlanAction,
    handlers: Sequence[Any],
    on_complete: Optional[Callable[[PlanAction, Any, Any], None]],
    retry_count: int,
) -> py_trees.behaviour.Behaviour:
    leaf = ActionLeaf(plan_action, handlers, on_complete=on_complete)
    kind = getattr(plan_action.action, "action", "")
    timeout = py_trees.decorators.Timeout(
        name=f"to({kind})",
        child=leaf,
        duration=timeout_for(kind),
    )
    return py_trees.decorators.Retry(
        name=f"retry({kind})",
        child=timeout,
        num_failures=retry_count,
    )


def build_tree(
    plan: InterleavedPlan,
    subtask_handlers: Sequence[Any],
    on_action_complete: Optional[Callable[[PlanAction, Any, Any], None]] = None,
    retry_count: int = 2,
    global_budget_s: float = GLOBAL_BUDGET_S,
) -> py_trees.behaviour.Behaviour:
    """Build the GPSR behaviour tree for ``plan``.

    Args:
        plan: the merged interleaved plan + per-command fallbacks.
        subtask_handlers: ordered list of objects exposing methods named
            after each action kind (typically ``[gpsr_tasks,
            gpsr_individual_tasks]``).
        on_action_complete: optional callback invoked after every action
            with ``(plan_action, status, result)`` — wire this to
            ``hri.add_command_history`` so per-action results are logged.
        retry_count: per-action retry attempts.
        global_budget_s: wall-clock budget for the interleaved sequence.

    Returns:
        the root Selector behaviour ready for ``tick()``.
    """
    interleaved_seq = py_trees.composites.Sequence(name="interleaved", memory=True)
    for pa in plan.actions:
        interleaved_seq.add_child(
            _wrap_action(pa, subtask_handlers, on_action_complete, retry_count)
        )

    interleaved_branch = py_trees.decorators.Timeout(
        name="global_budget",
        child=interleaved_seq,
        duration=global_budget_s,
    )

    fallback_seq = py_trees.composites.Sequence(name="sequential_fallback", memory=True)
    for cmd_idx, per_cmd in enumerate(plan.fallback):
        if not per_cmd:
            continue
        fallback_seq.add_child(
            SequentialFallbackLeaf(
                per_cmd,
                subtask_handlers,
                on_complete=on_action_complete,
                name=f"fallback_cmd{cmd_idx}",
            )
        )

    root = py_trees.composites.Selector(name="gpsr_root", memory=True)
    root.add_child(interleaved_branch)
    if fallback_seq.children:
        root.add_child(fallback_seq)
    return root


def render_tree_ascii(root: py_trees.behaviour.Behaviour) -> str:
    """Return a unicode tree dump for sanity inspection (logs / debug)."""
    return py_trees.display.unicode_tree(root, show_status=False)
