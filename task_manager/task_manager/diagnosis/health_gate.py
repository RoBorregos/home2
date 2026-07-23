"""Fase 5: health-gate for subtask execution.

Before the FSM runs a subtask, `guard(action)` checks that the nodes that
subtask depends on (from config/subtasks_catalog.json) are alive. Outcomes:

    PROCEED  — healthy, run the subtask normally.
    WARN     — unhealthy but gate is passive (or heal disabled): the FSM is told,
               logs a warning, and proceeds (never worse than today's behavior).
    RETRY    — orchestrator healed the fault; re-run the subtask. (Fase 6)
    REPLAN   — could not heal; FSM should re-plan a fallback. (Fase 6)
    SKIP     — could not heal and no fallback; skip this subtask. (Fase 6)

Robustness: the status/ engine lives at the repo root and may not be on the
PYTHONPATH once task_manager is installed via colcon. This module imports it
lazily and, if unavailable, `guard()` always returns PROCEED — the gate simply
disappears rather than breaking the task manager."""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

# Decisions returned by guard()
PROCEED = "PROCEED"  # healthy (or gate disabled) → run the subtask
WARN = "WARN"  # unhealthy, passive mode → warn and run anyway
NEEDS_HEAL = "NEEDS_HEAL"  # unhealthy, active mode → hand off to MAINTENANCE
# Decisions the FSM's MAINTENANCE state resolves to
RETRY = "RETRY"  # healed → retry the subtask
REPLAN = "REPLAN"  # could not heal → re-plan a fallback
SKIP = "SKIP"  # could not heal, no fallback → skip the subtask


def _resolve_catalog() -> Path:
    """Locate subtasks_catalog.json across source-tree and colcon-install layouts."""
    # 1. Installed share dir (ros2 run executes the installed copy).
    try:
        from ament_index_python.packages import get_package_share_directory

        cand = (
            Path(get_package_share_directory("task_manager")) / "config" / "subtasks_catalog.json"
        )
        if cand.is_file():
            return cand
    except Exception:
        pass
    # 2. Source tree / mounted repo: <pkg>/config/subtasks_catalog.json
    src = Path(__file__).resolve().parents[2] / "config" / "subtasks_catalog.json"
    if src.is_file():
        return src
    # 3. Walk up looking for task_manager/config/subtasks_catalog.json
    for parent in Path(__file__).resolve().parents:
        cand = parent / "task_manager" / "config" / "subtasks_catalog.json"
        if cand.is_file():
            return cand
    return src  # non-existent; loader returns {} and the gate no-ops


_DEFAULT_CATALOG = _resolve_catalog()


def _import_health_monitor():
    """Import status.health_monitor, injecting the repo root onto sys.path if needed.

    Returns the module, or None if the status engine can't be reached (e.g. rclpy
    missing, or running outside the source tree)."""
    try:
        from status import health_monitor  # type: ignore

        return health_monitor
    except Exception:
        pass
    # Walk up looking for a dir that contains status/health_monitor.py
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "status" / "health_monitor.py"
        if candidate.is_file():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            try:
                from status import health_monitor  # type: ignore

                return health_monitor
            except Exception:
                return None
    return None


@dataclass
class GateResult:
    decision: str
    action: str
    ok: bool
    missing_nodes: list[str] = field(default_factory=list)
    message: str = ""
    remediation: object = None  # Remediation from orchestrator, when healed


class HealthGate:
    """Guards subtask execution against the health of its critical nodes.

    Args:
        node: the rclpy node (used only for logging via CLog).
        catalog_path: override for subtasks_catalog.json.
        passive: if True (default for Fase 5a), unhealthy → WARN and proceed.
                 Set False + provide `orchestrator` to enable heal/retry (Fase 6).
        orchestrator: DiagnosisOrchestrator for active mode.
    """

    def __init__(
        self,
        node=None,
        catalog_path: Path | None = None,
        passive: bool = True,
        check_dds: bool = False,
    ):
        self.node = node
        self.passive = passive
        self.check_dds = check_dds
        self.catalog = self._load_catalog(catalog_path or _DEFAULT_CATALOG)

        self._hm_module = _import_health_monitor()
        self.monitor = None
        if self._hm_module is not None:
            try:
                self.monitor = self._hm_module.HealthMonitor()
            except Exception:
                self.monitor = None
        self.available = self.monitor is not None

    @staticmethod
    def _load_catalog(path: Path) -> dict:
        try:
            data = json.loads(Path(path).read_text())
        except (OSError, ValueError):
            return {}
        return {k: v for k, v in data.items() if not k.startswith("_")}

    def _log(self, level: str, msg: str):
        try:
            from task_manager.utils.colored_logger import CLog

            CLog.fsm(self.node, "HEALTH", msg, level=level)
        except Exception:
            print(f"[HEALTH:{level}] {msg}")

    def guard(self, action: str) -> GateResult:
        """Check the health prerequisites for `action` and decide what to do."""
        entry = self.catalog.get(action)
        if entry is None:
            # Unknown skill → don't block; nothing declared to check.
            return GateResult(PROCEED, action, ok=True, message="no catalog entry")

        if not self.available:
            # status engine unreachable → gate disabled, behave as before.
            return GateResult(PROCEED, action, ok=True, message="health monitor unavailable")

        critical = entry.get("critical_nodes", [])
        is_remote = bool(entry.get("remote"))
        res = self.monitor.check_subtask_health(critical, check_dds=self.check_dds)

        # rclpy itself unavailable → can't judge; proceed.
        if not res.rclpy_available:
            return GateResult(PROCEED, action, ok=True, message="rclpy unavailable")

        if res.ok:
            return GateResult(PROCEED, action, ok=True, missing_nodes=[])

        # Remote-area nodes (e.g. HRI on the Orin) don't appear in the local graph;
        # a "missing" here is expected, so don't act on it.
        if is_remote:
            self._log(
                "warn",
                f"{action}: remote nodes not visible locally ({', '.join(res.missing_nodes)}); proceeding",
            )
            return GateResult(PROCEED, action, ok=True, missing_nodes=res.missing_nodes)

        self._log("error", f"{action}: unhealthy — {res.summary()}")

        if self.passive:
            # Never worse than today: warn and let the FSM proceed.
            return GateResult(
                WARN,
                action,
                ok=False,
                missing_nodes=res.missing_nodes,
                message=res.summary(),
            )

        # Active mode: hand off to the FSM's MAINTENANCE state, which stops
        # actuators before running the (possibly slow) corrective action.
        return GateResult(
            NEEDS_HEAL,
            action,
            ok=False,
            missing_nodes=res.missing_nodes,
            message=res.summary(),
        )

    def critical_nodes_for(self, action: str) -> tuple[list[str], bool, str | None]:
        """Return (critical_nodes, is_remote, area) for an action from the catalog."""
        entry = self.catalog.get(action) or {}
        return (
            entry.get("critical_nodes", []),
            bool(entry.get("remote")),
            entry.get("area"),
        )

    def recheck(self, action: str) -> "object":
        """Fresh health check for an action's critical nodes (used after healing)."""
        critical, _remote, _area = self.critical_nodes_for(action)
        if not self.available:
            return GateResult(PROCEED, action, ok=True, message="health monitor unavailable")
        return self.monitor.check_subtask_health(critical, check_dds=self.check_dds, force=True)
