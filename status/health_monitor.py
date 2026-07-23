"""Programmatic health facade over the existing status/ engine.

The dashboard (status/dashboard.py) renders a live TUI, and scripts/status.sh
does one-shot checks. Neither is callable *in the loop* by the task manager FSM.
HealthMonitor exposes the same underlying snapshots (infra_checks +
ros_introspection) as plain Python so the FSM can gate each subtask on the health
of its critical nodes.

This module is the single source of truth for "which expected nodes are running"
— dashboard.py imports compute_area_states() from here so the TUI and the FSM can
never disagree.

Usage:
    from status.health_monitor import HealthMonitor
    hm = HealthMonitor()
    res = hm.check_subtask_health(["/bt_navigator", "/controller_server"])
    if not res.ok:
        print(res.missing_nodes)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from status import area_config, infra_checks, ros_introspection


@dataclass
class HealthResult:
    """Outcome of a health check for a set of critical nodes."""

    ok: bool
    critical_nodes: list[str] = field(default_factory=list)
    running_nodes: list[str] = field(default_factory=list)
    missing_nodes: list[str] = field(default_factory=list)
    dds_ok: bool = True
    containers_down: list[str] = field(default_factory=list)
    rclpy_available: bool = True
    error: str = ""

    def summary(self) -> str:
        if self.error:
            return f"health check error: {self.error}"
        if self.ok:
            return (
                f"ok ({len(self.running_nodes)}/{len(self.critical_nodes)} nodes alive)"
            )
        parts = []
        if self.missing_nodes:
            parts.append("missing nodes: " + ", ".join(self.missing_nodes))
        if not self.dds_ok:
            parts.append("DDS host config incomplete")
        if self.containers_down:
            parts.append("containers down: " + ", ".join(self.containers_down))
        return "; ".join(parts) or "unhealthy"


def compute_area_states(
    areas: list[str], task: str, running_nodes: set[str]
) -> dict[str, dict]:
    """Expected vs running nodes per area for a given task.

    Extracted from dashboard.py so the TUI and the FSM share one implementation.
    `task` is the status.sh-style flag (e.g. "--gpsr"); empty string means no
    expectations are declared and every area comes back with empty lists.
    """
    out: dict[str, dict] = {}
    for area in areas:
        expected = area_config.load_area_nodes(area, task) if task else []
        running = [n for n in expected if n in running_nodes]
        missing = [n for n in expected if n not in running_nodes]
        out[area] = {"expected": expected, "running": running, "missing": missing}
    return out


class HealthMonitor:
    """In-loop health facade for the task manager.

    Snapshots are moderately expensive (~150-300 ms because ros_introspection
    spins up a short-lived probe node). `cache_ttl` coalesces rapid successive
    checks — the FSM may gate several subtasks in quick succession.
    """

    def __init__(self, hz_window: float = 0.0, cache_ttl: float = 1.0):
        self._hz_window = hz_window
        self._cache_ttl = cache_ttl
        self._ros_cache: ros_introspection.RosSnapshot | None = None
        self._ros_cache_at: float = 0.0

    # ── snapshots ────────────────────────────────────────────────────
    def ros_snapshot(self, force: bool = False) -> ros_introspection.RosSnapshot:
        now = time.monotonic()
        if (
            not force
            and self._ros_cache is not None
            and (now - self._ros_cache_at) < self._cache_ttl
        ):
            return self._ros_cache
        self._ros_cache = ros_introspection.snapshot(hz_window=self._hz_window)
        self._ros_cache_at = now
        return self._ros_cache

    def running_nodes(self, force: bool = False) -> set[str]:
        return set(self.ros_snapshot(force=force).nodes)

    # ── queries ──────────────────────────────────────────────────────
    def nodes_alive(self, expected: list[str], force: bool = False) -> dict[str, bool]:
        running = self.running_nodes(force=force)
        return {n: (n in running) for n in expected}

    def check_subtask_health(
        self,
        critical_nodes: list[str],
        check_dds: bool = False,
        containers: list[str] | None = None,
        force: bool = False,
    ) -> HealthResult:
        """Health of the nodes a subtask depends on.

        DDS and container checks are opt-in (off by default) so the common
        per-subtask gate stays fast — a missing node is the signal that matters
        mid-task; DDS/containers are for the diagnosis stage.
        """
        snap = self.ros_snapshot(force=force)
        if not snap.rclpy_available:
            return HealthResult(
                ok=False,
                critical_nodes=list(critical_nodes),
                rclpy_available=False,
                error=snap.error,
            )

        running_set = set(snap.nodes)
        running = [n for n in critical_nodes if n in running_set]
        missing = [n for n in critical_nodes if n not in running_set]

        dds_ok = True
        if check_dds:
            dds_ok = infra_checks.check_dds().ok

        containers_down: list[str] = []
        if containers:
            states = infra_checks.check_containers(containers)
            containers_down = [n for n, cs in states.items() if not cs.ok]

        ok = not missing and dds_ok and not containers_down
        return HealthResult(
            ok=ok,
            critical_nodes=list(critical_nodes),
            running_nodes=running,
            missing_nodes=missing,
            dds_ok=dds_ok,
            containers_down=containers_down,
            error=snap.error,
        )

    def get_area_for_node(self, node_name: str, task: str = "") -> str | None:
        """Best-effort reverse lookup node → area via status/configs/*_nodes.cfg.

        Only resolves when a task flag is given (the .cfg maps are keyed by task).
        Returns None if the node is not declared for that task in any area.
        """
        if not task:
            return None
        for area in ("navigation", "manipulation", "vision", "hri"):
            if node_name in area_config.load_area_nodes(area, task):
                return area
        return None
