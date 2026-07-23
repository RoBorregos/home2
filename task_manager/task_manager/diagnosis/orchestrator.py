"""Fase 4: supervised diagnosis + corrective-action orchestrator.

Glues Fases 2→3→4 into one cycle:
    collect logs → parse errors → oracle diagnosis → resolve action →
    (SUPERVISED: propose & wait for human OK) → execute → re-verify health.

Default mode is SUPERVISED: the proposed action is printed and a human must
confirm before anything runs. Autonomous mode (`auto_heal=True`) executes
directly and is meant to be entered only from the FSM's MAINTENANCE state, which
stops the actuators first.

Everything degrades gracefully: if status.log_* is unreachable, diagnosis falls
back to the health result alone; if the oracle/Ollama is down, the oracle's own
rule-based fallback still yields an action."""

from __future__ import annotations

from dataclasses import dataclass, field

from task_manager.diagnosis import actions
from task_manager.diagnosis.oracle import Diagnosis, DiagnosisOracle


@dataclass
class Remediation:
    diagnosis: Diagnosis | None = None
    action: actions.CorrectiveAction | None = None
    executed: bool = False
    result: actions.ActionResult | None = None
    parsed_errors: list = field(default_factory=list)
    confirmed: bool = False
    message: str = ""


def _collect_and_parse(area: str | None):
    """Best-effort bridge to the status/ log collector + parser."""
    try:
        from status import log_collector, log_parser  # type: ignore
    except Exception:
        return []
    try:
        bundle = log_collector.collect()
        errors = log_parser.parse(bundle.text)
    except Exception:
        return []
    if area:
        area_errors = [e for e in errors if e.area == area]
        if area_errors:
            return area_errors
    return errors


class _SyntheticError:
    """Fallback 'error' when logs yield nothing but health says a node is missing."""

    def __init__(self, area: str | None, missing: list[str]):
        self.kind = "missing_node"
        self.area = area or ""
        node = missing[0] if missing else ""
        self.raw_line = f"node {node} not present in ROS graph (health check)"


class DiagnosisOrchestrator:
    def __init__(
        self,
        node=None,
        oracle: DiagnosisOracle | None = None,
        auto_heal: bool = False,
        dry_run: bool = False,
        confirm_fn=None,
    ):
        self.node = node
        self.oracle = oracle or DiagnosisOracle()
        self.auto_heal = auto_heal
        self.dry_run = dry_run
        # confirm_fn(remediation) -> bool. Default: console input.
        self.confirm_fn = confirm_fn or self._console_confirm

    def _log(self, level: str, msg: str):
        try:
            from task_manager.utils.colored_logger import CLog

            CLog.fsm(self.node, "HEAL", msg, level=level)
        except Exception:
            print(f"[HEAL:{level}] {msg}")

    def _console_confirm(self, rem: Remediation) -> bool:
        prompt = (
            f"\n[SUPERVISED HEAL] cause: {rem.diagnosis.razon_falla}\n"
            f"proposed: {rem.action.pretty()}\nExecute? [y/N] "
        )
        try:
            return input(prompt).strip().lower() in ("y", "yes", "s", "si")
        except (EOFError, KeyboardInterrupt):
            return False

    def run_diagnosis_cycle(self, health, area: str | None = None) -> Remediation:
        rem = Remediation()

        errors = _collect_and_parse(area)
        rem.parsed_errors = errors
        error = errors[0] if errors else _SyntheticError(area, getattr(health, "missing_nodes", []))

        rem.diagnosis = self.oracle.diagnose(error, health)
        self._log(
            "warn", f"diagnosis: {rem.diagnosis.razon_falla} → {rem.diagnosis.accion_sugerida}"
        )

        action = actions.resolve(rem.diagnosis.accion_sugerida)
        rem.action = action
        if action is None or action.kind == "none":
            rem.message = "no executable corrective action"
            self._log("error", rem.message)
            return rem

        # SUPERVISED (default): ask before executing.
        if not self.auto_heal:
            rem.confirmed = self.confirm_fn(rem)
            if not rem.confirmed:
                rem.message = "human declined the corrective action"
                self._log("warn", rem.message)
                return rem
        else:
            rem.confirmed = True

        self._log("warn", f"executing: {action.pretty()}")
        rem.result = actions.execute(action, dry_run=self.dry_run)
        rem.executed = True
        rem.message = "executed" if rem.result.ok else f"action failed ({rem.result.returncode})"
        self._log("success" if rem.result.ok else "error", rem.message)
        return rem
