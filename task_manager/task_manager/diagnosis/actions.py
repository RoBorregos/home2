"""Fase 4: closed catalog of corrective actions.

The oracle may only propose an `accion_sugerida` drawn from this vocabulary, and
the orchestrator can only execute what is defined here. Keeping the action space
closed is the safety boundary: an LLM cannot invent an arbitrary shell command.

Action strings are `kind` or `kind:arg`, e.g.:
    restart_container:navigation   → ./run.sh navigation --recreate
    rebuild_package:task_manager   → colcon build --packages-select task_manager
    reset_serial:navigation        → bash docker/navigation/setup-USB.sh
    restart_dds                    → sudo bash scripts/setup_cyclonedds.sh
    none                           → no automatic action; report to a human
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass, field
from pathlib import Path


def _repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "run.sh").is_file() and (parent / "docker").is_dir():
            return parent
    # Inside the container the repo is mounted at /workspace/src
    ws = Path("/workspace/src")
    return ws if (ws / "run.sh").is_file() else Path.cwd()


REPO_ROOT = _repo_root()


@dataclass
class ActionSpec:
    kind: str
    needs_arg: bool
    description: str
    builder: object  # callable(arg: str) -> list[str]
    requires_confirmation: bool = True


def _restart_container(area: str) -> list[str]:
    return ["./run.sh", area, "--recreate"]


def _rebuild_package(pkg: str) -> list[str]:
    return ["colcon", "build", "--packages-select", pkg]


def _reset_serial(_area: str) -> list[str]:
    return ["bash", "docker/navigation/setup-USB.sh"]


def _restart_dds(_arg: str) -> list[str]:
    return ["sudo", "bash", "scripts/setup_cyclonedds.sh"]


ACTION_SPECS: dict[str, ActionSpec] = {
    "restart_container": ActionSpec(
        "restart_container",
        True,
        "Recreate the Docker container for an area (./run.sh <area> --recreate)",
        _restart_container,
    ),
    "rebuild_package": ActionSpec(
        "rebuild_package",
        True,
        "Clean-rebuild a colcon package (colcon build --packages-select <pkg>)",
        _rebuild_package,
    ),
    "reset_serial": ActionSpec(
        "reset_serial",
        True,
        "Reset serial/USB port mappings (docker/navigation/setup-USB.sh)",
        _reset_serial,
    ),
    "restart_dds": ActionSpec(
        "restart_dds",
        False,
        "Reapply CycloneDDS host config and kernel buffers (setup_cyclonedds.sh)",
        _restart_dds,
    ),
    "none": ActionSpec(
        "none",
        False,
        "No automatic action; report to a human",
        lambda _a: [],
        requires_confirmation=False,
    ),
}

# Valid action names exposed to the oracle prompt (closed set).
VALID_ACTIONS = tuple(ACTION_SPECS.keys())


@dataclass
class CorrectiveAction:
    kind: str
    arg: str
    command: list[str] = field(default_factory=list)
    description: str = ""

    @property
    def requires_confirmation(self) -> bool:
        spec = ACTION_SPECS.get(self.kind)
        return spec.requires_confirmation if spec else True

    def pretty(self) -> str:
        cmd = " ".join(self.command) if self.command else "(none)"
        return f"{self.kind}({self.arg}) → {cmd}"


@dataclass
class ActionResult:
    ok: bool
    returncode: int = 0
    output: str = ""
    dry_run: bool = False


def parse_action(accion_sugerida: str) -> tuple[str, str]:
    raw = (accion_sugerida or "").strip()
    kind, _, arg = raw.partition(":")
    return kind.strip(), arg.strip()


def resolve(accion_sugerida: str) -> CorrectiveAction | None:
    """Turn a validated `accion_sugerida` string into an executable action."""
    kind, arg = parse_action(accion_sugerida)
    spec = ACTION_SPECS.get(kind)
    if spec is None:
        return None
    if spec.needs_arg and not arg:
        return None
    command = spec.builder(arg)
    return CorrectiveAction(kind=kind, arg=arg, command=command, description=spec.description)


def execute(action: CorrectiveAction, dry_run: bool = False, timeout: int = 600) -> ActionResult:
    """Run the corrective command from the repo root. `none` is a no-op."""
    if action.kind == "none" or not action.command:
        return ActionResult(ok=True, output="no-op", dry_run=dry_run)
    if dry_run:
        return ActionResult(ok=True, output="dry-run: " + " ".join(action.command), dry_run=True)
    try:
        proc = subprocess.run(
            action.command,
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return ActionResult(
            ok=proc.returncode == 0,
            returncode=proc.returncode,
            output=(proc.stdout or "") + (proc.stderr or ""),
        )
    except (FileNotFoundError, subprocess.SubprocessError) as e:
        return ActionResult(ok=False, returncode=-1, output=str(e))
