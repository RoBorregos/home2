"""Helper to read the bash *_nodes.cfg / *_infra.cfg files without re-implementing
the parser. We just shell out to bash to source the file and echo the desired key.
That way the bash configs remain the single source of truth."""

from __future__ import annotations

import subprocess
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parent / "configs"


def _eval_bash(snippet: str) -> str:
    result = subprocess.run(
        ["bash", "-c", snippet],
        capture_output=True,
        text=True,
        timeout=2,
    )
    if result.returncode != 0:
        return ""
    return result.stdout.strip()


def load_area_nodes(area: str, task: str) -> list[str]:
    cfg = CONFIG_DIR / f"{area}_nodes.cfg"
    if not cfg.is_file():
        return []
    var = f"{area.upper()}_NODES"
    out = _eval_bash(f'source "{cfg}" && echo "${{{var}[{task}]}}"')
    return out.split() if out else []


def load_area_containers(area: str) -> list[str]:
    cfg = CONFIG_DIR / f"{area}_infra.cfg"
    if not cfg.is_file():
        return []
    var = f"{area.upper()}_INFRA"
    out = _eval_bash(f'source "{cfg}" && echo "${{{var}[containers]}}"')
    return out.split() if out else []


def area_requires_shm(area: str) -> bool:
    cfg = CONFIG_DIR / f"{area}_infra.cfg"
    if not cfg.is_file():
        return False
    var = f"{area.upper()}_INFRA"
    out = _eval_bash(f'source "{cfg}" && echo "${{{var}[requires_shm]}}"')
    return out.lower() == "true"
