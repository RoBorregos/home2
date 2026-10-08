"""Reads the bash *_nodes.cfg / *_infra.cfg files by sourcing them, so the bash
configs stay the single source of truth for status.sh and the dashboard."""

from __future__ import annotations

import subprocess
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parent / "configs"


def _read_key(cfg_name: str, var: str, key: str) -> list[str]:
    cfg = CONFIG_DIR / cfg_name
    if not cfg.is_file():
        return []
    result = subprocess.run(
        ["bash", "-c", f'source "{cfg}" && echo "${{{var}[{key}]}}"'],
        capture_output=True,
        text=True,
        timeout=2,
    )
    return result.stdout.split() if result.returncode == 0 else []


def load_area_nodes(area: str, task: str) -> list[str]:
    return _read_key(f"{area}_nodes.cfg", f"{area.upper()}_NODES", task)


def load_area_containers(area: str) -> list[str]:
    return _read_key(f"{area}_infra.cfg", f"{area.upper()}_INFRA", "containers")
