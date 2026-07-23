"""Collect log context from the running system for diagnosis.

Two sources:
  1. `docker logs --tail N <container>` — same mechanism dashboard._fetch_logs
     uses, but returns raw text for the parser instead of colorized Text.
  2. The most recent ROS 2 launch log under ~/.ros/log/ (the `latest` symlink),
     concatenating the tail of every *.log file in it.

Kept dependency-free (stdlib only) so it runs inside the integration container
without extra installs."""

from __future__ import annotations

import subprocess
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_CONTAINER = "home2-integration"
ROS_LOG_DIR = Path.home() / ".ros" / "log"


@dataclass
class LogBundle:
    docker_logs: str = ""
    ros_logs: str = ""
    sources: list[str] = field(default_factory=list)

    @property
    def text(self) -> str:
        """All collected lines as one blob for the parser."""
        return "\n".join(t for t in (self.docker_logs, self.ros_logs) if t)


def _docker_logs(container: str, lines: int) -> str:
    try:
        result = subprocess.run(
            ["docker", "logs", "--tail", str(lines), container],
            capture_output=True,
            text=True,
            timeout=3,
        )
        if result.returncode != 0:
            return ""
        # docker interleaves stdout+stderr across the two streams
        return ((result.stdout or "") + (result.stderr or "")).strip()
    except (FileNotFoundError, subprocess.SubprocessError):
        return ""


def _latest_ros_log_dir() -> Path | None:
    latest = ROS_LOG_DIR / "latest"
    if latest.exists():
        return latest.resolve()
    if not ROS_LOG_DIR.is_dir():
        return None
    # Fall back to the most recently modified run directory.
    runs = [p for p in ROS_LOG_DIR.iterdir() if p.is_dir()]
    if not runs:
        return None
    return max(runs, key=lambda p: p.stat().st_mtime)


def _ros_logs(tail_lines: int) -> tuple[str, str]:
    run_dir = _latest_ros_log_dir()
    if run_dir is None:
        return "", ""
    chunks: list[str] = []
    for log_file in sorted(run_dir.glob("*.log")):
        try:
            lines = log_file.read_text(errors="replace").splitlines()
        except OSError:
            continue
        tail = lines[-tail_lines:]
        if tail:
            chunks.append(f"# {log_file.name}\n" + "\n".join(tail))
    return "\n".join(chunks), str(run_dir)


def collect(
    container: str = DEFAULT_CONTAINER,
    docker_lines: int = 50,
    ros_log_lines: int = 50,
) -> LogBundle:
    bundle = LogBundle()
    docker = _docker_logs(container, docker_lines)
    if docker:
        bundle.docker_logs = docker
        bundle.sources.append(f"docker:{container}")

    ros_text, run_dir = _ros_logs(ros_log_lines)
    if ros_text:
        bundle.ros_logs = ros_text
        bundle.sources.append(f"ros:{run_dir}")

    return bundle
