"""Structural regex parser: isolate known-critical errors from noisy ROS 2 logs.

Filters the usual INFO/DEBUG chatter and surfaces only the failure classes that
map to a corrective action:

    segfault  — a core node process crashed (SIGSEGV / exit code -11 / died)
    dds       — RMW / DDS discovery or service-timeout failures
    tf2       — transform lookup failures in the tf2 stack
    build     — compile / import failures (colcon, CMake, Python import)

Consumes the raw text produced by log_collector.LogBundle.text and returns a
de-duplicated list of ParsedError, most-recent-first."""

from __future__ import annotations

import re
from dataclasses import dataclass

# node name → area, by substring. Best-effort; used to route diagnosis/actions.
_AREA_HINTS = {
    "navigation": (
        "nav",
        "bt_navigator",
        "controller_server",
        "planner",
        "amcl",
        "rtabmap",
        "slam",
        "dashgo",
        "lidar",
        "map_server",
    ),
    "manipulation": (
        "manip",
        "pick",
        "place",
        "pour",
        "xarm",
        "moveit",
        "motion_planning",
        "grasp",
        "gpd",
    ),
    "vision": (
        "vision",
        "zed",
        "moondream",
        "detector",
        "tracker",
        "face_recognition",
        "perception_3d",
    ),
    "hri": ("hri", "speech", "stt", "tts", "nlp", "llm", "rag", "embeddings"),
}

# (kind, compiled pattern). Order matters: first match wins per line.
_PATTERNS: list[tuple[str, re.Pattern]] = [
    (
        "segfault",
        re.compile(
            r"process has died|exit code -11|signal SIGSEGV|core dumped|"
            r"Segmentation fault",
            re.IGNORECASE,
        ),
    ),
    (
        "dds",
        re.compile(
            r"RMW_UXR_CE_NODE_NOT_FOUND|rmw_cyclonedds|failed to create|"
            r"discovery|Service .* not available|Action .* not available|"
            r"wait_for_service .* timed out|client .* timed out",
            re.IGNORECASE,
        ),
    ),
    (
        "tf2",
        re.compile(
            r"LookupException|ExtrapolationException|ConnectivityException|"
            r"frame .* does not exist|Could not transform|canTransform",
            re.IGNORECASE,
        ),
    ),
    (
        "build",
        re.compile(
            r"CMake Error|colcon build .* failed|ModuleNotFoundError|"
            r"ImportError|undefined symbol|No module named",
            re.IGNORECASE,
        ),
    ),
]

# lines we never care about even if a pattern grazes them
_NOISE = re.compile(r"\[DEBUG\]|deprecation|pkg_resources", re.IGNORECASE)

# extract a node name from common ROS log framings, e.g.
#   [component_container-3] ... [bt_navigator]: ...
#   [gpsr_commands]: ...
_NODE_RE = re.compile(r"\[([A-Za-z_][\w/]*)(?:-\d+)?\]")


@dataclass(frozen=True)
class ParsedError:
    kind: str  # segfault | dds | tf2 | build
    node: str  # best-effort node name, or ""
    area: str  # navigation | manipulation | vision | hri | ""
    raw_line: str


def _guess_node(line: str) -> str:
    matches = _NODE_RE.findall(line)
    # skip generic launcher tokens; prefer the most specific (last) match
    ignore = {"INFO", "WARN", "ERROR", "FATAL", "DEBUG"}
    named = [m for m in matches if m not in ignore]
    return named[-1] if named else ""


def _guess_area(node: str, line: str) -> str:
    haystack = f"{node} {line}".lower()
    for area, hints in _AREA_HINTS.items():
        if any(h in haystack for h in hints):
            return area
    return ""


def parse(text: str) -> list[ParsedError]:
    seen: set[tuple[str, str, str]] = set()
    out: list[ParsedError] = []
    for line in text.splitlines():
        line = line.strip()
        if not line or _NOISE.search(line):
            continue
        for kind, pattern in _PATTERNS:
            if pattern.search(line):
                node = _guess_node(line)
                area = _guess_area(node, line)
                key = (kind, node, area)
                if key in seen:
                    break
                seen.add(key)
                out.append(ParsedError(kind, node, area, line[:200]))
                break
    # most recent lines are last in the log; surface them first
    out.reverse()
    return out
