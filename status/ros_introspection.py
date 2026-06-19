"""rclpy-based introspection: nodes, topics, orphan topics, and Hz of critical topics.

We create a short-lived probe node per snapshot. Cost ~150ms but state is always fresh
and there are no stale subscription side effects."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

CRITICAL_TOPICS_FILE = (
    Path(__file__).resolve().parent / "configs" / "critical_topics.yaml"
)

# Internal modules of ROS 2 are not loaded at import time so the dashboard can
# print a clean error if ROS is not sourced.
_rclpy = None
_get_message = None


def _ensure_rclpy():
    global _rclpy, _get_message
    if _rclpy is not None:
        return
    try:
        import rclpy  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as e:
        raise RuntimeError(
            "rclpy/rosidl_runtime_py not found. "
            "Source ROS 2 first: source /opt/ros/humble/setup.bash"
        ) from e
    _rclpy = rclpy
    _get_message = get_message


@dataclass
class TopicInfo:
    name: str
    types: list[str]
    pub_count: int
    sub_count: int


@dataclass
class RosSnapshot:
    nodes: list[str] = field(default_factory=list)
    topics: dict[str, TopicInfo] = field(default_factory=dict)
    orphans_no_pub: list[str] = field(default_factory=list)  # subs but no pubs
    orphans_no_sub: list[str] = field(default_factory=list)  # pubs but no subs
    hz: dict[str, float] = field(default_factory=dict)
    rclpy_available: bool = True
    error: str = ""


def _load_critical_topics() -> list[str]:
    if not CRITICAL_TOPICS_FILE.is_file():
        return []
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(CRITICAL_TOPICS_FILE.read_text()) or []
        return [t for t in data if isinstance(t, str)]
    except Exception:
        return []


def _measure_hz(
    node, topics_with_types: dict[str, str], window: float
) -> dict[str, float]:
    """Subscribe to each topic with a counter callback; spin for window seconds."""
    counts: dict[str, int] = {t: 0 for t in topics_with_types}
    subs = []
    for topic, type_str in topics_with_types.items():
        try:
            msg_pkg, _, msg_name = type_str.partition("/msg/")
            if not msg_name:
                # support legacy "pkg/Type" form
                msg_pkg, _, msg_name = type_str.partition("/")
            msg_type = _get_message(f"{msg_pkg}/msg/{msg_name}")
        except Exception:
            continue

        def _cb(_msg, _t=topic):
            counts[_t] += 1

        try:
            sub = node.create_subscription(msg_type, topic, _cb, 10)
            subs.append(sub)
        except Exception:
            continue

    deadline = time.monotonic() + window
    while time.monotonic() < deadline:
        _rclpy.spin_once(node, timeout_sec=min(0.05, deadline - time.monotonic()))

    for sub in subs:
        try:
            node.destroy_subscription(sub)
        except Exception:
            pass

    return {t: counts[t] / window for t in topics_with_types}


def snapshot(hz_window: float = 1.0) -> RosSnapshot:
    try:
        _ensure_rclpy()
    except RuntimeError as e:
        return RosSnapshot(rclpy_available=False, error=str(e))

    snap = RosSnapshot()
    initialized_here = False
    node = None
    try:
        if not _rclpy.ok():
            _rclpy.init()
            initialized_here = True
        node = _rclpy.create_node("frida_status_probe")

        snap.nodes = sorted(
            f"{ns.rstrip('/')}/{n}" if ns and ns != "/" else f"/{n}"
            for n, ns in node.get_node_names_and_namespaces()
        )

        critical = _load_critical_topics()
        hz_targets: dict[str, str] = {}

        for tname, ttypes in node.get_topic_names_and_types():
            pubc = node.count_publishers(tname)
            subc = node.count_subscribers(tname)
            snap.topics[tname] = TopicInfo(tname, ttypes, pubc, subc)
            if pubc == 0 and subc > 0:
                snap.orphans_no_pub.append(tname)
            elif subc == 0 and pubc > 0:
                snap.orphans_no_sub.append(tname)
            if tname in critical and ttypes:
                hz_targets[tname] = ttypes[0]

        for t in critical:
            snap.hz.setdefault(t, 0.0)
        if hz_targets:
            measured = _measure_hz(node, hz_targets, hz_window)
            snap.hz.update(measured)

    except Exception as e:
        snap.error = f"{type(e).__name__}: {e}"
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if initialized_here:
            try:
                _rclpy.shutdown()
            except Exception:
                pass

    return snap
