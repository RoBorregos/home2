"""rclpy-based introspection: nodes, topics, orphan topics, and Hz of critical topics.

One probe node lives for the whole dashboard session (see RosProbe); Hz subscriptions
are created per snapshot and destroyed afterwards, so nothing streams between refreshes."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

INTERNAL_TOPICS = {"/parameter_events", "/rosout"}
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
            "Source ROS 2 first: source /opt/ros/jazzy/setup.bash"
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
    services: list[str] = field(default_factory=list)
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


def _spin_for(node, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while (remaining := deadline - time.monotonic()) > 0:
        _rclpy.spin_once(node, timeout_sec=min(0.05, remaining))


def _measure_hz(
    node, topics_with_types: dict[str, str], window: float
) -> dict[str, float]:
    """Subscribe to each topic with a stamping callback; spin for window seconds.

    Hz comes from the first/last message stamps, so the time a fresh subscription
    takes to match its publisher doesn't drag the rate down."""
    stamps: dict[str, list[float]] = {t: [] for t in topics_with_types}
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
            stamps[_t].append(time.monotonic())

        try:
            sub = node.create_subscription(msg_type, topic, _cb, 10)
            subs.append(sub)
        except Exception:
            continue

    _spin_for(node, window)

    for sub in subs:
        try:
            node.destroy_subscription(sub)
        except Exception:
            pass

    hz: dict[str, float] = {}
    for topic, ts in stamps.items():
        if len(ts) >= 2 and ts[-1] > ts[0]:
            hz[topic] = (len(ts) - 1) / (ts[-1] - ts[0])
        else:
            hz[topic] = len(ts) / window
    return hz


class RosProbe:
    """Long-lived probe node. Keeping the same DDS participant across refreshes lets
    discovery finish once; a node created and queried right away sees an empty or
    partial graph."""

    def __init__(self, warmup: float = 1.5):
        self._node = None
        self._initialized_here = False
        self._error = ""
        try:
            _ensure_rclpy()
        except RuntimeError as e:
            self._error = str(e)
            return
        try:
            if not _rclpy.ok():
                _rclpy.init()
                self._initialized_here = True
            self._node = _rclpy.create_node("frida_status_probe")
            _spin_for(self._node, warmup)
        except Exception as e:
            self._error = f"{type(e).__name__}: {e}"

    def snapshot(self, hz_window: float = 1.0) -> RosSnapshot:
        if self._node is None:
            return RosSnapshot(rclpy_available=_rclpy is not None, error=self._error)

        node = self._node
        snap = RosSnapshot()
        try:
            snap.nodes = sorted(
                f"{ns.rstrip('/')}/{n}" if ns and ns != "/" else f"/{n}"
                for n, ns in node.get_node_names_and_namespaces()
                if n != "frida_status_probe"
            )

            critical = _load_critical_topics()
            hz_targets: dict[str, str] = {}

            try:
                snap.services = sorted(s for s, _ in node.get_service_names_and_types())
            except Exception:
                snap.services = []

            for tname, ttypes in node.get_topic_names_and_types():
                pubc = node.count_publishers(tname)
                subc = node.count_subscribers(tname)
                snap.topics[tname] = TopicInfo(tname, ttypes, pubc, subc)
                if tname in INTERNAL_TOPICS:
                    continue
                if pubc == 0 and subc > 0:
                    snap.orphans_no_pub.append(tname)
                elif subc == 0 and pubc > 0:
                    snap.orphans_no_sub.append(tname)
                if tname in critical and ttypes:
                    hz_targets[tname] = ttypes[0]

            for t in critical:
                snap.hz.setdefault(t, 0.0)
            if hz_targets:
                snap.hz.update(_measure_hz(node, hz_targets, hz_window))

        except Exception as e:
            snap.error = f"{type(e).__name__}: {e}"

        return snap

    def close(self) -> None:
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        if self._initialized_here:
            try:
                _rclpy.shutdown()
            except Exception:
                pass
            self._initialized_here = False
