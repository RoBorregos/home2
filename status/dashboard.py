"""Live FRIDA status dashboard.

Usage:
    python3 -m status.dashboard [area] [task]
    python3 -m status.dashboard --gpsr           # all areas, one task
    python3 -m status.dashboard vision --gpsr    # single area

Layout: 6 panels + footer.
    Row 1: Host DDS        | Integration Logs | Live signals
    Row 2: Containers      | ROS Activity     | Orphan topics
    Footer: Hints (correlates infra failures with missing nodes)

Defaults: refreshes every 2s. Ctrl+C to exit cleanly."""

from __future__ import annotations

import subprocess
import sys
import time
from collections import Counter
from datetime import datetime

from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from status import area_config, infra_checks, ros_introspection

ALL_AREAS = ("vision", "manipulation", "navigation", "hri", "integration")
REMOTE_AREAS = {"hri"}  # mirrors lib.sh ORIN_SERVER_AREAS — checked on remote Orin
REFRESH_SECONDS = 2.0
HZ_WINDOW = 0.8
LOGS_CONTAINER = "home2-integration"
LOGS_TAIL_LINES = 8
LOG_LINE_MAX = 140
ACTIVITY_MAX_ROWS = 12
ORPHAN_RED_MAX = 5
ORPHAN_YELLOW_MAX = 3


def _parse_args(argv: list[str]) -> tuple[list[str], str]:
    """Mirror scripts/status.sh: [area] [task] OR [task]."""
    args = [a for a in argv[1:] if a]
    if not args:
        return list(ALL_AREAS), ""
    if len(args) == 1:
        # single positional → task name; show all areas
        return list(ALL_AREAS), args[0]
    return [args[0]], args[1]


def _icon(ok: bool) -> Text:
    return Text("✓", style="bold green") if ok else Text("⨯", style="bold red")


# ── Panel 1: Host DDS ────────────────────────────────────────────────
def _render_dds_panel(infra: infra_checks.InfraSnapshot) -> Panel:
    dds = infra.dds
    t = Table.grid(padding=(0, 1))
    t.add_column()
    t.add_column()
    rows = [
        (dds.cyclone_xml, "/etc/cyclonedds.xml"),
        (dds.sysctl_conf, "sysctl buffers"),
        (dds.rmem_max >= infra_checks.EXPECTED_RMEM_MAX, f"rmem_max = {dds.rmem_max}"),
        (dds.rmw_impl == "rmw_cyclonedds_cpp", f"RMW = {dds.rmw_impl or 'unset'}"),
    ]
    for ok, label in rows:
        t.add_row(_icon(ok), label)

    if dds.cyclone_iface:
        t.add_row(Text("•", style="cyan"), f"iface = {dds.cyclone_iface}")
    if dds.iceoryx_roudi_status:
        t.add_row(
            _icon(dds.iceoryx_roudi_status == "running"),
            f"home2-roudi ({dds.iceoryx_roudi_status})",
        )
    style = "green" if dds.ok else "red"
    return Panel(t, title="Host DDS", border_style=style)


# ── Panel 2: Integration logs ────────────────────────────────────────
def _fetch_logs(container: str, lines: int) -> str:
    try:
        result = subprocess.run(
            ["docker", "logs", "--tail", str(lines), container],
            capture_output=True,
            text=True,
            timeout=2,
        )
        if result.returncode != 0:
            return f"(container '{container}' not running)"
        # docker writes both stdout+stderr; combine
        out = (result.stdout or "") + (result.stderr or "")
        return out.strip() or "(no recent output)"
    except (FileNotFoundError, subprocess.SubprocessError) as e:
        return f"(error reading logs: {e})"


def _colorize_log_line(line: str) -> Text:
    if len(line) > LOG_LINE_MAX:
        line = line[: LOG_LINE_MAX - 1] + "…"
    upper = line.upper()
    if "ERROR" in upper or "FATAL" in upper or "TRACEBACK" in upper:
        style = "red"
    elif "WARN" in upper:
        style = "yellow"
    elif "INFO" in upper:
        style = "dim cyan"
    else:
        style = "dim"
    return Text(line, style=style, no_wrap=True, overflow="ellipsis")


def _render_logs_panel() -> Panel:
    raw = _fetch_logs(LOGS_CONTAINER, LOGS_TAIL_LINES)
    lines = [_colorize_log_line(line) for line in raw.splitlines()[-LOGS_TAIL_LINES:]]
    if not lines:
        body: Group | Text = Text("(empty)", style="dim")
    else:
        body = Group(*lines)
    return Panel(body, title=f"Logs · {LOGS_CONTAINER}", border_style="cyan")


# ── Panel 3: Live signals (Hz) ───────────────────────────────────────
def _shorten(name: str, max_len: int = 30) -> str:
    if len(name) <= max_len:
        return name
    # keep the tail (more informative for ROS names) with leading ellipsis
    return "…" + name[-(max_len - 1) :]


def _render_signals_panel(ros: ros_introspection.RosSnapshot) -> Panel:
    t = Table(show_header=True, header_style="bold cyan", box=None, expand=True)
    t.add_column("Topic", no_wrap=True, overflow="ellipsis", ratio=3)
    t.add_column("Hz", justify="right", no_wrap=True, width=6)
    t.add_column("", no_wrap=True, width=2)
    if not ros.hz:
        t.add_row("(critical_topics.yaml empty)", "", "")
    else:
        for topic, hz in ros.hz.items():
            ok = hz > 0.5
            t.add_row(_shorten(topic, 28), f"{hz:5.1f}", _icon(ok))
    return Panel(t, title="Live signals", border_style="cyan")


# ── Panel 4: Containers ──────────────────────────────────────────────
def _render_containers_panel(
    infra: infra_checks.InfraSnapshot, area_states: dict[str, dict]
) -> Panel:
    t = Table(box=None, show_header=False, expand=True, padding=(0, 1))
    t.add_column(no_wrap=True, width=2)
    t.add_column(no_wrap=True, overflow="ellipsis", ratio=2)
    t.add_column(no_wrap=True, overflow="ellipsis", ratio=3, style="dim")
    if not infra.containers:
        t.add_row(Text("•", style="dim"), "(none declared)", "")
    else:
        for name, cs in sorted(infra.containers.items()):
            t.add_row(_icon(cs.ok), name, cs.detail)

    for area in REMOTE_AREAS:
        if area in area_states:
            t.add_row(
                Text("⇄", style="magenta"),
                f"home2-{area}",
                Text("on remote Orin", style="magenta"),
            )

    any_bad = any(not c.ok for c in infra.containers.values())
    return Panel(t, title="Containers", border_style="red" if any_bad else "green")


# ── Panel 5: ROS Activity (grouped) ─────────────────────────────────
def _group_by_namespace(items: list[str]) -> dict[str, int]:
    """Group /<ns>/whatever → ns. Items with no namespace bucket under '(root)'."""
    counts: Counter[str] = Counter()
    for item in items:
        parts = item.lstrip("/").split("/", 1)
        ns = parts[0] if parts and parts[0] else "(root)"
        # Skip ROS internal namespaces that pollute the view
        if ns in {"parameter_events", "rosout"}:
            ns = "(ros2 internal)"
        counts[ns] += 1
    return dict(counts.most_common())


def _render_activity_panel(
    ros: ros_introspection.RosSnapshot, area_states: dict[str, dict], task: str
) -> Panel:
    # Filter ROS noise + our own probe node so the panel reflects user workload.
    visible_nodes = [
        n
        for n in ros.nodes
        if not n.endswith("/frida_status_probe") and not n.startswith("/launch_ros_")
    ]
    visible_topics = [
        t
        for t in ros.topics
        if not t.startswith("/parameter_events") and not t.startswith("/rosout")
    ]
    visible_services = [
        s
        for s in ros.services
        if "frida_status_probe" not in s and not s.startswith("/launch_ros_")
    ]

    node_groups = _group_by_namespace(visible_nodes)
    topic_groups = _group_by_namespace(visible_topics)
    svc_groups = _group_by_namespace(visible_services)

    all_ns = set(node_groups) | set(topic_groups) | set(svc_groups)
    # Sort by total activity (most relevant first), then alphabetically.
    ranked = sorted(
        all_ns,
        key=lambda ns: (
            -(node_groups.get(ns, 0) + topic_groups.get(ns, 0) + svc_groups.get(ns, 0)),
            ns,
        ),
    )
    ranked = ranked[:ACTIVITY_MAX_ROWS]

    t = Table(show_header=True, header_style="bold cyan", box=None, expand=True)
    t.add_column("Namespace", no_wrap=True, overflow="ellipsis", ratio=3)
    t.add_column("N", justify="right", no_wrap=True, width=4)
    t.add_column("T", justify="right", no_wrap=True, width=4)
    t.add_column("S", justify="right", no_wrap=True, width=4)
    if task:
        t.add_column("Exp.", justify="right", no_wrap=True, width=6)
        t.add_column("", no_wrap=True, width=2)

    for ns in ranked:
        n = node_groups.get(ns, 0)
        tp = topic_groups.get(ns, 0)
        sv = svc_groups.get(ns, 0)
        row = [ns, str(n) if n else "·", str(tp) if tp else "·", str(sv) if sv else "·"]
        if task:
            st = area_states.get(ns)
            if st and st["expected"]:
                ratio = f"{len(st['running'])}/{len(st['expected'])}"
                icon = _icon(not st["missing"])
                row += [ratio, icon]
            else:
                row += ["", ""]
        t.add_row(*row)

    omitted = len(all_ns) - len(ranked)
    totals = (
        f"Σ N={len(visible_nodes)} T={len(visible_topics)} S={len(visible_services)}"
        + (f" · +{omitted} ns hidden" if omitted > 0 else "")
        + (f" · task={task}" if task else " · (no task)")
    )
    return Panel(
        Group(t, Text(totals, style="dim italic", no_wrap=True, overflow="ellipsis")),
        title="ROS Activity",
        border_style="cyan",
    )


# ── Panel 6: Orphan topics ──────────────────────────────────────────
def _render_orphans_panel(ros: ros_introspection.RosSnapshot) -> Panel:
    t = Table(box=None, show_header=False, expand=True, padding=(0, 1))
    t.add_column(no_wrap=True, width=2)
    t.add_column(no_wrap=True, overflow="ellipsis", ratio=1)
    rows_added = 0
    for top in ros.orphans_no_pub[:ORPHAN_RED_MAX]:
        info = ros.topics.get(top)
        sub_count = info.sub_count if info else "?"
        t.add_row(_icon(False), f"{_shorten(top, 32)}  s={sub_count} p=0")
        rows_added += 1
    for top in ros.orphans_no_sub[:ORPHAN_YELLOW_MAX]:
        info = ros.topics.get(top)
        pub_count = info.pub_count if info else "?"
        t.add_row(
            Text("•", style="yellow"),
            f"{_shorten(top, 32)}  p={pub_count} s=0",
        )
        rows_added += 1
    if rows_added == 0:
        t.add_row(Text("✓", style="green"), "no orphan topics")
    extra_red = max(0, len(ros.orphans_no_pub) - ORPHAN_RED_MAX)
    extra_yellow = max(0, len(ros.orphans_no_sub) - ORPHAN_YELLOW_MAX)
    if extra_red or extra_yellow:
        t.add_row(
            Text("…", style="dim"),
            Text(f"+{extra_red} red · +{extra_yellow} yellow", style="dim"),
        )
    return Panel(t, title="Orphan topics", border_style="yellow")


# ── Footer: Hints ───────────────────────────────────────────────────
def _hint(msg: str, style: str) -> Text:
    return Text(msg, style=style, no_wrap=True, overflow="ellipsis")


def _render_hints(
    infra: infra_checks.InfraSnapshot,
    area_states: dict[str, dict],
    ros: ros_introspection.RosSnapshot,
) -> Panel:
    lines: list[Text] = []
    if not infra.dds.ok:
        lines.append(
            _hint(
                " ⨯ DDS host config incomplete → sudo bash scripts/setup_cyclonedds.sh",
                "red",
            )
        )
    for area, st in area_states.items():
        if area in REMOTE_AREAS:
            continue
        down = [
            c
            for c, cs in infra.containers.items()
            if c.endswith(f"-{area}") and not cs.ok
        ]
        if down and st["missing"]:
            lines.append(
                _hint(
                    f" ⨯ {area}: containers down + nodes missing → ./run.sh {area} --recreate",
                    "red",
                )
            )
    if not ros.rclpy_available:
        lines.append(_hint(f" ⨯ {ros.error}", "red"))
    elif ros.error:
        lines.append(_hint(f" ⨯ rclpy: {ros.error}", "red"))
    for top in ros.orphans_no_pub[:2]:
        lines.append(_hint(f" ⨯ {_shorten(top, 60)}: subs>0 but pubs=0", "red"))
    if not lines:
        lines.append(_hint(" ✓ everything green", "green"))
    return Panel(Group(*lines[:4]), title="Hints", border_style="magenta")


# ── Layout ──────────────────────────────────────────────────────────
def _build_layout() -> Layout:
    root = Layout()
    root.split_column(
        Layout(name="header", size=3),
        Layout(name="row1", ratio=1),
        Layout(name="row2", ratio=1),
        Layout(name="footer", size=6),
    )
    root["row1"].split_row(
        Layout(name="dds", ratio=1),
        Layout(name="logs", ratio=2),
        Layout(name="signals", ratio=1),
    )
    root["row2"].split_row(
        Layout(name="containers", ratio=1),
        Layout(name="activity", ratio=2),
        Layout(name="orphans", ratio=1),
    )
    return root


def _header(env_type: str, iface: str, task: str, areas: list[str]) -> Panel:
    ts = datetime.now().strftime("%H:%M:%S")
    area_label = areas[0] if len(areas) == 1 else "all"
    task_label = task or "(none)"
    msg = Text(
        f"FRIDA STATUS · env={env_type} · iface={iface or 'autodetermine'} "
        f"· area={area_label} · task={task_label} · {ts} · refresh {REFRESH_SECONDS:.0f}s",
        style="bold white on blue",
    )
    return Panel(msg, border_style="blue")


def _compute_area_states(
    areas: list[str], task: str, running_nodes: set[str]
) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for area in areas:
        expected = area_config.load_area_nodes(area, task) if task else []
        running = [n for n in expected if n in running_nodes]
        missing = [n for n in expected if n not in running_nodes]
        out[area] = {"expected": expected, "running": running, "missing": missing}
    return out


def _collect_containers(areas: list[str]) -> list[str]:
    names: list[str] = []
    for area in areas:
        if area in REMOTE_AREAS:
            continue  # skip — checked on a different host
        names.extend(area_config.load_area_containers(area))
    if any(area_config.area_requires_shm(a) for a in areas):
        if "home2-roudi" not in names:
            names.append("home2-roudi")
    seen = set()
    return [n for n in names if not (n in seen or seen.add(n))]


def main() -> int:
    areas, task = _parse_args(sys.argv)
    console = Console()
    layout = _build_layout()

    container_names = _collect_containers(areas)

    try:
        with Live(layout, console=console, refresh_per_second=2, screen=True):
            while True:
                infra_snap = infra_checks.snapshot(container_names)
                ros_snap = ros_introspection.snapshot(hz_window=HZ_WINDOW)
                running_nodes = set(ros_snap.nodes)
                area_states = _compute_area_states(areas, task, running_nodes)

                layout["header"].update(
                    _header(
                        infra_snap.env_type, infra_snap.dds.cyclone_iface, task, areas
                    )
                )
                layout["dds"].update(_render_dds_panel(infra_snap))
                layout["logs"].update(_render_logs_panel())
                layout["signals"].update(_render_signals_panel(ros_snap))
                layout["containers"].update(
                    _render_containers_panel(infra_snap, area_states)
                )
                layout["activity"].update(
                    _render_activity_panel(ros_snap, area_states, task)
                )
                layout["orphans"].update(_render_orphans_panel(ros_snap))
                layout["footer"].update(
                    _render_hints(infra_snap, area_states, ros_snap)
                )

                time.sleep(max(0.0, REFRESH_SECONDS - HZ_WINDOW))
    except KeyboardInterrupt:
        console.print("[dim]dashboard stopped[/dim]")
        return 0


if __name__ == "__main__":
    sys.exit(main())
