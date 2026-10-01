"""Live FRIDA status dashboard.

Usage:
    python3 -m status.dashboard --hric           # all areas for a task
    python3 -m status.dashboard vision --hric    # single area
Extra run.sh flags (--build, --recreate, ...) are ignored. Ctrl+C to exit.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from datetime import datetime

from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from status import area_config, infra_checks, ros_introspection

ALL_AREAS = ("vision", "manipulation", "navigation", "integration", "hri", "display")
TASKS = ("--hric", "--ppc", "--gpsr", "--dlc", "--restaurant", "--finals", "--safety")
# Areas whose containers run on the second Orin (lib.sh ORIN_SERVER_AREAS).
REMOTE_AREAS = set(os.environ.get("ORIN_SERVER_AREAS", "hri").split())
REFRESH_SECONDS = 2.0
HZ_WINDOW = 0.8
LOGS_CONTAINER = "home2-integration"
LOGS_TAIL_LINES = 8
LOG_LINE_MAX = 140
ORPHAN_RED_MAX = 5
ORPHAN_YELLOW_MAX = 3


def _parse_args(argv: list[str]) -> tuple[list[str], str]:
    areas = [a for a in argv if a in ALL_AREAS] or list(ALL_AREAS)
    task = next((a for a in argv if a in TASKS), "")
    return areas, task


def _icon(ok: bool) -> Text:
    return Text("✓", style="bold green") if ok else Text("⨯", style="bold red")


def _shorten(name: str, max_len: int = 30) -> str:
    return name if len(name) <= max_len else "…" + name[-(max_len - 1) :]


def _render_dds_panel(infra: infra_checks.InfraSnapshot) -> Panel:
    dds = infra.dds
    t = Table.grid(padding=(0, 1))
    t.add_column()
    t.add_column()
    rows = [
        (dds.cyclone_xml, "/etc/cyclonedds.xml"),
        (dds.sysctl_conf, "sysctl buffers"),
        (dds.rmem_max >= infra_checks.EXPECTED_RMEM_MAX, f"rmem_max = {dds.rmem_max}"),
    ]
    for ok, label in rows:
        t.add_row(_icon(ok), label)
    t.add_row(Text("•", style="cyan"), f"RMW = {dds.rmw_impl or 'default'}")
    if dds.cyclone_iface:
        t.add_row(Text("•", style="cyan"), f"iface = {dds.cyclone_iface}")
    if dds.iceoryx_roudi_status:
        t.add_row(
            _icon(dds.iceoryx_roudi_status == "running"),
            f"home2-roudi ({dds.iceoryx_roudi_status})",
        )
    return Panel(t, title="Host DDS", border_style="green" if dds.ok else "red")


def _fetch_logs(container: str, lines: int) -> str:
    try:
        result = subprocess.run(
            ["docker", "logs", "--tail", str(lines), container],
            capture_output=True,
            text=True,
            timeout=2,
        )
    except (FileNotFoundError, subprocess.SubprocessError) as e:
        return f"(error reading logs: {e})"
    if result.returncode != 0:
        return f"(container '{container}' not running)"
    return (
        (result.stdout or "") + (result.stderr or "")
    ).strip() or "(no recent output)"


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
    body = Group(*lines) if lines else Text("(empty)", style="dim")
    return Panel(body, title=f"Logs · {LOGS_CONTAINER}", border_style="cyan")


def _render_signals_panel(ros: ros_introspection.RosSnapshot) -> Panel:
    t = Table(show_header=True, header_style="bold cyan", box=None, expand=True)
    t.add_column("Topic", no_wrap=True, overflow="ellipsis", ratio=3)
    t.add_column("Hz", justify="right", no_wrap=True, width=6)
    t.add_column("", no_wrap=True, width=2)
    if not ros.hz:
        t.add_row("(critical_topics.yaml empty)", "", "")
    for topic, hz in ros.hz.items():
        t.add_row(_shorten(topic, 28), f"{hz:5.1f}", _icon(hz > 0.5))
    return Panel(t, title="Live signals", border_style="cyan")


def _render_containers_panel(
    infra: infra_checks.InfraSnapshot, areas: list[str]
) -> Panel:
    t = Table(box=None, show_header=False, expand=True, padding=(0, 1))
    t.add_column(no_wrap=True, width=2)
    t.add_column(no_wrap=True, overflow="ellipsis", ratio=2)
    t.add_column(no_wrap=True, overflow="ellipsis", ratio=3, style="dim")
    for name, cs in sorted(infra.containers.items()):
        t.add_row(_icon(cs.ok), name, cs.detail)
    for area in sorted(REMOTE_AREAS & set(areas)):
        t.add_row(
            Text("⇄", style="magenta"),
            f"home2-{area}-*",
            Text("remote Orin", style="magenta"),
        )
    if not t.rows:
        t.add_row(Text("•", style="dim"), "(none declared)", "")
    any_bad = any(not c.ok for c in infra.containers.values())
    return Panel(t, title="Containers", border_style="red" if any_bad else "green")


def _render_nodes_panel(
    ros: ros_introspection.RosSnapshot, area_states: dict[str, dict], task: str
) -> Panel:
    t = Table(show_header=True, header_style="bold cyan", box=None, expand=True)
    t.add_column("Area", no_wrap=True, width=13)
    t.add_column("Nodes", justify="right", no_wrap=True, width=6)
    t.add_column("", no_wrap=True, width=2)
    t.add_column("Missing", overflow="fold", ratio=1, style="red")
    any_missing = False
    for area, st in area_states.items():
        if not st["expected"]:
            continue
        any_missing |= bool(st["missing"])
        t.add_row(
            area,
            f"{len(st['running'])}/{len(st['expected'])}",
            _icon(not st["missing"]),
            " ".join(st["missing"]),
        )
    if not t.rows:
        t.add_row("(no task)" if not task else "(nothing expected)", "", "", "")
    totals = Text(
        f"Σ nodes={len(ros.nodes)} topics={len(ros.topics)} services={len(ros.services)}",
        style="dim italic",
    )
    return Panel(
        Group(t, totals),
        title=f"Nodes · {task or 'no task'}",
        border_style="red" if any_missing else "green",
    )


def _render_orphans_panel(ros: ros_introspection.RosSnapshot) -> Panel:
    t = Table(box=None, show_header=False, expand=True, padding=(0, 1))
    t.add_column(no_wrap=True, width=2)
    t.add_column(no_wrap=True, overflow="ellipsis", ratio=1)
    for top in ros.orphans_no_pub[:ORPHAN_RED_MAX]:
        t.add_row(
            _icon(False), f"{_shorten(top, 32)}  s={ros.topics[top].sub_count} p=0"
        )
    for top in ros.orphans_no_sub[:ORPHAN_YELLOW_MAX]:
        t.add_row(
            Text("•", style="yellow"),
            f"{_shorten(top, 32)}  p={ros.topics[top].pub_count} s=0",
        )
    if not t.rows:
        t.add_row(Text("✓", style="green"), "no orphan topics")
    extra_red = max(0, len(ros.orphans_no_pub) - ORPHAN_RED_MAX)
    extra_yellow = max(0, len(ros.orphans_no_sub) - ORPHAN_YELLOW_MAX)
    if extra_red or extra_yellow:
        t.add_row(
            Text("…", style="dim"),
            Text(f"+{extra_red} red · +{extra_yellow} yellow", style="dim"),
        )
    return Panel(t, title="Orphan topics", border_style="yellow")


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
        down = [c for c in st["containers"] if not infra.containers[c].ok]
        if down and st["missing"]:
            lines.append(
                _hint(
                    f" ⨯ {area}: containers down + nodes missing → ./run.sh {area} --recreate",
                    "red",
                )
            )
    if ros.error:
        lines.append(_hint(f" ⨯ {ros.error}", "red"))
    for top in ros.orphans_no_pub[:2]:
        lines.append(_hint(f" ⨯ {_shorten(top, 60)}: subs>0 but pubs=0", "red"))
    if not lines:
        lines.append(_hint(" ✓ everything green", "green"))
    return Panel(Group(*lines[:4]), title="Hints", border_style="magenta")


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
        Layout(name="nodes", ratio=2),
        Layout(name="orphans", ratio=1),
    )
    return root


def _header(env_type: str, iface: str, task: str, areas: list[str]) -> Panel:
    area_label = areas[0] if len(areas) == 1 else "all"
    msg = Text(
        f"FRIDA STATUS · env={env_type} · iface={iface or 'autodetermine'} "
        f"· area={area_label} · task={task or '(none)'} · "
        f"{datetime.now():%H:%M:%S} · refresh {REFRESH_SECONDS:.0f}s",
        style="bold white on blue",
    )
    return Panel(msg, border_style="blue")


def _load_area_states(areas: list[str], task: str) -> dict[str, dict]:
    states = {}
    for area in areas:
        remote = area in REMOTE_AREAS
        states[area] = {
            "expected": area_config.load_area_nodes(area, task) if task else [],
            "containers": [] if remote else area_config.load_area_containers(area),
        }
    return states


def _update_node_states(area_states: dict[str, dict], running_nodes: set[str]) -> None:
    for st in area_states.values():
        st["running"] = [n for n in st["expected"] if n in running_nodes]
        st["missing"] = [n for n in st["expected"] if n not in running_nodes]


def main() -> int:
    areas, task = _parse_args(sys.argv[1:])
    area_states = _load_area_states(areas, task)
    container_names = list(
        dict.fromkeys(c for st in area_states.values() for c in st["containers"])
    )

    console = Console()
    layout = _build_layout()
    try:
        with Live(layout, console=console, refresh_per_second=2, screen=True):
            while True:
                infra_snap = infra_checks.snapshot(container_names)
                ros_snap = ros_introspection.snapshot(hz_window=HZ_WINDOW)
                _update_node_states(area_states, set(ros_snap.nodes))

                layout["header"].update(
                    _header(
                        infra_snap.env_type, infra_snap.dds.cyclone_iface, task, areas
                    )
                )
                layout["dds"].update(_render_dds_panel(infra_snap))
                layout["logs"].update(_render_logs_panel())
                layout["signals"].update(_render_signals_panel(ros_snap))
                layout["containers"].update(_render_containers_panel(infra_snap, areas))
                layout["nodes"].update(_render_nodes_panel(ros_snap, area_states, task))
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
