"""Live FRIDA status dashboard.

Usage:
    python3 -m status.dashboard [area] [task]
    python3 -m status.dashboard --gpsr           # all areas, one task
    python3 -m status.dashboard vision --gpsr    # single area

Defaults: refreshes every 2s. Ctrl+C to exit cleanly."""

from __future__ import annotations

import sys
import time
from datetime import datetime

from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from status import area_config, infra_checks, ros_introspection

ALL_AREAS = ("vision", "manipulation", "navigation", "hri", "integration")
REFRESH_SECONDS = 2.0
HZ_WINDOW = 0.8


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


def _render_containers_panel(infra: infra_checks.InfraSnapshot) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column()
    t.add_column()
    t.add_column(style="dim")
    if not infra.containers:
        t.add_row(Text("•", style="dim"), "(no containers declared)", "")
    else:
        for name, cs in sorted(infra.containers.items()):
            t.add_row(_icon(cs.ok), name, cs.detail[:30])
    any_bad = any(not c.ok for c in infra.containers.values())
    return Panel(t, title="Containers", border_style="red" if any_bad else "green")


def _render_areas_panel(area_states: dict[str, dict]) -> Panel:
    t = Table(show_header=True, header_style="bold cyan", box=None)
    t.add_column("Area")
    t.add_column("Nodes", justify="right")
    t.add_column("")
    t.add_column("Missing", style="red")
    for area, st in area_states.items():
        running = st["running"]
        expected = st["expected"]
        missing = st["missing"]
        ratio = f"{len(running)}/{len(expected)}" if expected else "—"
        icon = _icon(not missing and bool(expected))
        miss_txt = ", ".join(m.lstrip("/") for m in missing[:3])
        if len(missing) > 3:
            miss_txt += f" +{len(missing)-3}"
        t.add_row(area.capitalize(), ratio, icon, miss_txt)
    return Panel(t, title="Areas / Nodes", border_style="cyan")


def _render_signals_panel(ros: ros_introspection.RosSnapshot) -> Panel:
    t = Table(show_header=True, header_style="bold cyan", box=None)
    t.add_column("Topic")
    t.add_column("Hz", justify="right")
    t.add_column("")
    if not ros.hz:
        t.add_row("(critical_topics.yaml empty)", "", "")
    else:
        for topic, hz in ros.hz.items():
            ok = hz > 0.5
            t.add_row(topic, f"{hz:5.1f}", _icon(ok))
    return Panel(t, title="Live signals", border_style="cyan")


def _render_orphans_panel(ros: ros_introspection.RosSnapshot) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column()
    t.add_column()
    if ros.orphans_no_pub:
        for top in ros.orphans_no_pub[:6]:
            info = ros.topics.get(top)
            sub_count = info.sub_count if info else "?"
            t.add_row(_icon(False), f"{top} (subs={sub_count}, pubs=0)")
    if ros.orphans_no_sub:
        for top in ros.orphans_no_sub[:4]:
            info = ros.topics.get(top)
            pub_count = info.pub_count if info else "?"
            t.add_row(
                Text("•", style="yellow"),
                f"{top} (pubs={pub_count}, subs=0)",
            )
    if not ros.orphans_no_pub and not ros.orphans_no_sub:
        t.add_row(Text("✓", style="green"), "no orphan topics")
    return Panel(t, title="Orphan topics", border_style="yellow")


def _render_hints(
    infra: infra_checks.InfraSnapshot,
    area_states: dict[str, dict],
    ros: ros_introspection.RosSnapshot,
) -> Panel:
    lines: list[Text] = []
    if not infra.dds.ok:
        lines.append(
            Text(
                " ⨯ DDS host config incomplete → sudo bash scripts/setup_cyclonedds.sh",
                style="red",
            )
        )
    for area, st in area_states.items():
        down = [
            c
            for c, cs in infra.containers.items()
            if c.endswith(f"-{area}") and not cs.ok
        ]
        if down and st["missing"]:
            lines.append(
                Text(
                    f" ⨯ {area}: containers down + nodes missing → "
                    f"./run.sh {area} --recreate",
                    style="red",
                )
            )
    if not ros.rclpy_available:
        lines.append(Text(f" ⨯ {ros.error}", style="red"))
    elif ros.error:
        lines.append(Text(f" ⨯ rclpy: {ros.error}", style="red"))
    for top in ros.orphans_no_pub[:2]:
        lines.append(Text(f" ⨯ {top}: has subscribers but no publisher", style="red"))
    if not lines:
        lines.append(Text(" ✓ everything green", style="green"))
    return Panel(Text("\n").join(lines), title="Hints", border_style="magenta")


def _build_layout() -> Layout:
    root = Layout()
    root.split_column(
        Layout(name="header", size=3),
        Layout(name="body", ratio=1),
        Layout(name="footer", size=6),
    )
    root["body"].split_row(
        Layout(name="left", ratio=1),
        Layout(name="middle", ratio=1),
        Layout(name="right", ratio=1),
    )
    root["left"].split_column(
        Layout(name="dds"),
        Layout(name="containers"),
    )
    root["right"].split_column(
        Layout(name="signals"),
        Layout(name="orphans"),
    )
    return root


def _header(env_type: str, iface: str) -> Panel:
    ts = datetime.now().strftime("%H:%M:%S")
    msg = Text(
        f"FRIDA STATUS · env={env_type} · iface={iface or 'autodetermine'} · {ts} "
        f"· refresh {REFRESH_SECONDS:.0f}s",
        style="bold white on blue",
    )
    return Panel(msg, border_style="blue")


def _compute_area_states(
    areas: list[str], task: str, running_nodes: set[str]
) -> dict[str, dict]:
    out = {}
    for area in areas:
        expected = area_config.load_area_nodes(area, task) if task else []
        running = [n for n in expected if n in running_nodes]
        missing = [n for n in expected if n not in running_nodes]
        out[area] = {"expected": expected, "running": running, "missing": missing}
    return out


def _collect_containers(areas: list[str]) -> list[str]:
    names: list[str] = []
    for area in areas:
        names.extend(area_config.load_area_containers(area))
    # Always include roudi if SHM expected somewhere
    if any(area_config.area_requires_shm(a) for a in areas):
        if "home2-roudi" not in names:
            names.append("home2-roudi")
    # Dedup, preserve order
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
                    _header(infra_snap.env_type, infra_snap.dds.cyclone_iface)
                )
                layout["dds"].update(_render_dds_panel(infra_snap))
                layout["containers"].update(_render_containers_panel(infra_snap))
                layout["middle"].update(_render_areas_panel(area_states))
                layout["signals"].update(_render_signals_panel(ros_snap))
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
