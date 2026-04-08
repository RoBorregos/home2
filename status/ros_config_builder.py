#!/usr/bin/env python3
"""
TUI tool to select ROS topics, services, and actions and save them as a named config.

Usage:
    python3 status/ros_config_builder.py
    python3 status/ros_config_builder.py <config-name>

Controls:
    Tab / Shift+Tab   Switch between Topics / Services / Actions panels
    Up / Down         Navigate items
    Space             Toggle selection (group = toggle all children)
    Enter             Collapse / expand group
    ← / →             Collapse / expand group (← on item jumps to group header)
    s                 Save config (stays open)
    q / Esc           Quit
    /                 Filter items (type to search, Enter/Esc to finish)
"""

import curses
import json
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

CONFIGS_DIR = Path(__file__).parent / "configs"
PANEL_NAMES = ["Topics", "Services", "Actions"]


def query_ros(command: list[str]) -> list[str]:
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=5)
        lines = [L.strip() for L in result.stdout.splitlines() if L.strip()]
        return sorted(lines)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []


def load_ros_items() -> dict[str, list[str]]:
    return {
        "Topics": query_ros(["ros2", "topic", "list"]),
        "Services": query_ros(["ros2", "service", "list"]),
        "Actions": query_ros(["ros2", "action", "list"]),
    }


def _flatten(section) -> list[str]:
    """Accept either a flat list (old format) or a {prefix: [items]} dict."""
    if isinstance(section, list):
        return section
    items: list[str] = []
    for v in section.values():
        items.extend(v)
    return items


def _group_for_save(items: set[str]) -> dict[str, list[str]]:
    """Group a set of item paths by first-segment prefix for JSON storage."""
    groups: dict[str, list[str]] = defaultdict(list)
    for item in sorted(items):
        parts = item.split("/")
        prefix = "/" + parts[1] if len(parts) >= 3 else ""
        groups[prefix].append(item)
    # Sort groups so standalone ("") comes last
    return dict(sorted(groups.items(), key=lambda kv: (kv[0] == "", kv[0])))


def load_existing_config(name: str) -> dict[str, list[str]]:
    path = CONFIGS_DIR / f"{name}.json"
    if path.exists():
        with open(path) as f:
            data = json.load(f)
        return {
            "Topics": _flatten(data.get("topics", [])),
            "Services": _flatten(data.get("services", [])),
            "Actions": _flatten(data.get("actions", [])),
        }
    return {"Topics": [], "Services": [], "Actions": []}


def save_config(name: str, selected: dict[str, set[str]]) -> Path:
    CONFIGS_DIR.mkdir(parents=True, exist_ok=True)
    path = CONFIGS_DIR / f"{name}.json"
    data = {
        "name": name,
        "topics": _group_for_save(selected["Topics"]),
        "services": _group_for_save(selected["Services"]),
        "actions": _group_for_save(selected["Actions"]),
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    return path


_NEW_CONFIG = "[ + New config ]"


def _input_name(stdscr, prompt_y: int, prompt_x: int, w: int) -> str | None:
    """Inline text input; returns stripped name or None on cancel."""
    curses.curs_set(1)
    chars: list[str] = []
    while True:
        field = "".join(chars)
        stdscr.addstr(prompt_y, prompt_x, " " * (w - prompt_x - 1))
        stdscr.addstr(prompt_y, prompt_x, field)
        stdscr.move(prompt_y, prompt_x + len(chars))
        stdscr.refresh()
        ch = stdscr.get_wch()
        if ch in ("\n", "\r", curses.KEY_ENTER):
            name = field.strip()
            if name:
                curses.curs_set(0)
                return name
        elif ch in ("\x03", "\x1b"):
            curses.curs_set(0)
            return None
        elif ch in (curses.KEY_BACKSPACE, "\x7f", "\b"):
            if chars:
                chars.pop()
        elif isinstance(ch, str) and ch.isprintable():
            chars.append(ch)


def prompt_config_selection(stdscr) -> str | None:
    """Show existing configs + 'New config' option; return chosen name or None."""
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLUE)
    curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_CYAN)
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_WHITE)
    stdscr.keypad(True)

    CONFIGS_DIR.mkdir(parents=True, exist_ok=True)
    existing = sorted(p.stem for p in CONFIGS_DIR.glob("*.json"))
    options = existing + [_NEW_CONFIG]
    cursor = 0

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        title = " ROS Config Builder "
        stdscr.addstr(0, 0, title.ljust(w)[:w], curses.color_pair(1) | curses.A_BOLD)

        subtitle = "Select a config to edit or create a new one"
        stdscr.addstr(2, 2, subtitle, curses.A_BOLD)

        list_y = 4
        for i, opt in enumerate(options):
            y = list_y + i
            if y >= h - 2:
                break
            is_new = opt == _NEW_CONFIG
            label = f"  {'>' if i == cursor else ' '}  {opt}"
            attr = curses.A_REVERSE if i == cursor else curses.A_NORMAL
            if is_new:
                attr |= curses.A_BOLD
            stdscr.addstr(y, 0, label[: w - 1].ljust(w - 1)[: w - 1], attr)

        help_text = " ↑↓:navigate  Enter:select  q/Esc:quit "
        stdscr.addstr(
            h - 1, 0, help_text[: w - 1].ljust(w - 1)[: w - 1], curses.color_pair(4)
        )
        stdscr.refresh()

        ch = stdscr.get_wch()

        if ch in ("q", "Q", "\x03", "\x1b"):
            return None
        elif ch == curses.KEY_UP:
            cursor = max(0, cursor - 1)
        elif ch == curses.KEY_DOWN:
            cursor = min(len(options) - 1, cursor + 1)
        elif ch in ("\n", "\r", curses.KEY_ENTER, " "):
            chosen = options[cursor]
            if chosen != _NEW_CONFIG:
                return chosen
            # Prompt for new name inline
            stdscr.erase()
            stdscr.addstr(
                0, 0, title.ljust(w)[:w], curses.color_pair(1) | curses.A_BOLD
            )
            prompt = "New config name: "
            stdscr.addstr(h // 2, 2, prompt, curses.A_BOLD)
            stdscr.refresh()
            name = _input_name(stdscr, h // 2, 2 + len(prompt), w)
            if name:
                return name
            # If cancelled, loop back to selection


class Panel:
    def __init__(self, label: str, live_items: list[str], preselected: set[str]):
        self.label = label
        self.live_set: set[str] = set(live_items)
        self.selected: set[str] = set(preselected)
        # Show all live items plus any saved-but-offline selected items
        self.items: list[str] = sorted(self.live_set | (self.selected - self.live_set))
        self.cursor = 0
        self.scroll = 0
        self.filter = ""
        self.filtering = False
        self._groups: dict[str, list[str]] = self._build_groups(self.items)
        self._collapsed: dict[str, bool] = {}

    @staticmethod
    def _build_groups(items: list[str]) -> dict[str, list[str]]:
        groups: dict[str, list[str]] = defaultdict(list)
        for item in items:
            parts = item.split("/")
            # /prefix/rest -> grouped; /standalone -> key ""
            prefix = "/" + parts[1] if len(parts) >= 3 else ""
            groups[prefix].append(item)
        return dict(groups)

    def flat_entries(self) -> list[tuple]:
        """
        Returns flat list of visible rows:
          ('item',  name,     indented: bool)
          ('group', prefix,   children: list[str], collapsed: bool)
        When filtering, returns only matching items (no groups).
        """
        if self.filter:
            q = self.filter.lower()
            return [("item", i, False) for i in self.items if q in i.lower()]
        entries: list[tuple] = []
        for item in self._groups.get("", []):
            entries.append(("item", item, False))
        for prefix in sorted(k for k in self._groups if k):
            children = self._groups[prefix]
            collapsed = self._collapsed.get(prefix, False)
            entries.append(("group", prefix, children, collapsed))
            if not collapsed:
                for child in children:
                    entries.append(("item", child, True))
        return entries

    def clamp_cursor(self):
        n = len(self.flat_entries())
        self.cursor = max(0, min(self.cursor, n - 1)) if n else 0

    def toggle(self):
        entries = self.flat_entries()
        if not entries:
            return
        e = entries[self.cursor]
        if e[0] == "group":
            children = e[2]
            if all(c in self.selected for c in children):
                self.selected.difference_update(children)
            else:
                self.selected.update(children)
        else:
            name = e[1]
            self.selected.discard(name) if name in self.selected else self.selected.add(
                name
            )

    def toggle_collapse(self):
        entries = self.flat_entries()
        if not entries:
            return
        e = entries[self.cursor]
        if e[0] == "group":
            prefix = e[1]
            self._collapsed[prefix] = not self._collapsed.get(prefix, False)
            self.clamp_cursor()

    def collapse(self):
        """Collapse current group, or jump to parent group header if on an item."""
        entries = self.flat_entries()
        if not entries:
            return
        e = entries[self.cursor]
        if e[0] == "group" and not self._collapsed.get(e[1], False):
            self._collapsed[e[1]] = True
            self.clamp_cursor()
        elif e[0] == "item" and e[2]:  # indented item -> jump to its group header
            for i in range(self.cursor - 1, -1, -1):
                if entries[i][0] == "group":
                    self.cursor = i
                    break

    def expand(self):
        """Expand current group if collapsed."""
        entries = self.flat_entries()
        if not entries:
            return
        e = entries[self.cursor]
        if e[0] == "group" and self._collapsed.get(e[1], False):
            self._collapsed[e[1]] = False


def draw(stdscr, panels: list[Panel], active: int, config_name: str, status_msg: str):
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    # Title bar
    title = f" ROS Config Builder  —  config: {config_name} "
    stdscr.addstr(0, 0, title.ljust(w)[:w], curses.color_pair(1) | curses.A_BOLD)

    # Tab headers
    tab_y = 1
    x = 0
    for i, panel in enumerate(panels):
        label = f" {panel.label} ({len(panel.selected)}) "
        attr = (
            curses.color_pair(2) | curses.A_BOLD
            if i == active
            else curses.color_pair(3)
        )
        if x + len(label) < w:
            stdscr.addstr(tab_y, x, label, attr)
            x += len(label) + 1

    # Help bar
    help_text = " Tab:switch  Space:toggle  ←→:collapse  Enter:expand/collapse  s:save  q:quit  /:filter "
    stdscr.addstr(2, 0, help_text[:w], curses.color_pair(4))

    # Panel content (may be called with empty panels during loading)
    if not panels:
        stdscr.addstr(h // 2, max(0, (w - len(status_msg)) // 2), status_msg)
        stdscr.refresh()
        return

    panel = panels[active]
    entries = panel.flat_entries()
    content_h = h - 5
    content_y = 3

    # Scroll so cursor is always in view
    if panel.cursor < panel.scroll:
        panel.scroll = panel.cursor
    elif panel.cursor >= panel.scroll + content_h:
        panel.scroll = panel.cursor - content_h + 1

    if not entries:
        msg = "(no items)" if not panel.filter else f'(no matches for "{panel.filter}")'
        stdscr.addstr(content_y, 2, msg, curses.A_DIM)
    else:
        for row, entry in enumerate(entries[panel.scroll : panel.scroll + content_h]):
            idx = panel.scroll + row
            y = content_y + row
            is_cursor = idx == panel.cursor

            if entry[0] == "group":
                _, prefix, children, collapsed = entry
                n_sel = sum(1 for c in children if c in panel.selected)
                n_total = len(children)
                arrow = "▶" if collapsed else "▼"
                line = f" {arrow} {prefix}  [{n_sel}/{n_total}]"
                if n_sel == n_total and n_total > 0:
                    attr = curses.color_pair(5) | curses.A_BOLD  # all selected: green
                elif n_sel > 0:
                    attr = curses.color_pair(6) | curses.A_BOLD  # partial: yellow
                else:
                    attr = curses.A_BOLD
            else:
                _, name, indented = entry
                is_selected = name in panel.selected
                is_live = name in panel.live_set
                checkbox = "[x]" if is_selected else "[ ]"
                indent = "    " if indented else " "
                offline_mark = " [offline]" if not is_live else ""
                line = f"{indent}{checkbox} {name}{offline_mark}"
                if is_selected and not is_live:
                    attr = curses.color_pair(7)  # saved but offline
                elif is_selected:
                    attr = curses.color_pair(5)  # live + selected: green
                elif not is_live:
                    attr = curses.A_DIM  # offline + not selected
                else:
                    attr = curses.A_NORMAL

            if is_cursor:
                attr |= curses.A_REVERSE
            stdscr.addstr(y, 0, line[: w - 1].ljust(w - 1), attr)

    # Status / filter bar at bottom
    if panel.filtering:
        filter_line = f" Filter: {panel.filter}_"
    elif status_msg:
        filter_line = f" {status_msg}"
    else:
        sel = len(panel.selected)
        total = len(panel.items)
        filter_line = f" {sel}/{total} selected"
    stdscr.addstr(h - 1, 0, filter_line[:w].ljust(w - 1)[: w - 1], curses.color_pair(4))

    stdscr.refresh()


def run_tui(stdscr, config_name: str):
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()

    # Colour pairs
    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLUE)  # title
    curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_CYAN)  # active tab
    curses.init_pair(3, curses.COLOR_CYAN, -1)  # inactive tab
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_WHITE)  # help/status bar
    curses.init_pair(5, curses.COLOR_GREEN, -1)  # live + selected
    curses.init_pair(6, curses.COLOR_YELLOW, -1)  # partially selected group
    curses.init_pair(7, curses.COLOR_MAGENTA, -1)  # saved but offline

    stdscr.keypad(True)
    stdscr.timeout(100)

    # Load items and pre-existing selections
    status_msg = "Loading ROS items..."
    draw(stdscr, [], 0, config_name, status_msg)

    ros_items = load_ros_items()
    existing = load_existing_config(config_name)

    panels = [Panel(name, ros_items[name], set(existing[name])) for name in PANEL_NAMES]
    active = 0
    status_msg = ""
    status_until = 0.0

    while True:
        if status_msg and time.monotonic() > status_until:
            status_msg = ""
        draw(stdscr, panels, active, config_name, status_msg)

        try:
            ch = stdscr.get_wch()
        except curses.error:
            continue

        panel = panels[active]

        if panel.filtering:
            if ch in ("\n", "\r", curses.KEY_ENTER, "\x1b"):
                panel.filtering = False
                panel.clamp_cursor()
            elif ch in (curses.KEY_BACKSPACE, "\x7f", "\b"):
                panel.filter = panel.filter[:-1]
                panel.clamp_cursor()
            elif isinstance(ch, str) and ch.isprintable():
                panel.filter += ch
                panel.clamp_cursor()
            continue

        if ch in ("q", "Q", "\x1b"):
            return None  # quit without saving

        elif ch in ("s", "S"):
            path = save_config(config_name, {p.label: p.selected for p in panels})
            status_msg = f"Saved to {path}"
            status_until = time.monotonic() + 3.0

        elif ch == "/":
            panel.filtering = True
            panel.filter = ""

        elif ch == "\t":
            active = (active + 1) % len(panels)

        elif ch == curses.KEY_BTAB:
            active = (active - 1) % len(panels)

        elif ch == curses.KEY_UP:
            if panel.cursor > 0:
                panel.cursor -= 1

        elif ch == curses.KEY_DOWN:
            if panel.cursor < len(panel.flat_entries()) - 1:
                panel.cursor += 1

        elif ch == curses.KEY_PPAGE:
            panel.cursor = max(0, panel.cursor - (curses.LINES - 5))

        elif ch == curses.KEY_NPAGE:
            panel.cursor = min(
                len(panel.flat_entries()) - 1, panel.cursor + (curses.LINES - 5)
            )

        elif ch == curses.KEY_LEFT:
            panel.collapse()

        elif ch == curses.KEY_RIGHT:
            panel.expand()

        elif ch in ("\n", "\r", curses.KEY_ENTER):
            panel.toggle_collapse()

        elif ch == " ":
            panel.toggle()


def main():
    config_name = sys.argv[1] if len(sys.argv) > 1 else None

    if config_name is None:
        config_name = curses.wrapper(prompt_config_selection)

    if not config_name:
        print("Cancelled.")
        sys.exit(0)

    curses.wrapper(run_tui, config_name)


if __name__ == "__main__":
    main()
