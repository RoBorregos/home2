"""
Colored logger for task managers — subsystem-based ANSI coloring.

Usage:
    from task_manager.utils.colored_logger import CLog

    CLog.nav(self, "MOVE", "Moving to kitchen")
    CLog.manip(self, "PICK", f"Failed to pick {name}", level="error")
    CLog.fsm(self, "STATE", f"{old} → {new}", level="success")

Subsystems & colors:
    NAV     — Cyan        [96m
    MANIP   — Yellow      [93m
    VISION  — Magenta     [95m
    HRI     — Green       [92m
    FSM     — White bold  [1m

Levels & message colors:
    info    — no override (subsystem color on prefix only)
    success — green       [92m
    warn    — yellow      [93m
    error   — red         [91m
"""

_C = {
    "NAV": "\033[96m",
    "VISION": "\033[95m",
    "MANIP": "\033[93m",
    "HRI": "\033[92m",
    "FSM": "\033[1m",
}

_LEVEL = {
    "info": "",
    "success": "\033[92m",
    "warn": "\033[93m",
    "error": "\033[91m",
}

_RESET = "\033[0m"


def _clog(node, subsystem: str, sub_area: str, msg: str, level: str = "info"):
    """Dual output: print() with ANSI colors + get_logger() for ROS2."""
    tag = f"[{subsystem}:{sub_area}]"
    prefix = f"{_C.get(subsystem, '')}{tag}{_RESET}"
    level_color = _LEVEL.get(level, "")

    if level_color:
        colored_line = f"{prefix} {level_color}{msg}{_RESET}"
    else:
        colored_line = f"{prefix} {msg}"

    print(colored_line)

    ros_msg = f"{tag} {msg}"
    if level == "error":
        node.get_logger().error(ros_msg)
    elif level == "warn":
        node.get_logger().warn(ros_msg)
    else:
        node.get_logger().info(ros_msg)


class CLog:
    """Colored subsystem logger — drop-in companion for Logger."""

    @staticmethod
    def nav(node, sub_area: str, msg: str, level: str = "info"):
        _clog(node, "NAV", sub_area, msg, level)

    @staticmethod
    def manip(node, sub_area: str, msg: str, level: str = "info"):
        _clog(node, "MANIP", sub_area, msg, level)

    @staticmethod
    def vision(node, sub_area: str, msg: str, level: str = "info"):
        _clog(node, "VISION", sub_area, msg, level)

    @staticmethod
    def hri(node, sub_area: str, msg: str, level: str = "info"):
        _clog(node, "HRI", sub_area, msg, level)

    @staticmethod
    def fsm(node, sub_area: str, msg: str, level: str = "info"):
        _clog(node, "FSM", sub_area, msg, level)
