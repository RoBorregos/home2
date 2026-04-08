#!/usr/bin/env python3
"""
Check availability of ROS topics, services, and actions from a named JSON config.

Usage:
    python3 status/check_status.py <config-name>
    python3 status/check_status.py --list
"""

import json
import subprocess
import sys
from pathlib import Path

CONFIGS_DIR = Path(__file__).parent / "configs"

# ANSI colours
RED = "\033[0;31m"
GREEN = "\033[0;32m"
BLUE = "\033[0;34m"
HEADER = "\033[48;2;0;0;139m\033[38;2;255;255;255m"
NC = "\033[0m"


def run(cmd: list[str]) -> list[str]:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return [line.strip() for line in r.stdout.splitlines() if line.strip()]
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []


def check_section(
    label: str, expected: list[str], available: list[str]
) -> tuple[int, int]:
    if not expected:
        return 0, 0

    print(f"\n{HEADER}  {label}  {NC}")
    ok = [i for i in expected if i in available]
    missing = [i for i in expected if i not in available]

    for item in ok:
        print(f"{GREEN} ✓ {item}{NC}")
    for item in missing:
        print(f"{RED} ✗ {item}{NC}")

    return len(ok), len(expected)


def list_configs():
    configs = sorted(CONFIGS_DIR.glob("*.json"))
    if not configs:
        print("No JSON configs found.")
        return
    print("Available configs:")
    for c in configs:
        print(f"  {c.stem}")


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0)

    if sys.argv[1] == "--list":
        list_configs()
        sys.exit(0)

    config_name = sys.argv[1]
    path = CONFIGS_DIR / f"{config_name}.json"

    if not path.exists():
        print(
            f"Config '{config_name}' not found. Run 'python3 status/ros_config_builder.py {config_name}' to create it."
        )
        sys.exit(1)

    with open(path) as f:
        config = json.load(f)

    def flatten(section) -> list[str]:
        if isinstance(section, list):
            return section
        items: list[str] = []
        for v in section.values():
            items.extend(v)
        return items

    topics = flatten(config.get("topics", []))
    services = flatten(config.get("services", []))
    actions = flatten(config.get("actions", []))

    print(f"{HEADER}  ROS Status Check — {config_name}  {NC}")

    # Fetch all available items once
    avail_topics = set(run(["ros2", "topic", "list"]))
    avail_services = set(run(["ros2", "service", "list"]))
    avail_actions = set(run(["ros2", "action", "list"]))

    total_ok = 0
    total = 0

    ok, n = check_section("Topics", topics, avail_topics)
    total_ok += ok
    total += n

    ok, n = check_section("Services", services, avail_services)
    total_ok += ok
    total += n

    ok, n = check_section("Actions", actions, avail_actions)
    total_ok += ok
    total += n

    print()
    if total == 0:
        print(f"{BLUE}Config '{config_name}' has no items to check.{NC}")
    elif total_ok == total:
        print(f"{GREEN}✓ All {total} items are available!{NC}")
    else:
        print(f"{BLUE}{total_ok}/{total} items available{NC}")
        sys.exit(2)


if __name__ == "__main__":
    main()
