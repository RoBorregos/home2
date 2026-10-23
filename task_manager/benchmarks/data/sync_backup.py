#!/usr/bin/env python3
"""
Refresh the offline schedule backup from the Robot API (GET requests only).

The API has no "list everything" endpoint, so pass the team names to snapshot:
    python3 sync_backup.py "Los Robotines" "Las Chispas"
    python3 sync_backup.py --teams-file teams.txt      # one team name per line
Writes schedule_backup.json next to this script (gitignored).
"""

import argparse
import importlib.util
import json
import os
import time
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

# Load the module file directly so this runs on any host python (no ROS needed)
_spec = importlib.util.spec_from_file_location(
    "robot_api", os.path.join(REPO, "frida_constants", "frida_constants", "robot_api.py")
)
robot_api = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(robot_api)

REQUEST_PAUSE_S = 0.3  # be gentle with the dashboard


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("teams", nargs="*")
    parser.add_argument("--teams-file")
    parser.add_argument("--env-file", default=os.path.join(REPO, "docker", "robot_api.env"))
    parser.add_argument("--out", default=os.path.join(HERE, "schedule_backup.json"))
    args = parser.parse_args()

    names = list(args.teams)
    if args.teams_file:
        with open(args.teams_file) as file:
            names += [line.strip() for line in file if line.strip()]
    if not names:
        parser.error("Give at least one team name")

    values = robot_api.load_env_file(args.env_file)
    api = robot_api.RobotAPI(
        os.getenv("ROBOT_API_URL", values.get("ROBOT_API_URL", "")),
        os.getenv("ROBOT_API_KEY", values.get("ROBOT_API_KEY", "")),
    )

    teams, timezone_name = [], "America/Monterrey"
    for name in names:
        result = api.search_teams(name, limit=1)
        time.sleep(REQUEST_PAUSE_S)
        if not result["matches"]:
            print(f"! team not found: {name}")
            continue
        match = result["matches"][0]
        detail = api.team(match["id"])
        time.sleep(REQUEST_PAUSE_S)
        timezone_name = detail.get("timezone", timezone_name)
        teams.append(
            {
                "id": match["id"],
                "name": detail["name"],
                "track": detail.get("track"),
                "challenge": detail.get("challenge"),
                "members": detail.get("members", []),
                "rounds": detail.get("rounds", []),
            }
        )
        print(f"+ {detail['name']} ({len(detail.get('members', []))} members)")

    backup = {
        "edition": 2026,
        "timezone": timezone_name,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "teams": teams,
        "candidates_without_team": [],
    }
    with open(args.out, "w") as file:
        json.dump(backup, file, indent=2, ensure_ascii=False)
    print(f"Saved {len(teams)} teams to {args.out}")


if __name__ == "__main__":
    main()
