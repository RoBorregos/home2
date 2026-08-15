#!/bin/bash
# Live FRIDA status dashboard wrapper.
# - Sources ROS 2 if not yet sourced so rclpy is importable.
# - Sets PYTHONPATH so `python3 -m status.dashboard` finds the package.
# - Forwards all args (mirror of scripts/status.sh): [area] [task] or [task].

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -z "$ROS_DISTRO" ] && [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

export PYTHONPATH="$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m status.dashboard "$@"
