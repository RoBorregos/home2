#!/bin/bash
# Live FRIDA status dashboard. Usage: scripts/dashboard.sh [area] --<task> [run.sh flags]

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "$ROS_DISTRO" ] && [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

if ! python3 -c "import rich, yaml" 2>/dev/null; then
  echo "Missing dashboard deps. Install with: sudo apt install python3-rich python3-yaml"
  exit 1
fi

export PYTHONPATH="$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m status.dashboard "$@"
