#!/usr/bin/env bash
# stop.sh - Kill all tmux sessions and stop all running Docker containers.

set -euo pipefail #if error

if command -v tmux >/dev/null 2>&1; then
    if tmux ls >/dev/null 2>&1; then
        echo "Killing all tmux sessions..."
        tmux kill-server || true
    else
        echo "No tmux sessions found."
    fi
else
    echo "tmux not found; skipping."
fi

if command -v docker >/dev/null 2>&1; then
    if docker info >/dev/null 2>&1; then
        running_containers="$(docker ps -q)"
        if [ -n "$running_containers" ]; then
            echo "Stopping running Docker containers..."
            docker stop $running_containers >/dev/null
            echo "Stopped containers: $running_containers"
        else
            echo "No running Docker containers."
        fi
    else
        echo "Docker daemon not reachable; skipping."
    fi
else
    echo "docker not found; skipping."
fi

echo "Done."