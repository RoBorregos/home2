#!/bin/bash
source lib.sh

INPUT=$1
AREAS="vision manipulation navigation integration hri"
# check arguments passed as --help or -h
if [ "$INPUT" == "--help" ] || [ "$INPUT" == "-h" ] || [ -z "$INPUT" ]; then
  echo "Usage: ./run.sh [area] [--task] [--flags]"
  echo "Example: ./run.sh hri --receptionist --open-display"
  exit 0
fi

case $INPUT in
  vision|manipulation|navigation|integration|hri|frida_interfaces|stop|down)
    ;;
  *)
    echo "Invalid service name provided. Valid args are: vision, manipulation, navigation, integration, hri, frida_interfaces, stop, down"
    exit 1
    ;;
esac


# Check type of environment (cpu, cuda, or l4t), default cpu
ENV_TYPE=cpu

if [ -f /etc/nv_tegra_release ]; then
  ENV_TYPE=l4t
elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
  ENV_TYPE=cuda
fi
echo "Detected environment: $ENV_TYPE"

# Check and build base image if it doesn't exist
check_image_exists "roborregos/home2:${ENV_TYPE}_base"
if [ $? -eq 1 ]; then
  docker compose -f docker/${ENV_TYPE}.yaml build
fi

# Run the selected area
if [ "$INPUT" = "frida_interfaces" ]; then
  echo "Running frida_interfaces_cache to build frida_interfaces"
  docker compose -f docker/frida_interfaces_cache/docker-compose-${ENV_TYPE}.yaml run --rm frida_interfaces_cache
elif [ "$INPUT" = "stop" ]; then
  echo "Stopping all container and tmux sessions"
  if tmux ls >/dev/null 2>&1; then
    tmux kill-server || true
  fi
  if docker info >/dev/null 2>&1; then
        running_containers="$(docker ps -q)"
        if [ -n "$running_containers" ]; then
            docker stop $running_containers >/dev/null
            echo "Stopped containers: $running_containers"
        fi
    fi
elif [ "$INPUT" = "down" ]; then
  for area in $AREAS; do
    echo "Area: $area"
    AREA_RUN="docker/${area}/run.sh"
    if [ -f "$AREA_RUN" ]; then
      echo "Running: bash $AREA_RUN --stop"
      cd "docker/${area}" || continue
      bash "./run.sh" --stop "${ENV_TYPE}"
      cd - > /dev/null
      continue
    fi
  done
  echo "All areas down."
  exit 0
else
  # If frida_interfaces_cache hasn't been built yet, build it first
  if [ ! -d "docker/frida_interfaces_cache/build" ]; then
    echo "Cache directory missing. Running frida_interfaces_cache to build frida_interfaces"
    docker compose -f docker/frida_interfaces_cache/docker-compose-${ENV_TYPE}.yaml run --rm frida_interfaces_cache
  fi

  echo "Running image from area: $INPUT"
  cd "docker/$INPUT" || { echo "Error: failed to cd into docker/$INPUT" >&2; exit 1; }
  ./run.sh "${@:2}" ${ENV_TYPE}
fi