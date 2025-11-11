#!/bin/bash
source lib.sh

INPUT=$1
AREAS="vision manipulation navigation integration hri"

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

# check arguments passed as --help or -h
if [ "$INPUT" == "--help" ] || [ "$INPUT" == "-h" ] || [ -z "$INPUT" ]; then
  echo "Usage: ./run.sh [area] [--task] [--flags]"
  echo "Example: ./run.sh hri --receptionist --open-display"
  exit 0
fi

case $INPUT in
  frida_interfaces)
    run_frida_interfaces
    ;;
  stop)
    control --stop
    ;;
  down)
    control --down
    ;;
  vision|manipulation|navigation|integration|hri)
    run_area "$@"
    ;;
  *)
    echo "Invalid service name provided. Valid args are: vision, manipulation, navigation, integration, hri, frida_interfaces, stop, down"
    exit 1
    ;;
esac