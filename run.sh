#!/bin/bash
source lib.sh

INPUT=$1

# Check type of environment (cpu, cuda, or l4t), default cpu
ENV_TYPE=cpu

if [ -f /etc/nv_tegra_release ]; then
  ENV_TYPE=l4t
elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
  ENV_TYPE=cuda
fi
echo "Detected environment: $ENV_TYPE"

# Reset .env
echo "" > docker/.env
add_or_update_variable docker/.env "USER_UID" "$(id -u)"
add_or_update_variable docker/.env "USER_GID" "$(id -g)"

# Check and build base image if it doesn't exist
check_image_exists "roborregos/home2:${ENV_TYPE}_base"
if [ $? -eq 1 ]; then
  docker compose -f docker/${ENV_TYPE}.yaml build
fi

# check arguments passed as --help or -h
if [ "$INPUT" == "--help" ] || [ "$INPUT" == "-h" ] || [ -z "$INPUT" ]; then
  cat << EOF
Usage: ./run.sh [area | task | command] [flags]

Description:
  This script is the main entry point to initialize and manage the Docker 
  container environment for the FRIDA robot. It automatically detects the 
  architecture (CPU, CUDA, L4T) and spins up the corresponding services.

Available Areas:
  vision             Spins up the containers for the vision area.
  manipulation       Spins up the containers for manipulation.
  navigation         Spins up the containers for navigation.
  hri                Spins up the containers for human-robot interaction.
  integration        Spins up the complete integration environment.
  frida_interfaces   Builds and configures FRIDA's custom interfaces/messages.

Competition Tasks:
  --hric             Runs the Human-Robot Interaction challenge.
  --ppc              Runs the Pick and Place challenge.
  --gpsr             Runs the General Purpose Service Robot challenge.
  --dlc              Runs the Doing Laundry Challenge.
  --restaurant       Runs the Restaurant challenge.
  --finals           Runs the finals stage routine.

Control Commands:
  --stop             Stops the running containers without removing them.
  --down             Stops and removes all containers, networks, and volumes.

Additional Flags:
  --build            Builds the ros2 packages inside the container.
  --build-image      Builds the Docker image for the specified area or task.
  --recreate         Forces the recreation of containers (useful for network or .env changes).
  --open-display     Opens the graphical interface (UI/Display) required for HRI or Vision.

Examples:
  ./run.sh hri --receptionist --open-display
  ./run.sh vision --build
  ./run.sh --gpsr --recreate
  ./run.sh --down
EOF
  exit 0
fi

case $INPUT in
  frida_interfaces)
    run_frida_interfaces
    ;;
  --stop|--down)
    control "$INPUT"
    ;;
  --hric|--ppc|--gpsr|--dlc|--restaurant|--finals)
    run_task "$@"
    ;;
  vision|manipulation|navigation|integration|hri)
    run_area "$@"
    ;;
  *)
    ./run.sh --help
    exit 1
    ;;
esac