#!/bin/bash

ARGS=("$@")
ARG1=${ARGS[0]}
ARG2=${ARGS[1]}

# Check if the parameter is passed, if not, set a default value (e.g., "vision")
if [ -z "$ARG1" ]; then
  echo "No area or task provided. Valid areas are: vision, manipulation, etc. Or tasks like --hric, --gpsr, etc."
  exit 1
fi

# check arguments passed as --help or -h
if [ "$ARG1" == "--help" ] || [ "$ARG1" == "-h" ]; then
  echo "Usage: ./scripts/status.sh [area_name] [task] or ./scripts/status.sh [task]"
  echo "Example: ./scripts/status.sh vision --hric or ./scripts/status.sh --hric"
  exit 0
fi

# Get the directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CONFIG_PATH="$PROJECT_ROOT/status/configs"

source "$PROJECT_ROOT/status/check_nodes.sh"
source "$PROJECT_ROOT/status/check_infra.sh"
source_colors

check_dds_host
echo ""

run_area_status() {
    local AREA=$1
    local TASK=$2
    local AREA_TITLE
    local AREA_CAPS
    AREA_TITLE=$(echo "$AREA" | awk '{print toupper(substr($0,1,1)) substr($0,2)}')
    AREA_CAPS=$(echo "$AREA" | tr '[:lower:]' '[:upper:]')

    local INFRA_CFG="${CONFIG_PATH}/${AREA}_infra.cfg"
    if [ -f "$INFRA_CFG" ]; then
        source "$INFRA_CFG"
        check_containers ${AREA_CAPS}_INFRA "${AREA_TITLE}"
        echo ""
    fi

    local NODES_CFG="${CONFIG_PATH}/${AREA}_nodes.cfg"
    if [ -f "$NODES_CFG" ]; then
        source "$NODES_CFG"
        check_nodes "${AREA_CAPS}_NODES[$TASK]" "${AREA_TITLE} Status ${TASK}"
    fi
}

if [ -n "$ARG2" ]; then
    # Run status for a specific task in a specific area
    run_area_status "$ARG1" "$ARG2"
else
    # Run status for a specific task in all areas
    TASK=${ARG1}
    AREAS=("vision" "manipulation" "navigation" "integration" "hri" "display")

    for AREA in "${AREAS[@]}"; do
        run_area_status "$AREA" "$TASK"
        echo ""
    done
fi

echo ""
print_infra_summary
