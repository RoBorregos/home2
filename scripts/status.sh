#!/bin/bash

ARGS=("$@")
ARG1=${ARGS[0]}
ARG2=${ARGS[1]}

# Check if the parameter is passed, if not, set a default value (e.g., "vision")
if [ -z "$ARG1" ]; then
  echo "No area or task provided. Valid areas are: vision, manipulation, etc. Or tasks like --carry, --receptionist, etc."
  exit 1
fi

# check arguments passed as --help or -h
if [ "$ARG1" == "--help" ] || [ "$ARG1" == "-h" ]; then
  echo "Usage: ./scripts/status.sh [area_name] [task] or ./scripts/status.sh [task]"
  echo "Example: ./scripts/status.sh vision --receptionist or ./scripts/status.sh --receptionist"
  exit 0
fi

# Get the directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CONFIG_PATH="$PROJECT_ROOT/status/configs"

# Load lib to check nodes required from each area
source "$PROJECT_ROOT/status/check_nodes.sh"
source_colors

if [ -n "$ARG2" ]; then
    # Run status for a specific task in a specific area
    TASK=${ARG2}
    AREA=${ARG1}
    AREA_TITLE=$(echo "$AREA" | awk '{print toupper(substr($0,1,1)) substr($0,2)}')
    AREA_CAPS=$(echo "$AREA" | tr '[:lower:]' '[:upper:]')

    source "${CONFIG_PATH}/${AREA}_nodes.cfg"
    check_nodes ${AREA_CAPS}_NODES[$TASK] "${AREA_TITLE} Status ${TASK}"

else
    # Run status for a specific task in all areas
    TASK=${ARG1}
    AREAS=("vision" "manipulation" "hri" "manipulation")

    for AREA in "${AREAS[@]}"; do
        AREA_TITLE=$(echo "$AREA" | awk '{print toupper(substr($0,1,1)) substr($0,2)}')
        AREA_CAPS=$(echo "$AREA" | tr '[:lower:]' '[:upper:]')

        source "${CONFIG_PATH}/${AREA}_nodes.cfg"
        check_nodes ${AREA_CAPS}_NODES[$TASK] "${AREA_TITLE} Status ${TASK}"
        echo ""
    done
fi
