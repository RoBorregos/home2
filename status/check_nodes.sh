#!/bin/bash

source_colors() {
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    BLUE='\033[0;34m'
    BLUE_BG_WHITE='\033[44;1;37m'
    NC='\033[0m'
}

check_nodes() {
    local -n EXPECTED_NODES=$1
    local HEADER="${2:-Node Status Check}"
    
    
    echo -e "${BLUE_BG_WHITE}=== ${HEADER} ===${NC}"

    ALL_RUNNING_NODES=$(ros2 node list 2>/dev/null)
    MISSING_NODES=()
    RUNNING_NODES=()

    for node in ${EXPECTED_NODES}; do
        if grep -q "$node" <<< "$ALL_RUNNING_NODES"; then
            RUNNING_NODES+=("$node")
        else
            MISSING_NODES+=("$node")
        fi
    done

    echo "Running nodes"

    for node in "${RUNNING_NODES[@]}"; do
        echo -e "${GREEN} ✓ $node${NC}"
    done

    echo "Missing nodes"

    for node in "${MISSING_NODES[@]}"; do
        echo -e "${RED} ⨯ $node${NC}"
    done
}