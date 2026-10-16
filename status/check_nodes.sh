#!/bin/bash

source_colors() {
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[0;33m'
    BLUE='\033[0;34m'
    BLUE_BG_WHITE='\033[48;2;0;0;139m\033[38;2;255;255;255m'
    NC='\033[0m'
}

container_node_list() {
    if [ -z "${_CONTAINER_NODES+x}" ]; then
        local container
        container=$(docker ps --format "{{.Names}}" 2>/dev/null | grep -E "^home2-(integration|hri-ros|vision|manipulation|navigation)" | head -1)
        _CONTAINER_NODES=""
        if [ -n "$container" ]; then
            _CONTAINER_NODES=$(docker exec "$container" bash -c "source /opt/ros/jazzy/setup.bash; [ -f /usr/local/bin/cyclonedds_setup.sh ] && source /usr/local/bin/cyclonedds_setup.sh >/dev/null 2>&1; timeout 15 ros2 node list --no-daemon 2>/dev/null")
        fi
    fi
    echo "$_CONTAINER_NODES"
}

check_nodes() {
    local -n EXPECTED_NODES=$1
    local HEADER="${2:-Node Status Check}"
    
    
    echo -e "${BLUE_BG_WHITE} →   ${HEADER} ${NC}"

    ALL_RUNNING_NODES=$(ros2 node list 2>/dev/null)
    if [ -z "$ALL_RUNNING_NODES" ]; then
        ALL_RUNNING_NODES=$(container_node_list)
    fi
    MISSING_NODES=()
    RUNNING_NODES=()
    COUNT=0

    for node in ${EXPECTED_NODES}; do
        if grep -q "$node" <<< "$ALL_RUNNING_NODES"; then
            RUNNING_NODES+=("$node")
        else
            MISSING_NODES+=("$node")
        fi
        COUNT=$((COUNT + 1))
    done

    if [ ${#RUNNING_NODES[@]} -gt 0 ]; then
        echo "Running nodes" 
    fi

    for node in "${RUNNING_NODES[@]}"; do
        echo -e "${GREEN} ✓ $node${NC}"
    done

    if [ ${#MISSING_NODES[@]} -gt 0 ]; then
        echo "Missing nodes"
    fi

    for node in "${MISSING_NODES[@]}"; do
        echo -e "${RED} ⨯ $node${NC}"
    done

    RESULTS="${#RUNNING_NODES[@]} / ${COUNT}"
    if [ ${#MISSING_NODES[@]} -eq 0 ]; then
        echo -e "${BLUE}✓ All nodes are running! (${RESULTS})${NC}"
    else
        echo -e "${BLUE}${RESULTS} nodes are running${NC}"
    fi
}