#!/bin/bash
# Fase 1.1 - Container & DDS host infrastructure checks.
# Reuses color helpers from check_nodes.sh (source_colors must be called first).

EXPECTED_RMEM_MAX=2147483647
CYCLONE_XML_PATH="/etc/cyclonedds.xml"
CYCLONE_SYSCTL_PATH="/etc/sysctl.d/60-cyclonedds-buffers.conf"
CYCLONE_ENV_PATH="/etc/cyclonedds.env"

# Track infra failures globally so the final summary can correlate with missing nodes.
declare -gA INFRA_FAILED_AREAS=()
INFRA_DDS_OK="unknown"

_print_header() {
    echo -e "${BLUE_BG_WHITE} →   ${1} ${NC}"
}

_jetson_detected() {
    [ -f /etc/nv_tegra_release ]
}

_shm_expected() {
    if [ "${CYCLONE_SHM:-}" = "1" ]; then
        return 0
    fi
    if [ -z "${CYCLONE_SHM:-}" ] && _jetson_detected; then
        return 0
    fi
    return 1
}

# check_containers <nameref to associative array> <area title>
# Reads the "containers" key from a *_INFRA assoc array and checks each via docker ps.
check_containers() {
    local -n INFRA_MAP=$1
    local AREA_TITLE="${2:-Area}"
    local CONTAINERS="${INFRA_MAP[containers]:-}"

    _print_header "${AREA_TITLE} Containers"

    if [ -z "$CONTAINERS" ]; then
        echo -e "${BLUE}(no containers declared)${NC}"
        return 0
    fi

    if ! command -v docker >/dev/null 2>&1; then
        echo -e "${RED} ⨯ docker CLI not found on host${NC}"
        INFRA_FAILED_AREAS["$AREA_TITLE"]="docker-missing"
        return 1
    fi

    local up=0 total=0 failed=()
    for container in $CONTAINERS; do
        total=$((total + 1))
        local status
        status=$(docker ps -a --filter "name=^${container}$" --format '{{.Status}}' 2>/dev/null)

        if [ -z "$status" ]; then
            echo -e "${RED} ⨯ ${container} (not created — run \`./run.sh\` for the area)${NC}"
            failed+=("$container")
        elif [[ "$status" == Up* ]]; then
            if [[ "$status" == *"(unhealthy)"* ]]; then
                echo -e "${RED} ⨯ ${container} (${status})${NC}"
                failed+=("$container")
            else
                echo -e "${GREEN} ✓ ${container} (${status})${NC}"
                up=$((up + 1))
            fi
        else
            echo -e "${RED} ⨯ ${container} (${status})${NC}"
            failed+=("$container")
        fi
    done

    echo -e "${BLUE}${up} / ${total} containers up${NC}"
    if [ ${#failed[@]} -gt 0 ]; then
        INFRA_FAILED_AREAS["$AREA_TITLE"]="${failed[*]}"
        return 1
    fi
    return 0
}

# check_dds_host: validates host-side CycloneDDS config + kernel buffers.
# Run once at the start of status.sh.
check_dds_host() {
    _print_header "DDS Host Config"
    local ok=true

    if [ -f "$CYCLONE_XML_PATH" ]; then
        echo -e "${GREEN} ✓ ${CYCLONE_XML_PATH} present${NC}"
    else
        echo -e "${RED} ⨯ ${CYCLONE_XML_PATH} missing (run: sudo bash scripts/setup_cyclonedds.sh)${NC}"
        ok=false
    fi

    if [ -f "$CYCLONE_SYSCTL_PATH" ]; then
        echo -e "${GREEN} ✓ ${CYCLONE_SYSCTL_PATH} present${NC}"
    else
        echo -e "${RED} ⨯ ${CYCLONE_SYSCTL_PATH} missing${NC}"
        ok=false
    fi

    local rmem
    rmem=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
    if [ "${rmem:-0}" -ge "$EXPECTED_RMEM_MAX" ]; then
        echo -e "${GREEN} ✓ net.core.rmem_max = ${rmem}${NC}"
    else
        echo -e "${RED} ⨯ net.core.rmem_max = ${rmem} (expected >= ${EXPECTED_RMEM_MAX})${NC}"
        ok=false
    fi

    if [ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ]; then
        echo -e "${GREEN} ✓ RMW_IMPLEMENTATION = rmw_cyclonedds_cpp${NC}"
    else
        echo -e "${RED} ⨯ RMW_IMPLEMENTATION = '${RMW_IMPLEMENTATION:-unset}' (expected rmw_cyclonedds_cpp)${NC}"
        ok=false
    fi

    if [ -f "$CYCLONE_ENV_PATH" ]; then
        local iface
        iface=$(grep -E '^CYCLONE_INTERFACE=' "$CYCLONE_ENV_PATH" 2>/dev/null | cut -d= -f2-)
        if [ -n "$iface" ]; then
            echo -e "${GREEN} ✓ CYCLONE_INTERFACE = ${iface}${NC}"
        else
            echo -e "${BLUE} • CYCLONE_INTERFACE = autodetermine${NC}"
        fi
    else
        echo -e "${RED} ⨯ ${CYCLONE_ENV_PATH} missing (containers won't pick up the interface)${NC}"
        ok=false
    fi

    if _shm_expected; then
        local roudi_status
        roudi_status=$(docker ps --filter "name=^home2-roudi$" --format '{{.Status}}' 2>/dev/null)
        if [ -n "$roudi_status" ]; then
            echo -e "${GREEN} ✓ home2-roudi (${roudi_status}) — SHM ready${NC}"
        else
            echo -e "${RED} ⨯ home2-roudi not running (SHM expected on this host)${NC}"
            ok=false
        fi
    else
        echo -e "${BLUE} • SHM not expected (non-Jetson host, CYCLONE_SHM != 1)${NC}"
    fi

    if $ok; then
        INFRA_DDS_OK="ok"
        echo -e "${BLUE}✓ DDS host config looks healthy${NC}"
        return 0
    else
        INFRA_DDS_OK="fail"
        return 1
    fi
}

# print_infra_summary: end-of-run hint correlating infra failures with the missing nodes.
print_infra_summary() {
    if [ "$INFRA_DDS_OK" = "fail" ] || [ ${#INFRA_FAILED_AREAS[@]} -gt 0 ]; then
        _print_header "Diagnóstico rápido"
        if [ "$INFRA_DDS_OK" = "fail" ]; then
            echo -e "${RED} ⨯ Host DDS config incomplete — fix this first; missing nodes may just be a symptom.${NC}"
            echo -e "${BLUE}   → sudo bash scripts/setup_cyclonedds.sh${NC}"
        fi
        for area in "${!INFRA_FAILED_AREAS[@]}"; do
            echo -e "${RED} ⨯ ${area}: containers down (${INFRA_FAILED_AREAS[$area]})${NC}"
            local lower
            lower=$(echo "$area" | tr '[:upper:]' '[:lower:]')
            echo -e "${BLUE}   → ./run.sh ${lower} --recreate${NC}"
        done
    fi
}
