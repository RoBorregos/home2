#!/bin/bash

# Toggle wifi radios on/off on an OpenWrt-based TP-Link router over SSH.
#
# Usage: ./scripts/wifi.sh [on|off|toggle|status]
#   Default action is "toggle".
#
# Config: set ROUTER_HOST / ROUTER_USER as env vars, or edit the defaults below.

ROUTER_HOST=${ROUTER_HOST:-192.168.31.1}
ROUTER_USER=${ROUTER_USER:-root}
SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 ${ROUTER_USER}@${ROUTER_HOST}"

ACTION=${1:-toggle}

if [ "$ACTION" == "--help" ] || [ "$ACTION" == "-h" ]; then
  echo "Usage: ./scripts/wifi.sh [on|off|toggle|status]"
  echo "Env vars: ROUTER_HOST (default 192.168.1.1), ROUTER_USER (default root)"
  exit 0
fi

# 1 = disabled, 0/empty = enabled, per OpenWrt's wireless.@wifi-device[N].disabled
get_state() {
  $SSH "uci get wireless.@wifi-device[0].disabled 2>/dev/null || echo 0"
}

set_state() {
  local disabled=$1
  $SSH "
    count=\$(uci show wireless | grep -c '=wifi-device\$')
    for i in \$(seq 0 \$((count - 1))); do
      uci set wireless.@wifi-device[\$i].disabled='${disabled}'
    done
    uci commit wireless
    wifi
  "
}

case "$ACTION" in
  status)
    state=$(get_state)
    [ "$state" == "1" ] && echo "wifi is OFF" || echo "wifi is ON"
    ;;
  off)
    set_state 1
    echo "wifi turned OFF"
    ;;
  on)
    set_state 0
    echo "wifi turned ON"
    ;;
  toggle)
    state=$(get_state)
    if [ "$state" == "1" ]; then
      set_state 0
      echo "wifi turned ON"
    else
      set_state 1
      echo "wifi turned OFF"
    fi
    ;;
  *)
    echo "Unknown action: $ACTION"
    echo "Usage: ./scripts/wifi.sh [on|off|toggle|status]"
    exit 1
    ;;
esac
