#!/bin/bash
# Setup or revert CycloneDDS for ROS 2
#
# SHM (iceoryx zero-copy) is handled inside Docker containers only.
# The host just needs CycloneDDS network config and kernel buffer tuning.
#
# Usage:
#   sudo bash setup_cyclonedds.sh [INTERFACE]        - Full setup (host machine)
#   sudo bash setup_cyclonedds.sh --host-only [IFACE] - Only apply sysctl + save interface
#   sudo bash setup_cyclonedds.sh --revert            - Revert to FastDDS

set -e

CYCLONE_XML="/etc/cyclonedds.xml"
SYSCTL_CONF="/etc/sysctl.d/60-cyclonedds-buffers.conf"
ENV_MARKER="# CycloneDDS setup"

# Detect shell rc files
USER_HOME="/home/${SUDO_USER:-$USER}"
RC_FILES=()
[ -f "$USER_HOME/.bashrc" ] && RC_FILES+=("$USER_HOME/.bashrc")
[ -f "$USER_HOME/.zshrc" ] && RC_FILES+=("$USER_HOME/.zshrc")
[ ${#RC_FILES[@]} -eq 0 ] && RC_FILES=("$USER_HOME/.bashrc")

# ── Parse flags ──
case "${1:-}" in
    --revert)
        echo "=== Reverting to FastDDS ==="

        if [ -f "$CYCLONE_XML" ]; then
            rm "$CYCLONE_XML"
            echo "[1/4] Removed $CYCLONE_XML"
        else
            echo "[1/4] $CYCLONE_XML not found, skipping"
        fi

        if [ -f "$SYSCTL_CONF" ]; then
            rm "$SYSCTL_CONF"
            sysctl --system > /dev/null 2>&1
            echo "[2/4] Removed $SYSCTL_CONF and reloaded sysctl"
        else
            echo "[2/4] $SYSCTL_CONF not found, skipping"
        fi

        # Stop and remove RouDi systemd service (legacy cleanup)
        if systemctl is-active --quiet iceoryx-roudi 2>/dev/null; then
            systemctl stop iceoryx-roudi
        fi
        if [ -f "/etc/systemd/system/iceoryx-roudi.service" ]; then
            systemctl disable iceoryx-roudi 2>/dev/null
            rm "/etc/systemd/system/iceoryx-roudi.service"
            systemctl daemon-reload
            echo "[3/4] Removed legacy RouDi systemd service"
        else
            echo "[3/4] No legacy RouDi service, skipping"
        fi

        # Clean stale iceoryx artifacts and legacy files
        rm -f /tmp/roudi.lock 2>/dev/null
        find /dev/shm -maxdepth 1 -name 'iceoryx*' -delete 2>/dev/null || true
        rm -f /dev/shm/iceoryx_mgmt 2>/dev/null
        rm -f /etc/iceoryx/roudi_config.toml 2>/dev/null
        rm -f "${USER_HOME}/zed_shm_override.yaml" 2>/dev/null

        CLEANED=false
        for rc in "${RC_FILES[@]}"; do
            if grep -q "$ENV_MARKER" "$rc" 2>/dev/null; then
                sed -i "/$ENV_MARKER/d" "$rc"
                sed -i '/export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp/d' "$rc"
                sed -i '/export CYCLONEDDS_URI=file:\/\//d' "$rc"
                echo "[4/4] Removed CycloneDDS env vars from $rc"
                CLEANED=true
            fi
        done
        if [ "$CLEANED" = false ]; then
            echo "[4/4] No CycloneDDS env vars found in rc files, skipping"
        fi

        echo ""
        echo "=== Reverted to FastDDS ==="
        echo "To activate in current shell:"
        echo "  source ~/$(basename "${RC_FILES[0]}")"
        echo "  unset RMW_IMPLEMENTATION CYCLONEDDS_URI"
        exit 0
        ;;
    --host-only)
        echo "=== Host-only: Applying kernel buffer settings ==="
        cat > "$SYSCTL_CONF" <<EOF
# CycloneDDS optimized buffers
net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
        sysctl -p "$SYSCTL_CONF"

        # Save interface env for Docker containers
        IFACE_ENV="/etc/cyclonedds.env"
        if [ -n "${2:-}" ]; then
            echo "CYCLONE_INTERFACE=${2}" > "$IFACE_ENV"
            echo "[INFO] Saved interface '${2}' to $IFACE_ENV"
        else
            echo "CYCLONE_INTERFACE=" > "$IFACE_ENV"
            echo "[INFO] No interface specified, saved autodetermine to $IFACE_ENV"
        fi

        echo ""
        echo "=== Done ==="
        echo "Kernel settings applied."
        exit 0
        ;;
    *)
        INTERFACE="${1:-}"
        ;;
esac

# ── Setup CycloneDDS (host — no SHM, SHM is Docker-only) ──
echo "=== CycloneDDS Setup (host) ==="

# Install CycloneDDS RMW if not present
ROS_DISTRO="${ROS_DISTRO:-humble}"
if ! dpkg -s "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" &>/dev/null; then
    echo "[0/3] Installing ros-${ROS_DISTRO}-rmw-cyclonedds-cpp..."
    apt-get update -qq && apt-get install -y -qq "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
else
    echo "[0/3] ros-${ROS_DISTRO}-rmw-cyclonedds-cpp already installed"
fi

# Detect network interface
if [ -z "$INTERFACE" ]; then
    echo "[INFO] No interface specified, using autodetermine"
    IFACE_LINE='        <NetworkInterface autodetermine="true" priority="default" multicast="default" />'
else
    if [ ! -d "/sys/class/net/$INTERFACE" ]; then
        echo "[ERROR] Interface '$INTERFACE' not found. Available interfaces:"
        ls /sys/class/net/
        exit 1
    fi
    echo "[INFO] Using interface: $INTERFACE"
    IFACE_LINE="        <NetworkInterface name=\"$INTERFACE\" priority=\"default\" multicast=\"true\" autodetermine=\"false\"/>"
fi

# Write CycloneDDS XML config (no SHM on host — SHM runs inside Docker containers)
echo "[1/3] Writing CycloneDDS config to $CYCLONE_XML"
cat > "$CYCLONE_XML" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain>
    <General>
      <Interfaces>
$IFACE_LINE
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
      <EnableMulticastLoopback>true</EnableMulticastLoopback>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF

# Apply kernel sysctl tuning
echo "[2/3] Applying kernel buffer settings"
cat > "$SYSCTL_CONF" <<EOF
# CycloneDDS optimized buffers
net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sysctl -p "$SYSCTL_CONF"

# Save interface env for Docker containers
IFACE_ENV="/etc/cyclonedds.env"
if [ -n "$INTERFACE" ]; then
    echo "CYCLONE_INTERFACE=$INTERFACE" > "$IFACE_ENV"
    echo "[INFO] Saved interface '$INTERFACE' to $IFACE_ENV"
else
    echo "CYCLONE_INTERFACE=" > "$IFACE_ENV"
    echo "[INFO] No interface specified, saved autodetermine to $IFACE_ENV"
fi

# Add environment variables to shell rc files
echo "[3/3] Setting up environment variables"
for rc in "${RC_FILES[@]}"; do
    if ! grep -q "$ENV_MARKER" "$rc" 2>/dev/null; then
        cat >> "$rc" <<EOF

$ENV_MARKER
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$CYCLONE_XML
EOF
        echo "[INFO] Added environment variables to $rc"
    else
        echo "[INFO] Environment variables already in $rc, skipping"
    fi
done

echo ""
echo "=== Setup Complete ==="
echo "CycloneDDS config: $CYCLONE_XML (network only, no SHM)"
echo "Kernel tuning:     $SYSCTL_CONF"
echo ""
echo "SHM (iceoryx zero-copy) is enabled inside Docker containers automatically."
echo "To activate in current shell:"
echo "  source ~/$(basename "${RC_FILES[0]}")"
