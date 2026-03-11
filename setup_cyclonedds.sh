#!/bin/bash
# Setup or revert CycloneDDS for ROS 2 with ZED-optimized settings
#
# Usage:
#   sudo bash setup_cyclonedds.sh [INTERFACE]        - Full setup (host machine)
#   sudo bash setup_cyclonedds.sh --docker [IFACE]   - Setup inside Docker (skips sysctl)
#   sudo bash setup_cyclonedds.sh --host-only         - Only apply sysctl on host
#   sudo bash setup_cyclonedds.sh --revert            - Revert to FastDDS
#
# Examples:
#   sudo bash setup_cyclonedds.sh eno1          # On Orin / bare metal
#   sudo bash setup_cyclonedds.sh --docker eno1 # Inside Docker container
#   sudo bash setup_cyclonedds.sh --host-only   # On PC host (run once)
#   sudo bash setup_cyclonedds.sh --revert      # Go back to FastDDS

set -e

CYCLONE_XML="/etc/cyclonedds.xml"
SYSCTL_CONF="/etc/sysctl.d/60-cyclonedds-buffers.conf"
ENV_MARKER="# CycloneDDS setup"
DOCKER_MODE=false
HOST_ONLY=false

# Detect shell rc files
USER_HOME="/home/${SUDO_USER:-$USER}"
RC_FILES=()
[ -f "$USER_HOME/.bashrc" ] && RC_FILES+=("$USER_HOME/.bashrc")
[ -f "$USER_HOME/.zshrc" ] && RC_FILES+=("$USER_HOME/.zshrc")
# Fallback to bashrc if neither exists
[ ${#RC_FILES[@]} -eq 0 ] && RC_FILES=("$USER_HOME/.bashrc")

# ── Parse flags ──
case "${1:-}" in
    --revert)
        echo "=== Reverting to FastDDS ==="

        if [ -f "$CYCLONE_XML" ]; then
            rm "$CYCLONE_XML"
            echo "[1/3] Removed $CYCLONE_XML"
        else
            echo "[1/3] $CYCLONE_XML not found, skipping"
        fi

        if [ -f "$SYSCTL_CONF" ]; then
            rm "$SYSCTL_CONF"
            sysctl --system > /dev/null 2>&1
            echo "[2/3] Removed $SYSCTL_CONF and reloaded sysctl"
        else
            echo "[2/3] $SYSCTL_CONF not found, skipping"
        fi

        CLEANED=false
        for rc in "${RC_FILES[@]}"; do
            if grep -q "$ENV_MARKER" "$rc" 2>/dev/null; then
                sed -i "/$ENV_MARKER/d" "$rc"
                sed -i '/export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp/d' "$rc"
                sed -i '/export CYCLONEDDS_URI=file:\/\//d' "$rc"
                echo "[3/3] Removed CycloneDDS env vars from $rc"
                CLEANED=true
            fi
        done
        if [ "$CLEANED" = false ]; then
            echo "[3/3] No CycloneDDS env vars found in rc files, skipping"
        fi

        echo ""
        echo "=== Reverted to FastDDS ==="
        echo "To activate in current shell:"
        echo "  source ~/$(basename "${RC_FILES[0]}")"
        echo "  unset RMW_IMPLEMENTATION CYCLONEDDS_URI"
        exit 0
        ;;
    --docker)
        DOCKER_MODE=true
        INTERFACE="${2:-}"
        ;;
    --host-only)
        HOST_ONLY=true
        ;;
    *)
        INTERFACE="${1:-}"
        ;;
esac

# ── Host-only: apply sysctl + save interface for Docker containers ──
if [ "$HOST_ONLY" = true ]; then
    echo "=== Host-only: Applying kernel buffer settings ==="
    cat > "$SYSCTL_CONF" <<EOF
# CycloneDDS / ZED camera optimized buffers
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
        echo "       Use with: docker run --env-file $IFACE_ENV ..."
    else
        echo "CYCLONE_INTERFACE=" > "$IFACE_ENV"
        echo "[INFO] No interface specified, saved autodetermine to $IFACE_ENV"
    fi

    echo ""
    echo "=== Done ==="
    echo "Kernel settings applied."
    echo ""
    echo "For Docker containers, pass the interface with either:"
    echo "  docker run --env-file /etc/cyclonedds.env ..."
    echo "  docker run -e CYCLONE_INTERFACE=eno1 ..."
    exit 0
fi

# ── Setup CycloneDDS ──
if [ "$DOCKER_MODE" = true ]; then
    echo "=== CycloneDDS Setup (Docker mode - skipping sysctl) ==="
    echo "[NOTE] Make sure you ran 'sudo bash setup_cyclonedds.sh --host-only' on the host first!"
else
    echo "=== CycloneDDS Setup ==="
fi

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

# Write CycloneDDS XML config
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

# Apply kernel sysctl tuning (skip in Docker)
if [ "$DOCKER_MODE" = true ]; then
    echo "[2/3] Skipping sysctl (Docker mode - must be set on host)"
else
    echo "[2/3] Applying kernel buffer settings"
    cat > "$SYSCTL_CONF" <<EOF
# CycloneDDS / ZED camera optimized buffers
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
    echo "       Use with: docker run --env-file $IFACE_ENV ..."
else
    echo "CYCLONE_INTERFACE=" > "$IFACE_ENV"
    echo "[INFO] No interface specified, saved autodetermine to $IFACE_ENV"
fi

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
echo "CycloneDDS config: $CYCLONE_XML"
if [ "$DOCKER_MODE" = false ]; then
    echo "Kernel tuning:     $SYSCTL_CONF"
fi
echo ""
echo "To activate in current shell:"
echo "  source ~/$(basename "${RC_FILES[0]}")"
