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
ROUDI_CONFIG="/etc/iceoryx/roudi_config.toml"
ROUDI_SERVICE="/etc/systemd/system/iceoryx-roudi.service"
ENV_MARKER="# CycloneDDS setup"
DOCKER_MODE=false
HOST_ONLY=false

# Detect architecture for library paths
ARCH=$(dpkg --print-architecture 2>/dev/null || uname -m)
case "$ARCH" in
    arm64|aarch64) LIB_ARCH="aarch64-linux-gnu" ;;
    amd64|x86_64)  LIB_ARCH="x86_64-linux-gnu" ;;
    *)             LIB_ARCH="" ;;
esac
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROUDI_BIN="/opt/ros/${ROS_DISTRO}/bin/iox-roudi"
ROUDI_LD_PATH="/opt/ros/${ROS_DISTRO}/lib"
[ -n "$LIB_ARCH" ] && ROUDI_LD_PATH="${ROUDI_LD_PATH}:/opt/ros/${ROS_DISTRO}/lib/${LIB_ARCH}"

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
            echo "[2/6] Removed $SYSCTL_CONF and reloaded sysctl"
        else
            echo "[2/6] $SYSCTL_CONF not found, skipping"
        fi

        # Stop and remove RouDi systemd service
        if systemctl is-active --quiet iceoryx-roudi 2>/dev/null; then
            systemctl stop iceoryx-roudi
            echo "[3/6] Stopped RouDi service"
        else
            echo "[3/6] RouDi service not running, skipping"
        fi
        if [ -f "$ROUDI_SERVICE" ]; then
            systemctl disable iceoryx-roudi 2>/dev/null
            rm "$ROUDI_SERVICE"
            systemctl daemon-reload
            echo "[4/6] Removed RouDi systemd service"
        else
            echo "[4/6] RouDi service not found, skipping"
        fi

        # Remove RouDi config
        if [ -f "$ROUDI_CONFIG" ]; then
            rm "$ROUDI_CONFIG"
            echo "[5/6] Removed $ROUDI_CONFIG"
        else
            echo "[5/6] $ROUDI_CONFIG not found, skipping"
        fi

        # Clean stale iceoryx artifacts
        rm -f /tmp/roudi.lock /dev/shm/iceoryx_mgmt 2>/dev/null
        find /dev/shm -maxdepth 1 -name 'iceoryx*' -delete 2>/dev/null || true
        echo "[5.5/6] Cleaned stale iceoryx artifacts"

        CLEANED=false
        for rc in "${RC_FILES[@]}"; do
            if grep -q "$ENV_MARKER" "$rc" 2>/dev/null; then
                sed -i "/$ENV_MARKER/d" "$rc"
                sed -i '/export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp/d' "$rc"
                sed -i '/export CYCLONEDDS_URI=file:\/\//d' "$rc"
                echo "[6/6] Removed CycloneDDS env vars from $rc"
                CLEANED=true
            fi
        done
        if [ "$CLEANED" = false ]; then
            echo "[6/6] No CycloneDDS env vars found in rc files, skipping"
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
if ! dpkg -s "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" &>/dev/null; then
    echo "[0/4] Installing ros-${ROS_DISTRO}-rmw-cyclonedds-cpp..."
    apt-get update -qq && apt-get install -y -qq "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
else
    echo "[0/4] ros-${ROS_DISTRO}-rmw-cyclonedds-cpp already installed"
fi

# Install iceoryx for SHM support
echo "[0.5/4] Installing iceoryx for CycloneDDS SHM..."
apt-get update -qq && apt-get install -y -qq \
    "ros-${ROS_DISTRO}-iceoryx-posh" \
    "ros-${ROS_DISTRO}-iceoryx-hoofs" \
    "ros-${ROS_DISTRO}-iceoryx-binding-c" \
    && rm -rf /var/lib/apt/lists/*

# Rebuild CycloneDDS with SHM support (overrides apt version)
echo "[0.7/4] Building CycloneDDS with SHM support..."
cd /tmp && \
    git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git && \
    cd cyclonedds && mkdir build && cd build && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cmake .. \
        -DCMAKE_INSTALL_PREFIX=/opt/ros/${ROS_DISTRO} \
        -DCMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
        -DENABLE_SHM=ON \
        -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && \
    cd / && rm -rf /tmp/cyclonedds
echo "[0.7/4] CycloneDDS SHM build complete"

# ── Write iceoryx RouDi memory pool config ──
echo "[0.8/4] Writing RouDi config to $ROUDI_CONFIG"
mkdir -p "$(dirname "$ROUDI_CONFIG")"
cat > "$ROUDI_CONFIG" <<'TOML'
[general]
version = 1

[[segment]]

# Small messages (parameter events, tf, small services)
[[segment.mempool]]
size = 128
count = 10000

# Medium messages (sensor data, joint states, detections)
[[segment.mempool]]
size = 1024
count = 5000

# Larger messages (camera info, compressed data)
[[segment.mempool]]
size = 16384
count = 500

# Medium images (depth maps, smaller frames)
[[segment.mempool]]
size = 524288
count = 50

# Full camera frames (~2MB each at 960x540 BGR)
[[segment.mempool]]
size = 2097152
count = 30

# Large images (HD1080 full resolution ~6MB)
[[segment.mempool]]
size = 8388608
count = 10
TOML

# ── Clean stale iceoryx artifacts ──
echo "[0.9/4] Cleaning stale iceoryx artifacts..."
pkill -x iox-roudi 2>/dev/null || true
sleep 1
rm -f /tmp/roudi.lock 2>/dev/null
find /dev/shm -maxdepth 1 -name 'iceoryx*' -delete 2>/dev/null || true
rm -f /dev/shm/iceoryx_mgmt 2>/dev/null

# ── Create RouDi systemd service (host only) ──
if [ "$DOCKER_MODE" = false ]; then
    ACTUAL_USER="${SUDO_USER:-$USER}"
    echo "[0.95/4] Creating RouDi systemd service (runs as $ACTUAL_USER)..."
    cat > "$ROUDI_SERVICE" <<SVCEOF
[Unit]
Description=iceoryx RouDi daemon for CycloneDDS SHM
After=network.target

[Service]
Type=simple
User=${ACTUAL_USER}
Environment=LD_LIBRARY_PATH=${ROUDI_LD_PATH}
ExecStartPre=/bin/bash -c 'rm -f /tmp/roudi.lock; find /dev/shm -maxdepth 1 -name "iceoryx*" -delete 2>/dev/null; rm -f /dev/shm/iceoryx_mgmt 2>/dev/null; true'
ExecStart=${ROUDI_BIN} -c ${ROUDI_CONFIG}
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
SVCEOF

    systemctl daemon-reload
    systemctl enable iceoryx-roudi
    systemctl start iceoryx-roudi
    sleep 2

    if systemctl is-active --quiet iceoryx-roudi; then
        echo "[0.95/4] RouDi service started successfully"
    else
        echo "[WARN] RouDi service failed to start. Check: journalctl -u iceoryx-roudi"
    fi
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
    <SharedMemory>
      <Enable>true</Enable>
      <LogLevel>warn</LogLevel>
    </SharedMemory>
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
if [ -n "$INTERFACE" ]; then
    echo "CYCLONE_INTERFACE=$INTERFACE" > "$IFACE_ENV"
    echo "[INFO] Saved interface '$INTERFACE' to $IFACE_ENV"
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
echo "RouDi config:      $ROUDI_CONFIG"
if [ "$DOCKER_MODE" = false ]; then
    echo "Kernel tuning:     $SYSCTL_CONF"
    echo "RouDi service:     systemctl status iceoryx-roudi"
fi
echo ""
echo "To activate in current shell:"
echo "  source ~/$(basename "${RC_FILES[0]}")"
if [ "$DOCKER_MODE" = false ]; then
    echo ""
    echo "RouDi will auto-start on boot. To check:"
    echo "  systemctl status iceoryx-roudi"
    echo "  ls -la /dev/shm/iceoryx*"
fi
