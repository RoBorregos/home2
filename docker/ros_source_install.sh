#!/usr/bin/env bash
# Install ROS 2 packages from source into the /opt/ros/$ROS_DISTRO overlay.
#
# Why this exists: Thor runs Ubuntu 24.04 (Noble) and our Humble is compiled from
# source (see Dockerfile.ROS-l4t-thor). There are NO `ros-humble-*` apt binaries
# for Noble, so anything the Orin images grabbed with `apt install ros-humble-<pkg>`
# (e.g. ros-humble-moveit*, ros-humble-zed-msgs) has to be built from source here.
#
# Give this script the same package names. It uses rosinstall_generator to resolve
# each package's full ROS dependency tree to source (skipping whatever is already
# installed in /opt/ros/humble, e.g. ros_base), clones it, pulls the remaining
# system deps with rosdep, and merge-installs everything into /opt/ros/humble so a
# plain `source /opt/ros/humble/setup.bash` picks it all up — exactly like apt would.
#
# Usage:   ros_source_install.sh <ros_pkg> [<ros_pkg> ...]
# Example: ros_source_install.sh moveit zed_msgs
#
# Env knobs:
#   ROS_DISTRO        ROS distro to target (default: humble)
#   ROSDEP_SKIP_KEYS  extra space-separated rosdep keys to skip
#   COLCON_JOBS       parallel colcon workers (default: all cores)
set -eo pipefail

: "${ROS_DISTRO:=humble}"
ROS_INSTALL_BASE="/opt/ros/${ROS_DISTRO}"

if [ "$#" -eq 0 ]; then
    echo "usage: $0 <ros_pkg> [<ros_pkg> ...]" >&2
    exit 1
fi

# rosinstall_generator ships with ros-dev-tools, but install it if missing.
command -v rosinstall_generator >/dev/null 2>&1 || \
    pip3 install --no-cache-dir --break-system-packages rosinstall-generator

WS="$(mktemp -d)"
mkdir -p "${WS}/src"

echo ">>> [ros_source_install] resolving source tree for: $*"
# --deps             : pull recursive ROS dependencies too
# --exclude-path ... : skip packages already present in /opt/ros/humble (ros_base),
#                      so we only build the genuinely missing ones
rosinstall_generator "$@" \
    --rosdistro "${ROS_DISTRO}" \
    --deps \
    --exclude-path "${ROS_INSTALL_BASE}" \
    --format repos > "${WS}/pkgs.repos"

echo ">>> [ros_source_install] importing sources"
vcs import "${WS}/src" < "${WS}/pkgs.repos"

# Source the base install so colcon finds the underlay (rclcpp, etc).
source "${ROS_INSTALL_BASE}/setup.bash"

echo ">>> [ros_source_install] resolving system dependencies"
apt-get update
rosdep init >/dev/null 2>&1 || true
rosdep update
# -r keeps going if a ROS key has no Noble apt rule (those are built from source
# above); ROSDEP_SKIP_KEYS lets callers skip anything that still trips it up.
rosdep install --from-paths "${WS}/src" --ignore-src -r -y \
    --rosdistro "${ROS_DISTRO}" \
    --skip-keys "${ROSDEP_SKIP_KEYS:-}" || true

echo ">>> [ros_source_install] building into ${ROS_INSTALL_BASE}"
cd "${WS}"
# CMake 4.x rejects the old cmake_minimum_required(<3.5) some packages still ship.
export CMAKE_POLICY_VERSION_MINIMUM=3.5
colcon build \
    --merge-install \
    --install-base "${ROS_INSTALL_BASE}" \
    --parallel-workers "${COLCON_JOBS:-$(nproc)}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

cd /
rm -rf "${WS}"
apt-get clean && rm -rf /var/lib/apt/lists/*
echo ">>> [ros_source_install] done: $*"
