#!/usr/bin/env bash
# Install ROS 2 packages from source into the /opt/ros/$ROS_DISTRO overlay.
#
# Thor runs Ubuntu 24.04 (Noble) and our Humble is compiled from source, so there
# are NO `ros-humble-*` apt binaries. Anything the Orin images grabbed with
# `apt install ros-humble-<pkg>` (moveit, zed_msgs, ros2 controllers, ...) is built
# from source here and merge-installed into /opt/ros/humble, so a plain
# `source /opt/ros/humble/setup.bash` picks it up exactly like apt would.
#
# It resolves each package's recursive ROS deps, then builds ONLY the packages not
# already in /opt/ros/humble — everything else is satisfied from the existing
# overlay at build time. So topping up (e.g. one controller) is cheap, and adding a
# new subtree (e.g. moveit) compiles just the genuinely-new packages.
#
# Usage:   ros_source_install.sh <ros_pkg> [<ros_pkg> ...]
# Examples:
#   ros_source_install.sh moveit zed_msgs
#   ros_source_install.sh joint_trajectory_controller joint_state_broadcaster tf_transformations
# At runtime in a container, the `ros_install` alias wraps this with sudo.
#
# Env knobs:
#   ROS_DISTRO        ROS distro (default: humble)
#   ROSDEP_SKIP_KEYS  extra space-separated rosdep keys to skip
#   COLCON_JOBS       parallel colcon workers (default: all cores)
set -eo pipefail

: "${ROS_DISTRO:=humble}"
ROS_INSTALL_BASE="/opt/ros/${ROS_DISTRO}"

if [ "$#" -eq 0 ]; then
    echo "usage: $0 <ros_pkg> [<ros_pkg> ...]" >&2
    exit 1
fi

command -v rosinstall_generator >/dev/null 2>&1 || \
    pip3 install --no-cache-dir --break-system-packages rosinstall-generator

# Source the overlay first so we can (a) see what's already installed and
# (b) let colcon resolve existing deps from it at build time.
source "${ROS_INSTALL_BASE}/setup.bash"
INSTALLED="$(ros2 pkg list 2>/dev/null | tr '\n' ' ' || true)"

WS="$(mktemp -d)"
mkdir -p "${WS}/src"

echo ">>> [ros_source_install] resolving dependency tree for: $*"
rosinstall_generator "$@" \
    --rosdistro "${ROS_DISTRO}" \
    --deps \
    --format repos > "${WS}/full.repos"

# rosinstall_generator --deps pulls the FULL tree (incl. ros_base), and its own
# --exclude/--exclude-path do NOT reliably prune that tree against a real install.
# So drop already-installed packages from the .repos here, by package name (the
# basename of each repo key). This is what stops us from recloning + recompiling
# all of ros_base on every call — only genuinely-missing packages remain.
awk -v inst="${INSTALLED}" '
    BEGIN { n = split(inst, a, " "); for (i = 1; i <= n; i++) skip[a[i]] = 1; keep = 1 }
    /^repositories:[[:space:]]*$/ { print; next }
    /^[[:space:]][[:space:]][^[:space:]].*:[[:space:]]*$/ {
        key = $0; sub(/:[[:space:]]*$/, "", key); sub(/^[[:space:]]+/, "", key)
        base = key; sub(/.*\//, "", base)
        keep = (base in skip) ? 0 : 1
        if (keep) print
        next
    }
    { if (keep) print }
' "${WS}/full.repos" > "${WS}/pkgs.repos"

if ! grep -qE '^[[:space:]][[:space:]][^[:space:]].*:[[:space:]]*$' "${WS}/pkgs.repos"; then
    echo ">>> [ros_source_install] nothing to build — all requested packages already in ${ROS_INSTALL_BASE}"
    rm -rf "${WS}"
    exit 0
fi

echo ">>> [ros_source_install] will build:"
grep -E '^[[:space:]][[:space:]][^[:space:]].*:[[:space:]]*$' "${WS}/pkgs.repos" \
    | sed -E 's/^[[:space:]]+//; s/:[[:space:]]*$//; s#.*/##' | sort -u | sed 's/^/      /'

vcs import "${WS}/src" < "${WS}/pkgs.repos"

echo ">>> [ros_source_install] resolving system dependencies"
apt-get update
rosdep init >/dev/null 2>&1 || true
rosdep update
# -r keeps going if a ROS key has no Noble apt rule (those are built from source);
# ROSDEP_SKIP_KEYS lets callers skip anything that still trips it up.
rosdep install --from-paths "${WS}/src" --ignore-src -r -y \
    --rosdistro "${ROS_DISTRO}" \
    --skip-keys "${ROSDEP_SKIP_KEYS:-}" || true

# Make older ROS deps build under GCC 13 / Boost 1.83 (Noble). A thin compiler
# wrapper strips -Werror* (so deprecation *warnings* stay warnings) and defines
# BOOST_ALLOW_DEPRECATED_HEADERS / BOOST_TIMER_ENABLE_DEPRECATED (so Boost-1.83
# #error-guarded headers still compile). The wrapper wins regardless of where a
# package injected the flag.
EXTRA_CFLAGS="-Wno-error -DBOOST_ALLOW_DEPRECATED_HEADERS -DBOOST_TIMER_ENABLE_DEPRECATED"
NOWERR_DIR="$(mktemp -d)"
REAL_CC="$(command -v gcc)"
REAL_CXX="$(command -v g++)"
for pair in "cc:${REAL_CC}" "c++:${REAL_CXX}"; do
    name="${pair%%:*}"; real="${pair#*:}"
    cat > "${NOWERR_DIR}/${name}" <<EOF
#!/usr/bin/env bash
args=()
for a in "\$@"; do
  case "\$a" in
    -Werror|-Werror=*) continue ;;
    *) args+=("\$a") ;;
  esac
done
exec ${real} "\${args[@]}" ${EXTRA_CFLAGS}
EOF
    chmod +x "${NOWERR_DIR}/${name}"
done

echo ">>> [ros_source_install] building into ${ROS_INSTALL_BASE}"
cd "${WS}"
# CMake 4.x rejects the old cmake_minimum_required(<3.5) some packages still ship.
export CMAKE_POLICY_VERSION_MINIMUM=3.5
export CC="${NOWERR_DIR}/cc" CXX="${NOWERR_DIR}/c++"
colcon build \
    --merge-install \
    --install-base "${ROS_INSTALL_BASE}" \
    --parallel-workers "${COLCON_JOBS:-$(nproc)}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -DCMAKE_C_COMPILER="${NOWERR_DIR}/cc" -DCMAKE_CXX_COMPILER="${NOWERR_DIR}/c++"

cd /
rm -rf "${WS}" "${NOWERR_DIR}"
apt-get clean && rm -rf /var/lib/apt/lists/*
echo ">>> [ros_source_install] done: $*"
