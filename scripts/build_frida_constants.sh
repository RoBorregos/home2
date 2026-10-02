#!/usr/bin/env bash
# Build frida_constants with --symlink-install so editing a constant does not
# require a rebuild (#1162).
#
# Symlinks pointing at /workspace/build are then replaced with real copies:
# consumer containers (vision, navigation, integration) mount their own ./build
# there, not the cache's, so those links dangle and `source local_setup.bash`
# fails silently. It is 18 files of static build metadata — environment hooks,
# cmake config, ament_index, egg-info — none of which change with a constant.
#
# Symlinks pointing at /workspace/src are left live on purpose: the constants'
# .py files, the .hpp headers, package.xml and the map_areas/data JSONs. Every
# container mounts the repo at /workspace/src, so those resolve everywhere.
#
# Assumes ROS is already sourced and cwd is /workspace (the image WORKDIR);
# otherwise colcon fails first on its own.
set -euo pipefail

colcon build --packages-select frida_constants --symlink-install

while IFS= read -r link; do
  cp --remove-destination "$(readlink "$link")" "$link"
done < <(find install -type l -lname '/workspace/build/*')

# A leftover link makes the build unusable in consumers, so fail loudly.
if find install -type l -lname '/workspace/build/*' | grep -q .; then
  echo "ERROR: symlinks still pointing at /workspace/build" >&2
  exit 1
fi
