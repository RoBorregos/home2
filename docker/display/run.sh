#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

BUILD_DISPLAY=""
OPEN_DISPLAY=""

COMPOSE="docker-compose.yaml"
parse_common_flags "$COMPOSE" "${ARGS[@]}"

# Parse display-specific flags
for arg in "${ARGS[@]}"; do
  case "$arg" in
    "--build-display") BUILD_DISPLAY="true" ;;
    "--open-display")  OPEN_DISPLAY="true" ;;
  esac
done

#_________________________SETUP_________________________

setup_common_env "display" ".env"

add_or_update_variable .env "ENV_TYPE" "$ENV_TYPE"

if [ "$ENV_TYPE" != "cpu" ]; then
  add_or_update_variable .env "RUNTIME" "nvidia"
fi

# Install deps + build Next.js bundle if missing (or forced)
DISPLAY_DIR="../../hri/packages/display/display"
if [ ! -d "$DISPLAY_DIR/node_modules" ] || [ ! -d "$DISPLAY_DIR/.next" ] || [ "$BUILD_DISPLAY" == "true" ]; then
  echo "Installing dependencies and building display inside temporary container..."
  docker compose -f "$COMPOSE" run $BUILD_IMAGE --rm --entrypoint "" display-ros bash -c "cd /workspace/src/hri/packages/display/display && npm i && npm run build"
fi

#_________________________RUN_________________________

SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
IGNORE_PACKAGES="--packages-ignore frida_interfaces frida_constants xarm_msgs"
SOURCE_ROS="source /opt/ros/humble/setup.bash"
CYCLONE_SOURCE="source /usr/local/bin/cyclonedds_setup.sh"
PACKAGES="display"
RUN="ros2 launch display display_launch.py"

if [ "$BUILD" == "true" ]; then
    BUILD_COMMAND="colcon build $IGNORE_PACKAGES --symlink-install --packages-up-to $PACKAGES &&"
fi

COMMAND="$SOURCE_ROS && $SOURCE_INTERFACES && $CYCLONE_SOURCE && $BUILD_COMMAND source ~/.bashrc && $RUN"

cleanup() {
  [ -n "$wait_for_display_pid" ] && kill "$wait_for_display_pid" 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM

wait_and_launch_display() {
  until curl --output /dev/null --silent --head --fail http://localhost:3000; do
    sleep 1
  done
  chmod +x open-display.bash
  local task_route="${TASK#--}"
  bash open-display.bash "$task_route"
}

if [ -n "$OPEN_DISPLAY" ]; then
  wait_and_launch_display &
  wait_for_display_pid=$!
fi

if [ "$UPLOAD_IMAGE" == "true" ]; then
  echo "Uploading display image to DockerHub (env: ${ENV_TYPE})..."
  ensure_and_upload_image "roborregos/home2:display-${ENV_TYPE}" "$COMPOSE"
fi

add_or_update_variable .env "COMMAND" "$COMMAND"
docker compose -f "$COMPOSE" up $DETACHED $BUILD_IMAGE
