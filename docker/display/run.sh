#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

DISPLAY_BACKUP=""

COMPOSE="docker-compose.yaml"
parse_common_flags "$COMPOSE" "${ARGS[@]}"

# Parse display-specific flags
for arg in "${ARGS[@]}"; do
  case "$arg" in
    # Fall back to the legacy Next.js display instead of the default PyQt UI.
    "--backup") DISPLAY_BACKUP="true" ;;
  esac
done

#_________________________SETUP_________________________

setup_common_env "display" ".env"

add_or_update_variable .env "ENV_TYPE" "$ENV_TYPE"

if [ "$ENV_TYPE" != "cpu" ]; then
  add_or_update_variable .env "RUNTIME" "nvidia"
fi

#_________________________RUN_________________________

SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
IGNORE_PACKAGES="--packages-ignore frida_interfaces frida_constants xarm_msgs"
SOURCE_ROS="source /opt/ros/jazzy/setup.bash"
CYCLONE_SOURCE="source /usr/local/bin/cyclonedds_setup.sh"
PACKAGES="display"

# Map run.sh task flags to the PyQt display's task views (mirrors docker/hri/run.sh).
DISPLAY_TASK="${TASK#--}"
case "$DISPLAY_TASK" in
  "dlc")               DISPLAY_TASK="laundry" ;;
  "storing-groceries") DISPLAY_TASK="storing_groceries" ;;
  "finals")            DISPLAY_TASK="default" ;;
  "safety")            DISPLAY_TASK="ppc" ;;
  "--backup"|"")        DISPLAY_TASK="default" ;;
esac

if [ "$BUILD" == "true" ]; then
    BUILD_COMMAND="colcon build $IGNORE_PACKAGES --symlink-install --packages-up-to $PACKAGES &&"
fi

if [ "$DISPLAY_BACKUP" == "true" ]; then
  # Legacy Next.js display: install deps + build bundle if missing, then serve
  # it behind rosbridge/web_video_server and open it in a kiosk browser window.
  RUN="ros2 launch display display_launch_backup.py"

  DISPLAY_DIR="../../hri/packages/display/display"
  if [ ! -d "$DISPLAY_DIR/node_modules" ] || [ ! -d "$DISPLAY_DIR/.next" ]; then
    echo "Installing dependencies and building the legacy display inside a temporary container..."
    docker compose -f "$COMPOSE" run $BUILD_IMAGE --rm --entrypoint "" display-ros bash -c "cd /workspace/src/hri/packages/display/display && npm i && npm run build"
  fi

  cleanup() {
    [ -n "$wait_for_display_pid" ] && kill "$wait_for_display_pid" 2>/dev/null || true
  }
  trap cleanup SIGINT SIGTERM

  wait_and_launch_display() {
    until curl --output /dev/null --silent --head --fail http://localhost:3000; do
      sleep 1
    done
    chmod +x open-display.bash
    # The safety routine reuses the HRIC display page.
    local task_route="$DISPLAY_TASK"
    [ "$TASK" = "--safety" ] && task_route="hric"
    [ "$task_route" = "default" ] && task_route=""
    bash open-display.bash "$task_route"
  }

  wait_and_launch_display &
  wait_for_display_pid=$!
else
  RUN="ros2 launch display display_launch.py task:=${DISPLAY_TASK}"
fi

COMMAND="$SOURCE_ROS && $SOURCE_INTERFACES && $CYCLONE_SOURCE && $BUILD_COMMAND source ~/.bashrc && $RUN"

if [ "$UPLOAD_IMAGE" == "true" ]; then
  echo "Uploading display image to DockerHub (env: ${ENV_TYPE})..."
  ensure_and_upload_image "roborregos/home2:display-${ENV_TYPE}" "$COMPOSE"
fi

add_or_update_variable .env "COMMAND" "$COMMAND"
docker compose -f "$COMPOSE" up $DETACHED $BUILD_IMAGE
