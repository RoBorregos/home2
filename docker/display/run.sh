#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

COMPOSE="docker-compose.yaml"
parse_common_flags "$COMPOSE" "${ARGS[@]}"

#_________________________SETUP_________________________

setup_common_env "display" ".env"

add_or_update_variable .env "ENV_TYPE" "$ENV_TYPE"

if [ "$ENV_TYPE" != "cpu" ]; then
  add_or_update_variable .env "RUNTIME" "nvidia"
fi

#_________________________RUN_________________________

SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
IGNORE_PACKAGES="--packages-ignore frida_interfaces frida_constants xarm_msgs"
SOURCE_ROS="source /opt/ros/humble/setup.bash"
CYCLONE_SOURCE="source /usr/local/bin/cyclonedds_setup.sh"
PACKAGES="display"

# Map run.sh task flags to the PyQt display's task views (mirrors docker/hri/run.sh).
DISPLAY_TASK="${TASK#--}"
case "$DISPLAY_TASK" in
  "dlc")               DISPLAY_TASK="laundry" ;;
  "storing-groceries") DISPLAY_TASK="storing_groceries" ;;
  "finals")            DISPLAY_TASK="default" ;;
  "safety")            DISPLAY_TASK="ppc" ;;
  "")                  DISPLAY_TASK="default" ;;
esac

RUN="ros2 launch display display_launch.py task:=${DISPLAY_TASK}"

if [ "$BUILD" == "true" ]; then
    BUILD_COMMAND="colcon build $IGNORE_PACKAGES --symlink-install --packages-up-to $PACKAGES &&"
fi

COMMAND="$SOURCE_ROS && $SOURCE_INTERFACES && $CYCLONE_SOURCE && $BUILD_COMMAND source ~/.bashrc && $RUN"

if [ "$UPLOAD_IMAGE" == "true" ]; then
  echo "Uploading display image to DockerHub (env: ${ENV_TYPE})..."
  ensure_and_upload_image "roborregos/home2:display-${ENV_TYPE}" "$COMPOSE"
fi

add_or_update_variable .env "COMMAND" "$COMMAND"
docker compose -f "$COMPOSE" up $DETACHED $BUILD_IMAGE
