#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

# IMPORTANT: Also edit auto-complete.sh to add new arguments
DETACHED=""
BUILD=""
BUILD_IMAGE=""
BUILD_DISPLAY=""
OPEN_DISPLAY=""
DOWNLOAD_MODEL=""

COMPOSE="compose/docker-compose-${ENV_TYPE}.yml"

# Set flags from arguments
for arg in "${ARGS[@]}"; do
  case "$arg" in
    "-d")
        DETACHED="-d"
        ;;
    "--build")
        BUILD="true"
        ;;
    "--recreate")
        docker compose -f "$COMPOSE" down
        ;;
    "--down")
        docker compose -f "$COMPOSE" down
        exit 0
        ;;
    "--stop")
        docker compose -f "$COMPOSE" stop
        exit 0
        ;;
    "--build-image")
        BUILD_IMAGE="--build"
        ;;
    "--build-display")
        BUILD_DISPLAY="true"
        ;;
    "--open-display")
        OPEN_DISPLAY="true"
        ;;
    "--download-model")
        DOWNLOAD_MODEL="true"
        ;;
  esac
done

#_________________________SETUP_________________________

# Read .env if it exists and export variables
[ -f .env ] && { set -a; source .env; set +a; }

# Run setup if .env doesn't indicate it has been done
if [ -z "${SETUP_DONE:-}" ]; then
  echo "Running setup..."
  if bash scripts/setup.bash; then
    export SETUP_DONE=true
    add_or_update_variable .env "SETUP_DONE" "true"
  else
    echo "Error: setup failed" >&2
    exit 1
  fi
else
  export SETUP_DONE=true
fi

[ "$DOWNLOAD_MODEL" == "true" ] && bash ../../hri/packages/nlp/assets/download-model.sh

# Create dirs with current user to avoid permission problems
mkdir -p install build log \
  ../../hri/packages/speech/assets/downloads/offline_voice/model/ \
  ../../hri/packages/speech/assets/downloads/offline_voice/audios/

# Reset .env
echo "" > compose/.env

# Export user
add_or_update_variable compose/.env "LOCAL_USER_ID" "$(id -u)"
add_or_update_variable compose/.env "LOCAL_GROUP_ID" "$(id -g)"

# Set environment type and runtime
add_or_update_variable compose/.env "ENV_TYPE" "$ENV_TYPE"
if [ "$ENV_TYPE" != "cpu" ]; then
  add_or_update_variable compose/.env "RUNTIME" "nvidia"
fi

# If setup was done before persist it again now that .env has been reset
if [ "${SETUP_DONE:-}" = "true" ]; then
  add_or_update_variable .env "SETUP_DONE" "true"
fi

# Check if display setup is needed
if [ ! -d "../../hri/display/dist" ] || [ ! -d "../../hri/display/node_modules" ] || [ ! -d "../../hri/display/web-ui/.next" ] || [ ! -d "../../hri/display/web-ui/node_modules" ] || [ "$BUILD_DISPLAY" == "true" ]; then
  echo "Installing dependencies and building project inside temporary container..."
  docker compose -f compose/display.yaml run $BUILD_IMAGE --rm --entrypoint "" display bash -c "source /opt/ros/humble/setup.bash && npm run build"
fi

#_________________________RUN_________________________

GENERATE_BAML_CLIENT="baml-cli generate --from /workspace/src/task_manager/scripts/utils/baml_src/"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
IGNORE_PACKAGES="--packages-ignore frida_interfaces frida_constants xarm_msgs"
SOURCE_ROS="source /opt/ros/humble/setup.bash"
PACKAGES="speech nlp embeddings"
PROFILES=()
RUN=""

case $TASK in
  "--receptionist")
    RUN="ros2 launch speech hri_launch.py"
    PROFILES=("receptionist")
    ;;
  "--storing-groceries")
    PROFILES=("storing")
    RUN="ros2 launch speech hri_launch.py"
    ;;
  "--gpsr")
    PROFILES=("gpsr")
    RUN="ros2 launch speech hri_launch.py"
    ;;
  *)
    PROFILES=("*")
    RUN="bash"
    ;;
esac

if [ "$BUILD" == "true" ]; then
    BUILD_COMMAND="colcon build $IGNORE_PACKAGES --symlink-install --packages-up-to $PACKAGES &&"
fi

COMPOSE_PROFILES=$(IFS=, ; echo "${PROFILES[*]}")
add_or_update_variable compose/.env "COMPOSE_PROFILES" "$COMPOSE_PROFILES"

COMMAND="$GENERATE_BAML_CLIENT && $SOURCE_ROS && $SOURCE_INTERFACES && $BUILD_COMMAND source ~/.bashrc && $RUN"
add_or_update_variable compose/.env "COMMAND" "$COMMAND"
add_or_update_variable compose/.env "ROLE" "${PROFILES[0]}"

cleanup() {
  # Ensure the process is not left running
  [ -n "$wait_for_display_pid" ] && kill "$wait_for_display_pid" 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM

# Function to wait for service and launch display
wait_and_launch_display() {
  until curl --output /dev/null --silent --head --fail http://localhost:3000; do
    sleep 1
  done
  chmod +x scripts/open-display.bash
  bash scripts/open-display.bash
}

if [ -n "$OPEN_DISPLAY" ]; then
  wait_and_launch_display &
  wait_for_display_pid=$!
fi

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    docker compose -f "$COMPOSE" up -d $BUILD_IMAGE
    docker compose -f "$COMPOSE" exec hri-ros bash -c "$COMMAND"
else
    docker compose -f "$COMPOSE" up $DETACHED $BUILD_IMAGE
fi
