#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

# IMPORTANT: Also edit auto-complete.sh to add new arguments
detached=""
build_display=""
open_display=""
download_model=""

# Set flags from arguments
for arg in "${ARGS[@]}"; do
  case "$arg" in
    -d)
      detached="-d"
      ;;
    --build-display)
      build_display="true"
      ;;
    --open-display)
      open_display="true"
      ;;
    --download-model)
      download_model="true"
      ;;
  esac
done

#_________________________SETUP_________________________

bash scripts/setup.bash
[ "$download_model" == "true" ] && bash ../../hri/packages/nlp/assets/download-model.sh

# Create dirs with current user to avoid permission problems
mkdir -p install build log ../../hri/packages/speech/assets/downloads/offline_voice/model/

# Ensure compose/.env exists
if [ ! -f "compose/.env" ]; then
  touch compose/.env
fi

# Check if display setup is needed
if [ ! -d "../../hri/display/dist" ] || [ ! -d "../../hri/display/node_modules" ] || [ ! -d "../../hri/display/web-ui/.next" ] || [ ! -d "../../hri/display/web-ui/node_modules" ] || [ "$build_display" == "true" ]; then
  echo "Setting up display environment..."

  compose_file="compose/display.yaml"
  service="display"
  [ "$ENV_TYPE" == "l4t" ] && compose_file="compose/display-l4t.yaml" && service="display-l4t"
  
  echo "Installing dependencies and building project inside temporary container..."
  docker compose -f "$compose_file" run --entrypoint "" "$service" bash -c "source /opt/ros/humble/setup.bash && npm run build"
fi

#_________________________RUN_________________________

PROFILES=()
RUN=""

case $TASK in
    "--receptionist")
        RUN="ros2 launch speech hri_launch.py"
        PROFILES=("receptionist")
        ;;
    "--carry")
        PROFILES=("carry")
        RUN="ros2 launch speech hri_launch.py"
        ;;
    "--storing")
        PROFILES=("storing")
        RUN="ros2 launch speech hri_launch.py"
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

COMPOSE_PROFILES=$(IFS=, ; echo "${PROFILES[*]}")
add_or_update_variable compose/.env "COMPOSE_PROFILES" "$COMPOSE_PROFILES"

GENERATE_BAML_CLIENT="baml-cli generate --from /workspace/src/task_manager/scripts/utils/baml_src/"
SOURCE_INTERFACES="source frida_interfaces_cache/install/local_setup.bash"
IGNORE_PACKAGES="--packages-ignore frida_interfaces frida_constants xarm_msgs"
SOURCE_ROS="source /opt/ros/humble/setup.bash"
COMMAND="$GENERATE_BAML_CLIENT && $SOURCE_ROS && $SOURCE_INTERFACES && colcon build $IGNORE_PACKAGES --symlink-install --packages-up-to speech nlp embeddings && source ~/.bashrc && $RUN"
add_or_update_variable compose/.env "COMMAND" "$COMMAND"

# Trap Ctrl+C/SIGTERM to clean up containers
sigint_count=0
cleanup() {
  sigint_count=$((sigint_count + 1))

  # Stop background display opener if running
  [ -n "$curl_pid" ] && kill "$curl_pid" 2>/dev/null || true

  if [ -n "$compose_file" ]; then
    if [ "$sigint_count" -eq 1 ]; then
      echo " Caught Ctrl+C — stopping containers (press Ctrl+C again to force kill)..."
      # Gracefully stop and remove containers started by this compose file
      ROLE=$PROFILES docker compose -f "$compose_file" stop 2>/dev/null || true
    else
      echo " Force killing containers..."
      ROLE=$PROFILES docker compose -f "$compose_file" kill 2>/dev/null || true
    fi
  fi

  # Ensure the compose client process (if any) is not left running
  [ -n "$compose_pid" ] && kill "$compose_pid" 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM

# Function to wait for service and launch display
wait_and_launch_display() {
  until curl --output /dev/null --silent --head --fail http://localhost:3000; do
    printf '.'
    sleep 1
  done
  chmod +x scripts/open-display.bash
  bash scripts/open-display.bash
}

compose_file="compose/docker-compose-${ENV_TYPE}.yml"

# Run the selected docker compose file
if [ -n "$detached" ]; then
  ROLE=$PROFILES docker compose -f "$compose_file" up -d
  [ "$open_display" == "true" ] && wait_and_launch_display
else
  ROLE=$PROFILES docker compose -f "$compose_file" up &
  compose_pid=$!

  if [ "$open_display" == "true" ]; then
    wait_and_launch_display &
    curl_pid=$!
  fi

  # Wait for docker compose to finish, then kill the curl loop if it exists
  wait $compose_pid
  if [ -n "$curl_pid" ] && kill -0 "$curl_pid" 2>/dev/null; then
    kill "$curl_pid" 2>/dev/null
  fi
fi
