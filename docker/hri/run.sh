#!/bin/bash

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}

# Source image utilities
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${SCRIPT_DIR}/docker/image_utils.sh"

# Check for image management commands
PUSH_IMAGE=false
PULL_IMAGE=false
IMAGE_VERSION=""

# IMPORTANT: Also edit auto-complete.sh to add new arguments
detached=""
build_display=""
open_display=""
download_model=""

# Check if one of the arguments is --detached, --build-display, --open-display, --push-image, --pull-image
for arg in "${ARGS[@]}"; do
  if [ "$arg" == "-d" ]; then
    detached="-d"
  elif [ "$arg" == "--build-display" ]; then
    build_display="true"
  elif [ "$arg" == "--open-display" ]; then
    open_display="true"
  elif [ "$arg" == "--download-model" ]; then
    download_model="true"
  elif [ "$arg" == "--push-image" ]; then
    PUSH_IMAGE=true
  elif [ "$arg" == "--pull-image" ]; then
    PULL_IMAGE=true
  fi
done

# Get version from arguments
for i in "${!ARGS[@]}"; do
  if [ "${ARGS[$i]}" == "--push-image" ] || [ "${ARGS[$i]}" == "--pull-image" ]; then
    IMAGE_VERSION="${ARGS[$((i+1))]}"
    break
  fi
done

# Set default version
if [ "$PUSH_IMAGE" = true ] && [ -z "$IMAGE_VERSION" ]; then
  IMAGE_VERSION=$(date +%Y%m%d)
elif [ "$PULL_IMAGE" = true ] && [ -z "$IMAGE_VERSION" ]; then
  IMAGE_VERSION="latest"
fi

#_________________________BUILD_________________________

# Function to add or update a variable in a file
add_or_update_variable() {
    local file=$1
    local variable=$2
    local value=$3

    local escaped_value
    escaped_value=$(printf '%s\n' "$value" | sed -e 's/[&/\]/\\&/g')

    if grep -q "^${variable}=" "$file"; then
        sed -i "s|^${variable}=.*|${variable}=${escaped_value}|" "$file"
    else
        echo "${variable}=${value}" >> "$file"
    fi
}

# Check type of environment (cpu, cuda, or l4t), default cpu
ENV_TYPE="cpu"

# Check device type
if [ -f /etc/nv_tegra_release ]; then
  ENV_TYPE="l4t"
elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
  ENV_TYPE="cuda"
fi
echo "Detected environment: $ENV_TYPE"

# Handle push image command
if [ "$PUSH_IMAGE" = true ]; then
  ENV_SUFFIX="$ENV_TYPE"
  push_area_image "hri" "$ENV_SUFFIX" "$IMAGE_VERSION" "$SCRIPT_DIR"
  exit $?
fi

# Handle pull image command
if [ "$PULL_IMAGE" = true ]; then
  ENV_SUFFIX="$ENV_TYPE"
  pull_area_image "hri" "$ENV_SUFFIX" "$IMAGE_VERSION"
  exit $?
fi

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
COMMAND="$GENERATE_BAML_CLIENT && source /opt/ros/humble/setup.bash && $SOURCE_INTERFACES && colcon build $IGNORE_PACKAGES --symlink-install --packages-up-to speech nlp embeddings && source ~/.bashrc && $RUN"
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
