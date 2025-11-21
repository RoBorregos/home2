#!/bin/bash

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}



# IMPORTANT: Also edit auto-complete.sh to add new arguments
detached=""
build_display=""
open_display=""
download_model=""
display_mode=""
DISPLAY_TASK=""

# Check if one of the arguments is --detached, --build-display, or --open-display
for arg in "${ARGS[@]}"; do
    case "$arg" in
        -d) detached="-d" ;;
        --build-display) build_display="true" ;;
        --open-display) open_display="true" ;;
        --download-model) download_model="true" ;;
        --display-gpsr|--gpsr-display|--open-gpsr-display)
            DISPLAY_TASK="GPSR"
            display_mode="gpsr"
            open_display="true"
            ;;
        --display-store-groceries|--groceries-display|--open-groceries-display|--display-store)
            DISPLAY_TASK="StoreGroceries"
            display_mode="groceries"
            open_display="true"
            ;;
        --display=*)
            DISPLAY_TASK="${arg#--display=}"
            display_mode="$(echo "${DISPLAY_TASK}" | tr '[:upper:]' '[:lower:]')"
            open_display="true"
            ;;
    esac
done

#_________________________BUILD_________________________

CPU_IMAGE="roborregos/home2:cpu_base"
CUDA_IMAGE="roborregos/home2:cuda_base"
JETSON_IMAGE="roborregos/home2:l4t_base"

check_image_exists() {
    local image_name=$1
    if ! docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "^${image_name}$"; then
        echo "Image $image_name does not exist. Building it..."
        return 1
    else
        echo "Image $image_name already exists. Skipping build."
        return 0
    fi
}

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

ENV_TYPE="cpu"

if [[ -f /etc/nv_tegra_release ]]; then
    ENV_TYPE="jetson"
else
    if command -v nvidia-smi > /dev/null 2>&1; then
        if nvidia-smi > /dev/null 2>&1; then
            ENV_TYPE="gpu"
        fi
    fi
fi
echo "Detected environment: $ENV_TYPE"

case $ENV_TYPE in
  "gpu") ;&
  "cpu")
    check_image_exists "$CPU_IMAGE"
    if [ $? -eq 1 ]; then
        docker compose -f ../cpu.yaml build
    fi
    ;;
  "jetson")
    check_image_exists "$JETSON_IMAGE"
    if [ $? -eq 1 ]; then
        docker compose -f ../l4t.yaml build
    fi
    ;;
  *)
    echo "Unknown environment type!"
    exit 1
    ;;
esac

#_________________________SETUP_________________________

bash setup.bash
[ "$download_model" == "true" ] && bash ../../hri/packages/nlp/assets/download-model.sh

mkdir -p install build log ../../hri/packages/speech/assets/downloads/offline_voice/model/

if [ ! -d "../../hri/display/dist" ] || [ ! -d "../../hri/display/node_modules" ] || [ ! -d "../../hri/display/web-ui/.next" ] || [ ! -d "../../hri/display/web-ui/node_modules" ] || [ "$build_display" == "true" ]; then
  echo "Setting up display environment..."

  compose_file="display.yaml"
  [ "$ENV_TYPE" == "jetson" ] && compose_file="display-l4t.yaml"

  echo "Installing dependencies and building project inside temporary container..."

  docker compose -f "$compose_file" run --rm --entrypoint "" display bash -c "cd web-ui && source /opt/ros/humble/setup.bash && npm ci --silent && npm run build"
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
        DISPLAY_TASK="StoreGroceries"
        display_mode="groceries"
        open_display="true"
        ;;
    "--gpsr")
        PROFILES=("gpsr")
        RUN="ros2 launch speech hri_launch.py"
        DISPLAY_TASK="GPSR"
        display_mode="gpsr"
        open_display="true"
        ;;
    *)
        PROFILES=("*")
        RUN="bash"
        ;;
esac

COMPOSE_PROFILES=$(IFS=, ; echo "${PROFILES[*]}")
add_or_update_variable .env "COMPOSE_PROFILES" "$COMPOSE_PROFILES"

# Function to wait for frontend and open display
wait_and_launch_display() {
    until curl --output /dev/null --silent --head --fail http://localhost:3000; do sleep 1; done
    chmod +x scripts/open-display.bash
    # Open display passing task as query param
    bash scripts/open-display.bash "$DISPLAY_TASK"
}

GENERATE_BAML_CLIENT="baml-cli generate --from /workspace/src/task_manager/scripts/utils/baml_src/"
SOURCE_INTERFACES="source frida_interfaces_cache/install/local_setup.bash"
IGNORE_PACKAGES="--packages-ignore frida_interfaces frida_constants xarm_msgs"
COMMAND="$GENERATE_BAML_CLIENT && source /opt/ros/humble/setup.bash && $SOURCE_INTERFACES && colcon build $IGNORE_PACKAGES --symlink-install --packages-up-to speech nlp embeddings && source ~/.bashrc && $RUN"

add_or_update_variable .env "COMMAND" "$COMMAND"

cleanup() {
  [ -n "$compose_pid" ] && kill "$compose_pid" 2>/dev/null
  [ -n "$curl_pid" ] && kill "$curl_pid" 2>/dev/null
  exit 1
}
trap cleanup SIGINT


compose_file="docker-compose-cpu.yml"
[ "$ENV_TYPE" == "gpu" ] && compose_file="docker-compose-gpu.yml"
[ "$ENV_TYPE" == "jetson" ] && compose_file="docker-compose.yml"

if [ "$display_mode" == "gpsr" ]; then
  echo "Launching GPSR display..."
  export DISPLAY_MODE="gpsr"
elif [ "$display_mode" == "groceries" ]; then
  echo "Launching Groceries display..."
  export DISPLAY_MODE="groceries"
fi

# Run the selected docker compose file
if [ -n "$detached" ]; then
  docker compose -f "$compose_file" up -d
  [ "$open_display" == "true" ] && wait_and_launch_display
else
  ROLE=$PROFILES docker compose -f "$compose_file" up &
  compose_pid=$!

  if [ "$open_display" == "true" ]; then
    wait_and_launch_display &
    curl_pid=$!
  fi

  wait $compose_pid
  [ -n "$curl_pid" ] && kill $curl_pid 2>/dev/null
fi
