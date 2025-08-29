#!/bin/bash

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}

detached=""
# check if one of the arguments is --detached
for arg in "${ARGS[@]}"; do
  if [ "$arg" == "-d" ]; then
    detached="-d"
  fi
done

#_________________________BUILD_________________________

# Image names
CPU_IMAGE="roborregos/home2:cpu_base"
CUDA_IMAGE="roborregos/home2:cuda_base"
JETSON_IMAGE="roborregos/home2:l4t_base"

# Function to check if an image exists
check_image_exists() {
    local image_name=$1
    if ! docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "^${image_name}$"; then
        echo "Image $image_name does not exist. Building it..."
        return 1  # Image doesn't exist
    else
        echo "Image $image_name already exists. Skipping build."
        return 0  # Image exists
    fi
}

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

# Check type of environment (CPU, GPU, or Jetson), default CPU
ENV_TYPE="cpu"

# Check device type
if [[ -f /etc/nv_tegra_release ]]; then
    ENV_TYPE="jetson"
else
    # Check if NVIDIA GPUs are available
    if command -v nvidia-smi > /dev/null 2>&1; then
        if nvidia-smi > /dev/null 2>&1; then
            ENV_TYPE="gpu"
        fi
    fi
fi
echo "Detected environment: $ENV_TYPE"

# Build base image

case $ENV_TYPE in
  "gpu")
    ;&
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

# TODO: Uncomment this if you need to download the model
# bash ../../hri/packages/nlp/assets/download-model.sh

# Create dirs with current user to avoid permission problems
mkdir -p install build log ../../hri/packages/speech/assets/downloads/offline_voice/model/


# Check if display setup is needed
if [ ! -d "../../hri/display/dist" ] || [ ! -d "../../hri/display/node_modules" ] || [ ! -d "../../hri/display/web-ui/.next" ] || [ ! -d "../../hri/display/web-ui/node_modules" ]; then
  echo "Setting up display environment..."

  compose_file="display.yaml"
  [ "$ENV_TYPE" == "jetson" ] && compose_file="display-l4t.yaml"
  
  echo "Installing dependencies and building project inside temporary container..."
  docker compose -f "$compose_file" run --entrypoint "" display bash -c "source /opt/ros/humble/setup.bash && npm run build"
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
add_or_update_variable .env "COMPOSE_PROFILES" "$COMPOSE_PROFILES"

GENERATE_BAML_CLIENT="baml-cli generate --from /workspace/src/task_manager/scripts/utils/baml_src/"
SOURCE_INTERFACES="source frida_interfaces_cache/install/local_setup.bash"
IGNORE_PACKAGES="--packages-ignore frida_interfaces frida_constants xarm_msgs"
COMMAND="$GENERATE_BAML_CLIENT && source /opt/ros/humble/setup.bash && $SOURCE_INTERFACES && colcon build $IGNORE_PACKAGES --symlink-install --packages-up-to speech nlp embeddings && source ~/.bashrc && $RUN"

# echo "COMMAND= $COMMAND " >> .env
add_or_update_variable .env "COMMAND" "$COMMAND"

# Trap Ctrl+C to clean up
cleanup() {
  echo -e "\n🛑 Interrupted. Cleaning up..."
  [ -n "$compose_pid" ] && kill "$compose_pid" 2>/dev/null
  [ -n "$curl_pid" ] && kill "$curl_pid" 2>/dev/null
  exit 1
}
trap cleanup SIGINT

compose_file="docker-compose-cpu.yml"
[ "$ENV_TYPE" == "gpu" ] && compose_file="docker-compose-gpu.yml"
[ "$ENV_TYPE" == "jetson" ] && compose_file="docker-compose.yml"

if [ -n "$detached" ]; then
  echo "🚀 Launching containers in detached mode..."
  docker compose -f "$compose_file" up -d

  echo "⏳ Waiting for localhost:3000 to be available..."
  until curl --output /dev/null --silent --head --fail http://localhost:3000; do
    printf '.'
    sleep 1
  done
  echo -e "\n✅ Web service is up. Launching Firefox..."
  bash open-display.bash

else
  echo "🚀 Launching containers in attached mode..."
  ROLE=$PROFILES docker compose -f "$compose_file" up &
  compose_pid=$!

  echo "⏳ Waiting for localhost:3000 to be available..."
  (
    while ! curl --output /dev/null --silent --head --fail http://localhost:3000; do
      printf '.'
      sleep 1
    done
    echo -e "\n✅ Web service is up. Launching Firefox..."
    bash open-display.bash
  ) &
  curl_pid=$!

  # Wait for docker compose to finish, then kill the curl loop
  wait $compose_pid
  kill $curl_pid 2>/dev/null
fi
