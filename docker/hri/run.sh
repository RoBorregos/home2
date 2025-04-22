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
bash ../../hri/packages/nlp/assets/download-model.sh

# Directories to persist hri ros2 build. Create here to avoid permission issues
mkdir install build log 

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
    "--storing-groceries")
        PROFILES=("storing-groceries")
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

COMMAND="source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select frida_interfaces frida_constants speech nlp embeddings && source ~/.bashrc && $RUN"

# echo "COMMAND= $COMMAND " >> .env
add_or_update_variable .env "COMMAND" "$COMMAND"

# Check if the container exists
if [ $ENV_TYPE == "cpu" ]; then
        docker compose -f docker-compose-cpu.yml up $detached
    elif [ $ENV_TYPE == "gpu" ]; then
        docker compose -f docker-compose-cpu.yml up $detached
    elif [ $ENV_TYPE == "jetson" ]; then
        docker compose up $detached
fi
