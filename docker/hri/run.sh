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

#_________________________RUN_________________________

SERVICE_NAME="home2-hri-cpu-devices"  # Change this to your service name in docker-compose.yml

if [ $ENV_TYPE == "jetson" ]; then
    SERVICE_NAME="home2-hri-l4t-devices"
fi


# Check if the container exists
EXISTING_CONTAINER=$(docker ps -q -f name=$SERVICE_NAME)
if [ -z "$EXISTING_CONTAINER" ]; then
    echo "No container with the name $SERVICE_NAME exists. Building and starting the container now..."
    if [ $ENV_TYPE == "cpu" ]; then
        docker compose -f docker-compose-cpu.yml up -d
    elif [ $ENV_TYPE == "gpu" ]; then
        docker compose -f docker-compose-cpu.yml up -d
    elif [ $ENV_TYPE == "jetson" ]; then
        docker compose up -d
    fi
fi


# TODO: implement task option
# case $TASK in
#     "--moondream")
#         RUN=""
#         MOONDREAM=true
#         ;;
#     "--receptionist")
#         RUN="ros2 launch vision_general receptionist_launch.py"
#         MOONDREAM=true
#         ;;

#     *)
#         RUN=""
#         ;;
# esac


# check if TASK is not empty
if [ -z "$TASK" ]; then
    docker exec -it $SERVICE_NAME /bin/bash
fi
