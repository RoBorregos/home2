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
> .env

# Write environment variables to .env file for Docker Compose and build base images
case $ENV_TYPE in
  "cpu")
    #_____CPU_____
    echo "DOCKERFILE=docker/vision/Dockerfile.cpu" >> .env
    echo "BASE_IMAGE=roborregos/home2:cpu_base" >> .env
    echo "IMAGE_NAME=roborregos/home2:vision-cpu" >> .env
    
    # Build the base image if it doesn't exist
    check_image_exists "$CPU_IMAGE"
    if [ $? -eq 1 ]; then
        docker compose -f ../cpu.yaml build
    fi
    ;;
  "gpu")
    #_____GPU_____
    echo "DOCKERFILE=docker/vision/Dockerfile.cuda" >> .env
    echo "BASE_IMAGE=roborregos/home2:cuda_base" >> .env
    echo "IMAGE_NAME=roborregos/home2:vision-gpu" >> .env

    # Build the base image if it doesn't exist
    check_image_exists "$CUDA_IMAGE"
    if [ $? -eq 1 ]; then
        docker compose -f ../cuda.yaml build
    fi

    echo "DOCKER_RUNTIME=nvidia" >> .env
    ;;
  "jetson")
    #_____Jetson_____
    echo "DOCKERFILE=docker/vision/Dockerfile.jetson" >> .env
    echo "BASE_IMAGE=roborregos/home2:l4t_base" >> .env
    echo "IMAGE_NAME=roborregos/home2:vision-jetson" >> .env

    # Build the base image if it doesn't exist
    check_image_exists "$JETSON_IMAGE"
    if [ $? -eq 1 ]; then
        docker compose -f ../jetson.yaml build
    fi

    echo "DOCKER_RUNTIME=nvidia" >> .env
    echo "DISPLAY=:0" >> .env
    ;;
  *)
    echo "Unknown environment type!"
    exit 1
    ;;
esac


#_________________________SETUP_________________________

# Export user
export LOCAL_USER_ID=$(id -u)
export LOCAL_GROUP_ID=$(id -g)

mkdir -p install build log
mkdir -p moondream/install moondream/build moondream/log

# Setup camera permissions
# if [ -e /dev/video0 ]; then
#     echo "Setting permissions for /dev/video0..."
#     sudo chmod 666 /dev/video0  # Allow the container to access the camera device
# fi

#_________________________RUN_________________________



# Check which commands and services to run
MOONDREAM=false
VISION=true
SETUP="colcon build --symlink-install"
PROFILES=()
SERVICES=()

case $TASK in
    "--receptionist")
        PACKAGES="vision_general"
        RUN="ros2 launch vision_general receptionist_launch.py"
        PROFILES=("vision" "moondream")
        SERVICES=("vision" "moondream-node" "moondream-server")
        ;;
    "--carry")
        PACKAGES="vision_general object_detector_2d object_detection_handler"
        RUN="ros2 launch vision_general help_me_carry_launch.py"
        PROFILES=("vision" "moondream")
        SERVICES=("vision" "moondream-node" "moondream-server")
        ;;
    "--storing-groceries")
        PACKAGES="object_detector_2d object_detection_handler"
        RUN="ros2 launch object_detector_2d object_detector_combined.launch.py"
        PROFILES=("vision")
        SERVICES=("vision")
        ;;
    "--gpsr")
        PACKAGES="vision_general object_detector_2d object_detection_handler"
        RUN="ros2 launch vision_general gpsr_launch.py"
        PROFILES=("vision" "moondream")
        SERVICES=("vision" "moondream-node" "moondream-server")
        ;;
    "--moondream")
        PROFILES=("moondream")
        SERVICES=("moondream-node" "moondream-server")
        ;;
    *)
        PROFILES=("vision")
        SERVICES=("vision")
        ;;
esac

# Add command to env if TASK is not empty, otherwise it will run bash
if [ -n "$TASK" ]; then
    COMMAND="source /opt/ros/humble/setup.bash && source frida_interfaces_cache/install/local_setup.bash \
    && $SETUP --packages-select $PACKAGES && source install/setup.bash && $RUN"
    echo "COMMAND= $COMMAND " >> .env
fi

# Add services to compose profiles
COMPOSE_PROFILES=$(IFS=, ; echo "${PROFILES[*]}")
echo "COMPOSE_PROFILES=$COMPOSE_PROFILES" >> .env

NEEDS_BUILD=false

# Check if any enabled service is missing a container
for SERVICE in "${SERVICES[@]}"; do
    CONTAINER=$(docker ps -a --filter "name=${SERVICE}" --format "{{.ID}}")
    if [ -z "$CONTAINER" ]; then
        echo "No container found for service '$SERVICE'."
        NEEDS_BUILD=true
        break 
    fi
done

# If no task set, enter with bash
if [ -z "$TASK" ]; then
    if [ "$NEEDS_BUILD" = true ]; then
        docker compose up --build -d
    else
       RUNNING=$(docker ps --filter "name=vision" --format "{{.ID}}")
        
        if [ -z "$RUNNING" ]; then
            echo "Starting vision service..."
            docker compose up -d
        fi 
        
        docker compose exec vision /bin/bash
    fi

else
    if [ "$NEEDS_BUILD" = true ]; then
        echo "Building and starting containers..."
        docker compose up --build 
    else
        echo "All containers exist. Starting without build..."
        docker compose up
    fi
fi
