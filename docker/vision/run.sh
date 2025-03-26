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
    echo "DOCKER_RUNTIME=''" >> .env
    
    # Build the base image if it doesn't exist
    check_image_exists "$CPU_IMAGE"
    if [ $? -eq 1 ]; then
        docker compose -f ../cpu.yaml build
    fi
    ;;
  "gpu")
    #_____GPU_____
    echo "DOCKERFILE=docker/vision/Dockerfile.gpu" >> .env
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

# Remove current install, build and log directories if they exist 
# if [ -d "../../install" ] || [ -d "../../log" ] || [ -d "../../build" ]; then
#   read -p "Do you want to delete 'install', 'log', and 'build' directories (Recommended for first build)? (y/n): " confirmation
#   if [[ "$confirmation" == "y" ]]; then
#     rm -rf ../../install/ ../../log/ ../../build/
#     echo "Directories deleted."
#   else
#     echo "Operation cancelled."
#   fi
# fi

# Setup camera permissions
# if [ -e /dev/video0 ]; then
#     echo "Setting permissions for /dev/video0..."
#     sudo chmod 666 /dev/video0  # Allow the container to access the camera device
# fi

#_________________________RUN_________________________

SERVICE_NAME="vision"  # Change this to your service name in docker-compose.yml

# Check if the container exists
EXISTING_CONTAINER=$(docker ps -a -q -f name=$SERVICE_NAME)
if [ -z "$EXISTING_CONTAINER" ]; then
    echo "No container with the name $SERVICE_NAME exists. Building and starting the container now..."
    docker compose up --build -d
fi

# Check if the container is running
RUNNING_CONTAINER=$(docker ps -q -f name=$SERVICE_NAME)

# If the container is not running, start it
if [ -z "$RUNNING_CONTAINER" ]; then
    echo "Container $SERVICE_NAME is not running. Starting it now..."
    docker compose up --build -d
fi

# Commands to run inside the container
SOURCE_ROS="source /opt/ros/humble/setup.bash"
COLCON="colcon build --packages-up-to "
SOURCE="source install/setup.bash"
SETUP="$SOURCE_ROS && $COLCON vision_general && $SOURCE"
RUN=""
MOONDREAM=false

case $TASK in
    "--moondream")
        RUN=""
        MOONDREAM=true
        ;;
    "--receptionist")
        RUN="ros2 launch vision_general receptionist_launch.py"
        MOONDREAM=true
        ;;

    *)
        RUN=""
        ;;
esac


if [ "$MOONDREAM" = true ]; then
    echo "Running Moondream..."
    RUNNING_CONTAINER=$(docker ps -q -f name=moondream-node)
    if [ -z "$RUNNING_CONTAINER" ]; then
        docker compose -f moondream.yaml up -d --build
    fi
fi

# check if TASK is not empty
if [ -z "$TASK" ]; then
    docker compose exec $SERVICE_NAME /bin/bash
else
    if [ -z "$detached" ]; then
        docker compose exec $SERVICE_NAME bash -c "$SETUP && $RUN"
    else
        echo "Running in detached mode..."
        docker compose exec -d $SERVICE_NAME bash -c "$SETUP && $RUN"
    fi
fi
