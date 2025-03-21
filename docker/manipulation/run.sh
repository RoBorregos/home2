#!/bin/bash


#_________________________BUILD_________________________

# Image names
CPU_IMAGE="roborregos/home2:cpu_base"
CUDA_IMAGE="roborregos/home2:cuda_base"
CONTAINER_NAME="home2-manipulation" # Service name in docker-compose yaml file

rebuild=0

# Check if --rebuild flag is passed
if [ "$1" == "--rebuild" ]; then
    echo "Rebuilding image..."
    rebuild=1
    # if container exists, prompt the user to ensure they want to delete it
    if [ -n "$(docker ps -a -q -f "name=home2-manipulation")" ]; then
        read -p "Do you want to delete the existing container? (y/n): " confirmation
        if [[ "$confirmation" == "y" ]]; then
            docker stop home2-manipulation
            docker rm home2-manipulation
            echo "Container deleted."
        else
            echo "Operation cancelled."
        fi 
    fi
fi

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

# Check type of environment (CPU, cuda, or Jetson), default CPU
ENV_TYPE="cpu"

# Check device type
if [[ -f /etc/nv_tegra_release ]]; then
    ENV_TYPE="jetson"
else
    # Check if NVIDIA cudas are available
    if command -v nvidia-smi > /dev/null 2>&1; then
        if nvidia-smi > /dev/null 2>&1; then
            ENV_TYPE="cuda"
        fi
    fi
fi
echo "Detected environment: $ENV_TYPE"
> .env
# Write environment variables to .env file for Docker Compose and build base images
USER_UID=$(id -u)
USER_GID=$(id -g)
echo "USER_UID=$USER_UID, USER_GID=$USER_GID"

case $ENV_TYPE in
  "cpu")
    #_____CPU_____
    echo "DOCKERFILE=docker/manipulation/Dockerfile.cpu" >> .env
    echo "BASE_IMAGE=roborregos/home2:cpu_base" >> .env
    echo "IMAGE_NAME=roborregos/home2:manipulation-cpu" >> .env
    
    # Build the base image if it doesn't exist or if --rebuild flag is passed
    check_image_exists "$CPU_IMAGE" || [ $rebuild -eq 1 ]
    if [ $? -eq 1 ]; then
        docker compose -f ../cpu.yaml build --build-arg "USER_UID=$(id -u)" --build-arg "USER_GID=$(id -g)"
        rebuild=1
    fi
    ;;
  "cuda")
    #_____cuda_____
    echo "DOCKERFILE=docker/manipulation/Dockerfile.cuda" >> .env
    echo "BASE_IMAGE=roborregos/home2:cuda_base" >> .env
    echo "IMAGE_NAME=roborregos/home2:manipulation-cuda" >> .env

    # Build the base image if it doesn't exist or if --rebuild flag is passed
    check_image_exists "$CUDA_IMAGE" || [ $rebuild -eq 1 ]
    if [ $? -eq 1 ]; then
        docker compose -f ../cuda.yaml build --build-arg "USER_UID=$(id -u)" --build-arg "USER_GID=$(id -g)"
        rebuild=1
    fi

    ;;
    "jetson")
    #_____Jetson_____
    echo "DOCKERFILE=docker/manipulation/Dockerfile.jetson" >> .env
    echo "BASE_IMAGE=roborregos/home2:l4t_base" >> .env
    echo "IMAGE_NAME=roborregos/home2:manipulation-jetson" >> .env
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

#_________________________RUN_________________________
xhost +
# Check if the container exists
EXISTING_CONTAINER=$(docker ps -a -q -f "name=$CONTAINER_NAME")
if [ -z "$EXISTING_CONTAINER" ]; then
    echo "No container with the name $CONTAINER_NAME exists. Building and starting the container now..."
    if [ $ENV_TYPE == "cpu" ]; then
        docker compose -f docker-compose-cpu.yaml up --build -d
    elif [ $ENV_TYPE == "cuda" ]; then
        docker compose -f docker-compose-cuda.yaml up --build -d
    elif [ $ENV_TYPE == "jetson" ]; then
        docker compose -f docker-compose-jetson.yaml up --build -d
    fi
    echo "Running prebuild script..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME /bin/bash -c "./src/home2/prebuild.sh"
fi

# Check if the container is running
RUNNING_CONTAINER=$(docker ps -q -f "name=$CONTAINER_NAME")

if [ -n "$RUNNING_CONTAINER" ]; then
    echo "Container $CONTAINER_NAME is already running. Executing bash..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME /bin/bash
else
    echo "Container $CONTAINER_NAME is stopped. Starting it now..."
    docker compose up --build -d
    docker start $CONTAINER_NAME
    docker exec $CONTAINER_NAME /bin/bash
fi

