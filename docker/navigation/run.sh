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
CUDA_IMAGE="roborregos/home2:gpu_base"

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
    echo "DOCKERFILE=docker/navigation/Dockerfile.cpu" >> .env
    echo "BASE_IMAGE=roborregos/home2:cpu_base" >> .env
    echo "IMAGE_NAME=roborregos/home2:navigation-cpu" >> .env
    
    # Build the base image if it doesn't exist
    check_image_exists "$CPU_IMAGE"
    if [ $? -eq 1 ]; then
        docker compose -f ../cpu.yaml build
    fi
    ;;
  "gpu")
    #_____GPU_____
    echo "DOCKERFILE=docker/navigation/Dockerfile.gpu" >> .env
    echo "BASE_IMAGE=roborregos/home2:gpu_base" >> .env
    echo "IMAGE_NAME=roborregos/home2:navigation-gpu" >> .env

    # Build the base image if it doesn't exist
    check_image_exists "$CUDA_IMAGE"
    if [ $? -eq 1 ]; then
        docker compose -f ../cuda.yaml build
    fi

    ;;
  "jetson")
    #_____Jetson_____
    echo "DOCKERFILE=docker/navigation/Dockerfile.jetson" >> .env
    echo "BASE_IMAGE=roborregos/home2:l4t_base" >> .env
    echo "IMAGE_NAME=roborregos/home2:navigation-jetson" >> .env
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

NAV_NAME="home2-navigation"
# Check which commands and services to run
echo "TASK=$TASK"
case $TASK in
    "--receptionist")
        PACKAGES="nav_main dashgo_driver sllidar_ros2"
        RUN="cp /workspace/src/navigation/rtabmapdbs/lab_3d_grid.db /home/ros/.ros/rtabmap.db && ros2 launch nav_main receptionist.launch.py"
        ;;
    "--help-me-carry")
        PACKAGES="nav_main dashgo_driver sllidar_ros2"
        RUN="ros2 launch nav_main carry_my.launch.py"
        ;;
    "--storing-groceries")
        PACKAGES="nav_main dashgo_driver sllidar_ros2"
        RUN="cp /workspace/src/navigation/rtabmapdbs/lab_3d_grid.db /home/ros/.ros/rtabmap.db && ros2 launch nav_main storing_groceries.launch.py"
        ;;

esac
echo "PACKAGES=$PACKAGES"
echo "RUN=$RUN"


if [ -n "$TASK" ]; then
    COMMAND="source /opt/ros/humble/setup.bash && source /home/ros/ros_packages/install/setup.bash && colcon build --symlink-install --packages-up-to $PACKAGES && source install/setup.bash && $RUN"
    echo "COMMAND= $COMMAND " >> .env
fi

NEEDS_BUILD=false

# Check if any enabled service is missing a container
CONTAINER=$(docker ps -a --filter "name=${NAV_NAME}" --format "{{.ID}}")
if [ -z "$CONTAINER" ]; then
    echo "No container found for '$NAV_NAME'."
    NEEDS_BUILD=true
    break 
fi


# If no task set, enter with bash
if [ -z "$TASK" ]; then
    if [ "$NEEDS_BUILD" = true ]; then
        docker compose up --build -d
    else
        echo "checking container run"
       RUNNING=$(docker ps --filter "name=home2-navigation" --format "{{.ID}}")
       echo "RUNNING=$RUNNING"
        if [ -z "$RUNNING" ]; then
            echo "Starting navigation docker container..."
            docker compose up -d
        fi 
        docker start home2-navigation
        docker exec -it home2-navigation /bin/bash
    fi

else
    if [ "$NEEDS_BUILD" = true ]; then
        echo "Building and starting containers..."
        docker compose up --build -d
    else
        echo "All containers exist. Starting without build..."
        docker compose up
    fi
fi
