#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")
ENV_TYPE="${*: -1}"

BUILD_IMAGE=""
DETACHED=""

for arg in "${ARGS[@]}"; do
    case $arg in
    "-d")
        DETACHED="-d"
        ;;
    "--build-image")
        BUILD_IMAGE="--build "
        ;;
    "--down")
        docker compose down
        exit 0
        ;;
    "--stop")
        docker compose stop
        exit 0
        ;;
    esac
done

#_________________________SETUP_________________________

echo "" > .env

# CycloneDDS interface from host
if [ -f /etc/cyclonedds.env ]; then
    source /etc/cyclonedds.env
fi
add_or_update_variable .env "CYCLONE_INTERFACE" "${CYCLONE_INTERFACE:-}"

# Export user
add_or_update_variable .env "LOCAL_USER_ID" "$(id -u)"
add_or_update_variable .env "LOCAL_GROUP_ID" "$(id -g)"

case $ENV_TYPE in
  "l4t")
    add_or_update_variable .env "BASE_IMAGE" "roborregos/home2:l4t_base"
    add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:l4t_base"
    add_or_update_variable .env "DOCKERFILE" "docker/Dockerfile.ROS-l4t"
    add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
    add_or_update_variable .env "DISPLAY" ":0"
    add_or_update_variable .env "ZED_WORKSPACE" "/home/orin/zed"
    add_or_update_variable .env "ZED_CAMERA_MODEL" "zed2"
    ;;
  "cuda")
    add_or_update_variable .env "BASE_IMAGE" "roborregos/home2:cuda_base"
    add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:cuda_base"
    add_or_update_variable .env "DOCKERFILE" "docker/Dockerfile.ROS"
    add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
    add_or_update_variable .env "ZED_WORKSPACE" "${HOME}/zed"
    add_or_update_variable .env "ZED_CAMERA_MODEL" "zed2"
    ;;
  *)
    echo "ZED requires GPU. Use: ./run.sh zed l4t  or  ./run.sh zed cuda"
    exit 1
    ;;
esac

#_________________________RUN_________________________

echo "Starting ZED camera container (env: ${ENV_TYPE})..."
docker compose up $DETACHED $BUILD_IMAGE
