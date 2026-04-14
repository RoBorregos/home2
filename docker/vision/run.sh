#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

# IMPORTANT: Also edit auto-complete.sh to add new arguments
DETACHED=""
BUILD=""
BUILD_IMAGE=""
UPLOAD_IMAGE=""
CLEAN=""

# check if one of the arguments is --detached
for arg in "${ARGS[@]}"; do
    case $arg in
    "-d")
        DETACHED="-d"
        ;;
    "--build")
        BUILD="true"
        ;;
    "--recreate")
        docker compose down
        ;;
    "--down")
        docker compose down
        exit 0
        ;;
    "--stop")
        docker compose stop
        exit 0
        ;;
    "--build-image")
        BUILD_IMAGE="--build "
        ;;
    "--upload-image")
        UPLOAD_IMAGE="true"
        ;;
    "--clean")
        CLEAN="true"
        ;;
    esac
done

#_________________________SETUP_________________________

# Reset .env
echo "" > .env

# CycloneDDS interface from host
if [ -f /etc/cyclonedds.env ]; then
    source /etc/cyclonedds.env
fi
add_or_update_variable .env "CYCLONE_INTERFACE" "${CYCLONE_INTERFACE:-}"
# Default SHM on for Jetson (l4t), off otherwise
if [ -z "${CYCLONE_SHM:-}" ]; then
    if [ -f /etc/nv_tegra_release ]; then
        CYCLONE_SHM=1
    else
        CYCLONE_SHM=0
    fi
fi
add_or_update_variable .env "CYCLONE_SHM" "$CYCLONE_SHM"

# Export user
add_or_update_variable .env "LOCAL_USER_ID" "$(id -u)"
add_or_update_variable .env "LOCAL_GROUP_ID" "$(id -g)"

# Write environment variables to .env file for Docker Compose and build base images
add_or_update_variable .env "BASE_IMAGE" "roborregos/home2:${ENV_TYPE}_base"
add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:vision-${ENV_TYPE}"
add_or_update_variable .env "DOCKERFILE" "docker/vision/Dockerfile.${ENV_TYPE}"

case $ENV_TYPE in
  "cuda")
  add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
    ;;
  "l4t")
    add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
    add_or_update_variable .env "DISPLAY" ":0"
    ;;
  *)
    echo "Unknown environment type!"
    exit 1
    ;;
esac

# Clean build artifacts if requested
if [ "$CLEAN" == "true" ]; then
  clean_directories .
fi

mkdir -p install build log

#_________________________RUN_________________________

SOURCE_ROS="source /opt/ros/humble/setup.bash && source /usr/local/bin/cyclonedds_setup.sh"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
IGNORE_PACKAGES="--packages-ignore frida_interfaces frida_constants"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"
CYCLONE_SOURCE="source /usr/local/bin/cyclonedds_setup.sh"
PROFILES=()

case $TASK in
    "--hric")
        PACKAGES="vision_general object_detector_2d moondream_run"
        RUN="ros2 launch vision_general hric_launch.py"
        PROFILES=("vision" "moondream")
        ;;
    "--ppc")
        PACKAGES="vision_general object_detector_2d moondream_run"
        RUN="ros2 launch vision_general ppc_launch.py"
        PROFILES=("vision" "moondream")
        ;;
    "--gpsr")
        PACKAGES="vision_general object_detector_2d moondream_run"
        RUN="ros2 launch vision_general gpsr_launch.py"
        PROFILES=("vision" "moondream")
        ;;
    "--moondream")
        PROFILES=("moondream")
        ;;
    *)
        PACKAGES="vision_general object_detector_2d moondream_run"
        RUN="bash"
        PROFILES=("vision")
        ;;
esac

if [ "$BUILD" == "true" ]; then
    SETUP="$SOURCE_ROS && $SOURCE_INTERFACES && $CYCLONE_SOURCE && colcon build $IGNORE_PACKAGES --packages-up-to $PACKAGES && $SOURCE"
else
    SETUP="$SOURCE_ROS && $SOURCE_INTERFACES && $SOURCE && $CYCLONE_SOURCE"
fi

COMMAND="$SETUP && $RUN"

COMPOSE_PROFILES=$(IFS=, ; echo "${PROFILES[*]}")
add_or_update_variable .env "COMPOSE_PROFILES" "$COMPOSE_PROFILES"

if [ "$UPLOAD_IMAGE" == "true" ]; then
  echo "Uploading vision image to DockerHub (env: ${ENV_TYPE})..."
  ensure_and_upload_image "roborregos/home2:vision-${ENV_TYPE}" "docker-compose.yml"
fi

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    ALREADY_RUNNING=$(docker ps -q -f name="vision")
    if [ -z "$ALREADY_RUNNING" ] || [ -n "$BUILD_IMAGE" ]; then
	docker compose up -d $BUILD_IMAGE
    fi
    docker compose exec vision bash -c "$COMMAND"
else
    add_or_update_variable .env "COMMAND" "$COMMAND"
    echo "COmmand = $COMMAND"
    echo "Running docker compose up $DETACHED $BUILD_IMAGE"
    docker compose up $DETACHED $BUILD_IMAGE
fi
