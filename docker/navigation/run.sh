#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

DETACHED=""
BUILD=""
BUILD_IMAGE=""
UPLOAD_IMAGE=""
CLEAN=""
COMPOSE_FILE="docker-compose.yaml"
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
        BUILD_IMAGE="--build"
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

echo "Configuring USB devices for Navigation..."
if ! bash ./setup-USB.sh; then
    echo "Error: USB devices setup failed."
fi

# Reset .env
echo "" > .env

# CycloneDDS interface from host
if [ -f /etc/cyclonedds.env ]; then
    source /etc/cyclonedds.env
fi
add_or_update_variable .env "CYCLONE_INTERFACE" "${CYCLONE_INTERFACE:-}"
add_or_update_variable .env "MAP_NAME" "${MAP_NAME:-lab_23_march.db}"
# Export user
add_or_update_variable .env "LOCAL_USER_ID" "$(id -u)"
add_or_update_variable .env "LOCAL_GROUP_ID" "$(id -g)"

add_or_update_variable .env "BASE_IMAGE" "roborregos/home2:${ENV_TYPE}_base"
add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:navigation-${ENV_TYPE}"
add_or_update_variable .env "DOCKERFILE" "docker/navigation/Dockerfile.${ENV_TYPE}"

case $ENV_TYPE in
    "cpu")
        add_or_update_variable .env "DOCKER_RUNTIME" "runc"
        PROFILES=("cpu_l4t")
        ;;
    "gpu")
        add_or_update_variable .env "DOCKER_RUNTIME" "runc"
        PROFILES=("gpu")
        ;;
    *)
        add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
        PROFILES=("cpu_l4t")
        ;;
esac

COMPOSE_PROFILES=$(IFS=, ; echo "${PROFILES[*]}")
add_or_update_variable .env "COMPOSE_PROFILES" "$COMPOSE_PROFILES"

# Clean build artifacts if requested
if [ "$CLEAN" == "true" ]; then
  clean_directories .
fi

# Create dirs with current user to avoid permission problems
mkdir -p install build log

#_________________________RUN_________________________

COLCON="colcon build --symlink-install --packages-up-to nav_main --packages-ignore frida_interfaces frida_constants"
SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_RTABMAP="if [ -f /home/ros/ros_packages3/install/setup.bash ]; then source /home/ros/ros_packages3/install/setup.bash; fi"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"
CYCLONE_SOURCE="source /usr/local/bin/cyclonedds_setup.sh"

if [ "$BUILD" == "true" ]; then
    SETUP="$SOURCE_ROS && $SOURCE_RTABMAP && $SOURCE_INTERFACES && $SOURCE && $CYCLONE_SOURCE && $COLCON"
else
    SETUP="$SOURCE_ROS && $SOURCE_RTABMAP && $SOURCE_INTERFACES && $SOURCE && $CYCLONE_SOURCE"
fi

case $TASK in
    "--mapping")
        RUN="ros2 launch nav_main mapping.launch.py"
        ;;
    "--hric")
        RUN="ros2 launch nav_main navigation_composition.launch.py"
        ;;
    *)
        RUN="bash"
        ;;
esac

COMMAND="$SETUP && $RUN"

if [ "$UPLOAD_IMAGE" == "true" ]; then
  echo "Uploading navigation image to DockerHub (env: ${ENV_TYPE})..."
  ensure_and_upload_image "roborregos/home2:navigation-${ENV_TYPE}" "$COMPOSE_FILE"
fi

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    ALREADY_RUNNING=$(docker ps -q -f name="navigation")
    if [ -z "$ALREADY_RUNNING" ] || [ -n "$BUILD_IMAGE" ]; then
        docker compose up -d $BUILD_IMAGE 
    fi
    docker compose exec navigation bash -c "$COMMAND"
else
    add_or_update_variable .env "COMMAND" "$COMMAND"
    docker compose up $DETACHED $BUILD_IMAGE
fi
