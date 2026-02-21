#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

DETACHED=""
BUILD=""
BUILD_IMAGE=""
COMPOSE_FILE="docker-compose.yaml"
# Parse arguments
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
    esac
done

#_________________________SETUP_________________________

# Reset .env
echo "" > .env

# Export user
add_or_update_variable .env "LOCAL_USER_ID" "$(id -u)"
add_or_update_variable .env "LOCAL_GROUP_ID" "$(id -g)"

add_or_update_variable .env "BASE_IMAGE" "roborregos/home2:${ENV_TYPE}_base"
add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:navigation-${ENV_TYPE}"
add_or_update_variable .env "DOCKERFILE" "docker/navigation/Dockerfile.${ENV_TYPE}"

case $ENV_TYPE in
    "cpu")
        add_or_update_variable .env "DOCKER_RUNTIME" "runc"
        ;;
    "gpu")
        COMPOSE_FILE="docker-compose-gpu.yaml"
        ;;
    *)
        add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
        ;;
esac

# Create dirs with current user to avoid permission problems
mkdir -p install build log

#_________________________RUN_________________________

COLCON="colcon build --symlink-install --packages-up-to nav_main --packages-ignore frida_interfaces"
SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"

if [ "$BUILD" == "true" ]; then
    SETUP="$SOURCE_ROS && $SOURCE_INTERFACES && $SOURCE && $COLCON"
else
    SETUP="$SOURCE_ROS && $SOURCE_INTERFACES && $SOURCE"
fi

case $TASK in
    "--receptionist")
        RUN="echo 'WORKING IN PROGRESS'"
        ;;
    "--mapping")
        RUN="echo 'WORKING IN PROGRESS'"
        ;;
    "--storing-groceries")
        RUN="echo 'WORKING IN PROGRESS'"
        ;;
    *)
        RUN="bash"
        ;;
esac

COMMAND="$SETUP && $RUN"

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    ALREADY_RUNNING=$(docker ps -q -f name="navigation")
    if [ -z "$ALREADY_RUNNING" ] || [ -n "$BUILD_IMAGE" ]; then
        docker compose -f $COMPOSE_FILE up -d $BUILD_IMAGE 
    fi
    docker compose -f $COMPOSE_FILE exec navigation bash -c "$COMMAND"
else
    add_or_update_variable .env "COMMAND" "$COMMAND"
    docker compose -f $COMPOSE_FILE up $DETACHED $BUILD_IMAGE
fi