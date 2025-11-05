#!/bin/bash
source ../../lib.sh

# Export user
export LOCAL_USER_ID=$(id -u)
export LOCAL_GROUP_ID=$(id -g)

#_________________________ARGUMENTS_________________________

ARGS=("$@")
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

DETACHED=""
BUILD=""
BUILD_IMAGE=""
COMPOSE="docker-compose-${ENV_TYPE}.yaml"

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

# Ensure .env exists
if [ ! -f ".env" ]; then
    touch .env
fi

add_or_update_variable .env "BASE_IMAGE" "roborregos/home2:${ENV_TYPE}_base"
add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:navigation-${ENV_TYPE}"
add_or_update_variable .env "DOCKERFILE" "docker/navigation/Dockerfile.${ENV_TYPE}"

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
        RUN="ls"
        ;;
    "--mapping")
        RUN="ls"
        ;;
    "--help-me-carry")
        RUN="ros2 launch nav_main carry_my.launch.py"
        ;;
    "--storing-groceries")
        RUN="ros2 launch nav_main storing_groceries.launch.py"
        ;;
        *)
        RUN="bash"
        ;;

esac

COMMAND="$SETUP && $RUN"
add_or_update_variable .env "COMMAND" "$COMMAND"

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    EXISTING_CONTAINER=$(docker ps -a -q -f name="navigation")
    if [ -z "$EXISTING_CONTAINER" ] || [ "$BUILD_IMAGE" == "--build" ]; then
        docker compose up -d $BUILD_IMAGE
    else
        docker compose start
    fi
    docker compose exec navigation bash -c "$COMMAND"
else
    docker compose up $DETACHED $BUILD_IMAGE
fi