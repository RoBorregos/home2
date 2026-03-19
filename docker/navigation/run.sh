#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

# Resolve compose file before parsing flags so --down/--stop/--recreate use the correct file
COMPOSE_FILE="docker-compose.yaml"
case $ENV_TYPE in
    "gpu") COMPOSE_FILE="docker-compose-gpu.yaml" ;;
    "l4t") COMPOSE_FILE="docker-compose-l4t.yaml" ;;
esac

COMPOSE_CMD="docker compose -f $COMPOSE_FILE"
parse_common_flags "${ARGS[@]}"

#_________________________SETUP_________________________

setup_common_env "navigation"

# Navigation-specific env vars
add_or_update_variable .env "DOCKERFILE" "docker/navigation/Dockerfile.${ENV_TYPE}"

case $ENV_TYPE in
    "cpu")
        add_or_update_variable .env "DOCKER_RUNTIME" "runc"
        ;;
    "gpu"|"l4t")
        # compose file already switched above
        ;;
    *)
        add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
        ;;
esac

#_________________________RUN_________________________

COLCON="colcon build --symlink-install --packages-up-to nav_main --packages-ignore frida_interfaces frida_constants"
SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_RTABMAP="if [ -f /home/ros/ros_packages3/install/setup.bash ]; then source /home/ros/ros_packages3/install/setup.bash; fi"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"

if [ "$BUILD" == "true" ]; then
    SETUP="$SOURCE_ROS && $SOURCE_RTABMAP && $SOURCE_INTERFACES && $SOURCE && $COLCON"
else
    SETUP="$SOURCE_ROS && $SOURCE_RTABMAP && $SOURCE_INTERFACES && $SOURCE"
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

if [ "$UPLOAD_IMAGE" == "true" ]; then
  echo "Uploading navigation image to DockerHub (env: ${ENV_TYPE})..."
  ensure_and_upload_image "roborregos/home2:navigation-${ENV_TYPE}" "$COMPOSE_FILE"
fi

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