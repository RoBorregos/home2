#!/bin/bash
source ../../lib.sh

# Export user
export LOCAL_USER_ID=$(id -u)
export LOCAL_GROUP_ID=$(id -g)

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

# IMPORTANT: Also edit auto-complete.sh to add new arguments
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
        docker compose -f "$COMPOSE" down
        ;;
    "--down")
        docker compose -f "$COMPOSE" down
        exit 0
        ;;
    "--stop")
        docker compose -f "$COMPOSE" stop
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

# Create dirs with current user to avoid permission problems
mkdir -p install build log

#_________________________RUN_________________________


SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
GPD_SETUP=". /home/ros/setup_gpd.sh"
GPD_EXPORT="export GPD_INSTALL_DIR=/workspace/install/gpd"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"
COLCON="colcon build --symlink-install --packages-up-to manipulation_general --packages-ignore realsense_gazebo_plugin xarm_gazebo frida_interfaces"

if [ "$BUILD" == "true" ]; then
    SETUP="$GPD_SETUP && $GPD_EXPORT && $SOURCE_ROS && $SOURCE_INTERFACES && $COLCON && $SOURCE"
else
    SETUP="$GPD_SETUP && $GPD_EXPORT && $SOURCE_ROS && $SOURCE_INTERFACES && $SOURCE"
fi

case $TASK in
    "--receptionist")
        RUN="ros2 launch manipulation_general receptionist.launch.py"
        ;;
    "--carry")
        RUN="ros2 launch manipulation_general carry.launch.py"
        ;;
    "--storing-groceries")
        RUN="ros2 launch vision_general storing_groceries_launch.py"
        ;;
    "--gpsr")
        RUN="ros2 launch manipulation_general gpsr.launch.py"
        ;;
    *)
        RUN="bash"
        ;;
esac

COMMAND="$SETUP && $RUN"
add_or_update_variable .env "COMMAND" "$COMMAND"

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    EXISTING_CONTAINER=$(docker ps -a -q -f name="manipulation")
    if [ -z "$EXISTING_CONTAINER" ] || [ "$BUILD_IMAGE" == "--build" ]; then
        docker compose -f "$COMPOSE" up -d $BUILD_IMAGE
    else
        docker compose -f "$COMPOSE" start
    fi
    docker compose -f "$COMPOSE" exec manipulation bash -c "$COMMAND"
else
    docker compose -f "$COMPOSE" up $DETACHED $BUILD_IMAGE
fi
