#!/bin/bash
source ../../lib.sh

export LOCAL_USER_ID=$(id -u)
export LOCAL_GROUP_ID=$(id -g)

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

# IMPORTANT: Also edit auto-complete.sh to add new arguments
DETACHED=""
BUILD=""

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
    # TODO: build
    esac
done

#_________________________BUILD_________________________

# Ensure .env exists
if [ ! -f ".env" ]; then
  touch .env
fi

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


#_________________________SETUP_________________________

mkdir -p install build log moondream/install moondream/build moondream/log

#_________________________RUN_________________________

# Check which commands and services to run
SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
IGNORE_PACKAGES="--packages-ignore frida_interfaces frida_constants xarm_msgs"
MOONDREAM_PACKAGES="moondream_run"
MOONDREAM_COMMAND="ros2 run moondream_run moondream_node.py"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"

PROFILES=()

case $TASK in
    "--receptionist")
        PACKAGES="vision_general object_detector_2d object_detection_handler"
        RUN="ros2 launch vision_general receptionist_launch.py"
        PROFILES=("vision" "moondream")
        ;;
    "--carry")
        PACKAGES="vision_general object_detector_2d object_detection_handler"
        RUN="ros2 launch vision_general help_me_carry_launch.py"
        PROFILES=("vision" "moondream")
        ;;
    "--storing-groceries")
        PACKAGES="object_detector_2d object_detection_handler"
        RUN="ros2 launch object_detector_2d object_detector_combined.launch.py"
        PROFILES=("vision")
        ;;
    "--gpsr")
        PACKAGES="vision_general object_detector_2d object_detection_handler"
        RUN="ros2 launch vision_general gpsr_launch.py"
        PROFILES=("vision" "moondream")
        ;;
    "--moondream")
        PROFILES=("moondream")
        ;;
    *)
        PROFILES=("vision")
        ;;
esac

if [ "$BUILD" == "true" ]; then
    SETUP="$SOURCE_ROS && $SOURCE_INTERFACES && colcon build $IGNORE_PACKAGES --packages-up-to $PACKAGES && $SOURCE"
  COMMAND_MOONDREAM="$SOURCE_ROS && $SOURCE_INTERFACES && colcon build $IGNORE_PACKAGES --packages-up-to $MOONDREAM_PACKAGES  && $SOURCE && $MOONDREAM_COMMAND"

else
    SETUP="$SOURCE_ROS && $SOURCE_INTERFACES && $SOURCE"
fi

COMMAND="$SETUP && $RUN"
add_or_update_variable .env "COMMAND" "$COMMAND"
add_or_update_variable .env "COMMAND_MOONDREAM" "$COMMAND_MOONDREAM"



# Add services to compose profiles
COMPOSE_PROFILES=$(IFS=, ; echo "${PROFILES[*]}")
add_or_update_variable .env "COMPOSE_PROFILES" "$COMPOSE_PROFILES"

docker compose up $DETACHED
