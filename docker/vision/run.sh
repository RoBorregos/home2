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
PULL_IMAGE=""
COMPOSE="docker-compose.yaml"

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
        BUILD_IMAGE="--build"
        ;;
    "--upload-image")
        UPLOAD_IMAGE="true"
        ;;
    "--pull-image")
        PULL_IMAGE="true"
        ;;
    esac
done

#_________________________SETUP_________________________
if [ "$UPLOAD_IMAGE" = "true" ]; then
  upload_images "$COMPOSE"
  exit $?
elif [ "${PULL_IMAGE:-}" = "true" ]; then
  if [ -n "$COMPOSE" ] && [ -f "$COMPOSE" ]; then
    echo "Pull flag set: pulling prebuilt images referenced in $COMPOSE ..."
    if pull_image --compose "$COMPOSE"; then
      echo "Pulled images successfully — skipping local image builds."
      BUILD_IMAGE=""
    else
      echo "Pull failed. Falling back to local build (if requested)." >&2
    fi
  else
    echo "Compose file not found; cannot pull images." >&2
  fi
fi
# Reset .env
echo "" > .env

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

mkdir -p install build log moondream/install moondream/build moondream/log

#_________________________RUN_________________________

SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
IGNORE_PACKAGES="--packages-ignore frida_interfaces frida_constants"
MOONDREAM_PACKAGES="moondream_run"
MOONDREAM_COMMAND="ros2 run moondream_run moondream_node.py"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"
PROFILES=()

case $TASK in
    "--receptionist")
        PACKAGES="vision_general object_detector_2d object_detection_handler"
        RUN="ros2 launch object_detector_2d object_detector_combined.launch.py"
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
        PACKAGES="vision_general object_detector_2d object_detection_handler"
        RUN="bash"
        PROFILES=("vision")
        ;;
esac

if [ "$BUILD" == "true" ]; then
    SETUP="$SOURCE_ROS && $SOURCE_INTERFACES && colcon build $IGNORE_PACKAGES --packages-up-to $PACKAGES && $SOURCE"
else
    SETUP="$SOURCE_ROS && $SOURCE_INTERFACES && $SOURCE"
fi

COMMAND="$SETUP && $RUN"
COMMAND_MOONDREAM="$SOURCE_ROS && $SOURCE_INTERFACES && colcon build $IGNORE_PACKAGES --packages-up-to $MOONDREAM_PACKAGES && $SOURCE && $MOONDREAM_COMMAND"
add_or_update_variable .env "COMMAND" "$COMMAND"
add_or_update_variable .env "COMMAND_MOONDREAM" "$COMMAND_MOONDREAM"

COMPOSE_PROFILES=$(IFS=, ; echo "${PROFILES[*]}")
add_or_update_variable .env "COMPOSE_PROFILES" "$COMPOSE_PROFILES"

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    EXISTING_CONTAINER=$(docker ps -a -q -f name="vision")
    if [ -z "$EXISTING_CONTAINER" ] || [ -n "$BUILD_IMAGE" ]; then
        docker compose up -d $BUILD_IMAGE
    else
        docker compose start
    fi
    docker compose exec vision bash -c "$COMMAND"
else
    docker compose up $DETACHED $BUILD_IMAGE
fi
