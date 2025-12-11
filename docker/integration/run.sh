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

# Write environment variables to .env file for Docker Compose and build base images
add_or_update_variable .env "BASE_IMAGE" "roborregos/home2:${ENV_TYPE}_base"
add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:integration-${ENV_TYPE}"
case $ENV_TYPE in
  "cuda")
      add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
      ;;
  "l4t")
      add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
      add_or_update_variable .env "DISPLAY" ":0"
      ;;
esac

# Create dirs with current user to avoid permission problems
mkdir -p install build log

#_________________________RUN_________________________

# Commands to run inside the container
GENERATE_BAML_CLIENT="baml-cli generate --from /workspace/src/task_manager/scripts/utils/baml_src/"
SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"
COLCON="colcon build --packages-ignore frida_interfaces frida_constants --packages-up-to task_manager"

if [ "$BUILD" == "true" ]; then
    SETUP="$GENERATE_BAML_CLIENT && $SOURCE_ROS && $SOURCE_INTERFACES && $COLCON && $SOURCE"
else
    SETUP="$SOURCE_ROS && $SOURCE_INTERFACES && $SOURCE"
fi

case $TASK in
    "--receptionist")
        RUN="ros2 run task_manager receptionist_task_manager.py"
        ;;
    "--help-me-carry")
        RUN="ros2 run task_manager help_me_carry.py"
        ;;
    "--gpsr")
        RUN="ros2 run task_manager gpsr_task_manager.py"
        ;;
    "--test-hri")
        RUN="ros2 run task_manager test_hri_manager.py"
        ;;
    *)
        RUN="bash"
        ;;
esac

COMMAND="$SETUP && $RUN"
add_or_update_variable .env "COMMAND" "$COMMAND"

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    docker compose up -d $BUILD_IMAGE
    docker compose exec integration bash -c "$COMMAND"
else
    docker compose up $DETACHED $BUILD_IMAGE
fi
