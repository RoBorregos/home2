#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

# IMPORTANT: Also edit auto-complete.sh to add new arguments
parse_common_flags "${ARGS[@]}"

#_________________________SETUP_________________________

setup_common_env "integration"

# Integration-specific env vars
case $ENV_TYPE in
  "cuda")
      add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
      ;;
  "l4t")
      add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
      add_or_update_variable .env "DISPLAY" ":0"
      ;;
esac

#_________________________RUN_________________________

# Commands to run inside the container
GENERATE_BAML_CLIENT="baml-cli generate --from /workspace/src/task_manager/scripts/utils/baml_src/"
SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"
COLCON="colcon build --symlink-install --packages-ignore frida_interfaces frida_constants --packages-up-to task_manager"

if [ "$BUILD" == "true" ]; then
    SETUP="$GENERATE_BAML_CLIENT && $SOURCE_ROS && $SOURCE_INTERFACES && $COLCON && $SOURCE"
else
    SETUP="$SOURCE_ROS && $SOURCE_INTERFACES && $SOURCE"
fi

case $TASK in
    "--hric")
        RUN="ros2 run task_manager hric_task_manager.py"
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

if [ "$UPLOAD_IMAGE" == "true" ]; then
  echo "Uploading integration image to DockerHub (env: ${ENV_TYPE})..."
  ensure_and_upload_image "roborregos/home2:integration-${ENV_TYPE}" "docker-compose.yml"
fi

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    ALREADY_RUNNING=$(docker ps -q -f name="integration")
    if [ -z "$ALREADY_RUNNING" ] || [ -n "$BUILD_IMAGE" ]; then
        docker compose up -d $BUILD_IMAGE
    fi
    docker compose exec integration bash -c "$COMMAND"
else
    add_or_update_variable .env "COMMAND" "$COMMAND"
    docker compose up $DETACHED $BUILD_IMAGE
fi