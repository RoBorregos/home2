#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="cpu" # Simulation will initially be CPU-based

COMPOSE="docker-compose.yaml"
parse_common_flags "$COMPOSE" "${ARGS[@]}"

#_________________________SETUP_________________________

setup_common_env "simulation"

#_________________________RUN_________________________

SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_INSTALL="source /workspace/install/setup.bash"

# Launch the MuJoCo simulation with the robot model
# Assuming a launch file exists in mujoco_spawn or mujoco_ros2_control
RUN_SIMULATION="ros2 launch mujoco_spawn mujoco_sim_init.launch.py"

# Default command to run if no specific task is provided
DEFAULT_COMMAND="bash"

if [ "$BUILD" == "true" ]; then
    COMMAND="$SOURCE_ROS && $SOURCE_INSTALL && $COLCON && $DEFAULT_COMMAND"
else
    COMMAND="$SOURCE_ROS && $SOURCE_INSTALL && $DEFAULT_COMMAND"
fi

case $TASK in
    "--sim")
        COMMAND="$SOURCE_ROS && $SOURCE_INSTALL && $RUN_SIMULATION"
        ;;
    *)
        # Default command (bash) is already set up
        ;;
esac

if [ "$UPLOAD_IMAGE" == "true" ]; then
  echo "Uploading simulation image to DockerHub (env: ${ENV_TYPE})..."
  ensure_and_upload_image "roborregos/home2:simulation-${ENV_TYPE}" "$COMPOSE"
fi

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    ALREADY_RUNNING=$(docker ps -q -f name="simulation")
    if [ -z "$ALREADY_RUNNING" ] || [ -n "$BUILD_IMAGE" ]; then
        docker compose -f "$COMPOSE" up -d $BUILD_IMAGE
    fi
    docker compose -f "$COMPOSE" exec simulation bash -c "$COMMAND"
else
    add_or_update_variable .env "COMMAND" "$COMMAND"
    docker compose -f "$COMPOSE" up $DETACHED $BUILD_IMAGE
fi
