#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
echo "TASK is: $TASK"
ENV_TYPE="${*: -1}"

if [[ ! "$ENV_TYPE" =~ ^(cpu|cuda|l4t)$ ]]; then
  ENV_TYPE="cpu"
fi

COMPOSE="docker-compose.yaml"
parse_common_flags "$COMPOSE" "${ARGS[@]}"

#_________________________SETUP_________________________

setup_common_env "simulation"
add_or_update_variable .env "BASE_IMAGE" "roborregos/home2:${ENV_TYPE}_base"
add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:simulation-${ENV_TYPE}"

if [ "$ENV_TYPE" != "cpu" ]; then
    add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
fi

#_________________________RUN_________________________

SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_INSTALL="if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi"
SOURCE_CYCLONE="source /usr/local/bin/cyclonedds_setup.sh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///etc/cyclonedds.xml"

# Launch the MuJoCo simulation with the robot model
# Assuming a launch file exists in mujoco_spawn or mujoco_ros2_control
RUN_SIMULATION="ros2 launch mujoco_spawn mujoco_sim_init.launch.py"

# Default command to run if no specific task is provided
DEFAULT_COMMAND="bash"

COLCON="colcon build --symlink-install --packages-up-to mujoco_spawn mujoco_ros2_control xarm_description frida_description xarm_msgs --packages-ignore xarm_gazebo"

if [ "$BUILD" == "true" ]; then
    PRE_COMMAND="$COLCON && source /workspace/install/setup.bash && "
else
    PRE_COMMAND=""
fi

case $TASK in
    "--sim")
        COMMAND="$SOURCE_ROS && $SOURCE_CYCLONE && $SOURCE_INSTALL && $PRE_COMMAND$RUN_SIMULATION"
        ;;
    *)
        COMMAND="$SOURCE_ROS && $SOURCE_CYCLONE && $SOURCE_INSTALL && $PRE_COMMAND$DEFAULT_COMMAND"
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
