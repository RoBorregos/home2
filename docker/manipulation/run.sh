#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")  # Save all arguments in an array
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

COMPOSE="docker-compose-${ENV_TYPE}.yaml"
parse_common_flags "$COMPOSE" "${ARGS[@]}"

#_________________________SETUP_________________________

setup_common_env "manipulation"

#_________________________RUN_________________________


SOURCE_ROS="source /opt/ros/humble/setup.bash"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
GPD_SETUP=". /home/ros/setup_gpd.sh"
GPD_EXPORT="export GPD_INSTALL_DIR=/workspace/install/gpd"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"
COLCON="colcon build --symlink-install --packages-up-to manipulation_general xarm6_ikfast_plugin --packages-ignore realsense_gazebo_plugin xarm_gazebo frida_interfaces"
CYCLONE_SOURCE="source /usr/local/bin/cyclonedds_setup.sh"

if [ "$BUILD" == "true" ]; then
    SETUP="$GPD_SETUP && $GPD_EXPORT && $SOURCE_ROS && $SOURCE_INTERFACES &&  $CYCLONE_SOURCE && $COLCON && $SOURCE"
else
    SETUP="$GPD_SETUP && $GPD_EXPORT && $SOURCE_ROS && $SOURCE_INTERFACES && $SOURCE &&  $CYCLONE_SOURCE "
fi

case $TASK in
    "--hric")
        RUN="ros2 launch manipulation_general hric.launch.py"
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
    "--ppc")
        RUN="ros2 launch manipulation_general ppc.launch.py"
        ;;
    *)
        RUN="bash"
        ;;
esac

COMMAND="$SETUP && $RUN"

if [ "$UPLOAD_IMAGE" == "true" ]; then
  echo "Uploading manipulation image to DockerHub (env: ${ENV_TYPE})..."
  ensure_and_upload_image "roborregos/home2:manipulation-${ENV_TYPE}" "$COMPOSE"
fi

if [ "$RUN" = "bash" ] && [ -z "$DETACHED" ]; then
    ALREADY_RUNNING=$(docker ps -q -f name="manipulation")
    if [ -z "$ALREADY_RUNNING" ] || [ -n "$BUILD_IMAGE" ]; then
        docker compose -f "$COMPOSE" up -d $BUILD_IMAGE
    fi
    docker compose -f "$COMPOSE" exec manipulation bash -c "$COMMAND"
else
    add_or_update_variable .env "COMMAND" "$COMMAND"
    docker compose -f "$COMPOSE" up $DETACHED $BUILD_IMAGE
fi
