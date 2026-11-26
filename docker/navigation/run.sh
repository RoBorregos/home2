#!/bin/bash
source ../../lib.sh

#_________________________ARGUMENTS_________________________

ARGS=("$@")
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

DETACHED=""
BUILD=""
BUILD_IMAGE=""
UPLOAD_IMAGE=""
CLEAN=""
COMPOSE_FILE="docker-compose.yaml"
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
    "--clean")
        CLEAN="true"
        ;;
    esac
done

#_________________________CACHE PAIRS_________________________

# Warm GPSR's named path pairs inside the running stack; never starts or reconfigures it
if [ "$TASK" == "--cache-pairs" ]; then
    if [ -z "$(docker ps -q -f name=^home2-navigation$)" ]; then
        echo "Navigation is not running. Start it first (e.g. ./run.sh navigation --gpsr)." >&2
        exit 1
    fi
    echo "Caching location pairs (blocks until warmup finishes)..."
    CACHE_SETUP="source /opt/ros/jazzy/setup.bash && if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi && if [ -f install/setup.bash ]; then source install/setup.bash; fi && source /usr/local/bin/cyclonedds_setup.sh"
    # Without nav_central the call would wait for the service forever
    if ! docker compose exec -T navigation bash -c "$CACHE_SETUP && ros2 service list" | grep -qx "/navigation/cache_pairs"; then
        echo "nav_central is not up yet (no /navigation/cache_pairs). Wait for navigation setup and retry." >&2
        exit 1
    fi
    OUTPUT=$(docker compose exec -T navigation bash -c "$CACHE_SETUP && ros2 service call /navigation/cache_pairs std_srvs/srv/Trigger {}") || { echo "$OUTPUT"; exit 1; }
    echo "$OUTPUT"
    # The service answers success=False when not ready or incomplete
    echo "$OUTPUT" | grep -q "success=True" || exit 1
    exit 0
fi

#_________________________SETUP_________________________

echo "Configuring USB devices for Navigation..."
if ! bash ./setup-USB.sh; then
    echo "Error: USB devices setup failed."
fi

# Reset .env
echo "" > .env

# CycloneDDS interface from host
add_or_update_variable .env "CYCLONE_INTERFACE" "$(resolve_cyclone_interface)"
# Default SHM on for Jetson (l4t), off otherwise
if [ -z "${CYCLONE_SHM:-}" ]; then
    if [ -f /etc/nv_tegra_release ]; then
        CYCLONE_SHM=1
    else
        CYCLONE_SHM=0
    fi
fi
add_or_update_variable .env "CYCLONE_SHM" "$CYCLONE_SHM"
RESOLVED_MAP_NAME=$(resolve_map_name "lab_23_march.db")
if [ -z "${MAP_NAME:-}" ]; then
    echo "MAP_NAME not exported in this shell; using '$RESOLVED_MAP_NAME' from shell rc file."
fi
add_or_update_variable .env "MAP_NAME" "$RESOLVED_MAP_NAME"
# Export user
add_or_update_variable .env "LOCAL_USER_ID" "$(id -u)"
add_or_update_variable .env "LOCAL_GROUP_ID" "$(id -g)"

add_or_update_variable .env "BASE_IMAGE" "roborregos/home2:${ENV_TYPE}_base"
add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:navigation-${ENV_TYPE}"
add_or_update_variable .env "DOCKERFILE" "docker/navigation/Dockerfile.${ENV_TYPE}"

case $ENV_TYPE in
    "cpu")
        add_or_update_variable .env "DOCKER_RUNTIME" "runc"
        ;;
    *)
        add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
        ;;
esac
PROFILES=("cpu_l4t")

COMPOSE_PROFILES=$(IFS=, ; echo "${PROFILES[*]}")
add_or_update_variable .env "COMPOSE_PROFILES" "$COMPOSE_PROFILES"

# Clean build artifacts if requested
if [ "$CLEAN" == "true" ]; then
  clean_directories .
fi

# Create dirs with current user to avoid permission problems
mkdir -p install build log

#_________________________RUN_________________________

COLCON="colcon build --symlink-install --packages-up-to nav_main --packages-ignore frida_interfaces frida_constants --cmake-args -Wno-dev"
SOURCE_ROS="source /opt/ros/jazzy/setup.bash"
SOURCE_RTABMAP="if [ -f /home/ros/ros_packages3/install/setup.bash ]; then source /home/ros/ros_packages3/install/setup.bash; fi"
SOURCE_NAV2="if [ -f /home/ros/nav2_ws/install/setup.bash ]; then source /home/ros/nav2_ws/install/setup.bash; fi"
SOURCE_INTERFACES="if [ -f frida_interfaces_cache/install/local_setup.bash ]; then source frida_interfaces_cache/install/local_setup.bash; fi"
SOURCE="if [ -f install/setup.bash ]; then source install/setup.bash; fi"
CYCLONE_SOURCE="source /usr/local/bin/cyclonedds_setup.sh"

if [ "$BUILD" == "true" ]; then
    SETUP="$SOURCE_ROS && $SOURCE_RTABMAP && $SOURCE_NAV2 && $SOURCE_INTERFACES && $CYCLONE_SOURCE && $COLCON && $SOURCE"
else
    SETUP="$SOURCE_ROS && $SOURCE_RTABMAP && $SOURCE_NAV2 && $SOURCE_INTERFACES && $SOURCE && $CYCLONE_SOURCE"
fi

case $TASK in
    "--mapping")
        RUN="ros2 launch nav_main mapping.launch.py"
        ;;
    "--hric" | "--ppc" | "--dlc" | "--gpsr" | "--safety" )
        RUN="ros2 launch nav_main general_navigation.launch.py"
        ;;
    "--restaurant")
        # Unknown venue: slam_toolbox mapping + nav2 with the restaurant overlay
        # (slow speeds, long voxel persistence) + table docking.
        RUN="ros2 launch nav_main restaurant.launch.py"
        ;;
    "--tagger")
        RUN="ros2 run map_context map_area_tagger.py"
        ;;
    "--move")
        RUN="ros2 launch nav_main omni_basics.launch.py"
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
        docker compose up -d $BUILD_IMAGE 
    fi
    docker compose exec navigation bash -c "$COMMAND"
else
    add_or_update_variable .env "COMMAND" "$COMMAND"
    docker compose up $DETACHED $BUILD_IMAGE
fi
