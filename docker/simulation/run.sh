#!/bin/bash
# Gazebo simulation area: pick and place (manip) or navigation (nav).
source ../../lib.sh
set -e

#_________________________ARGUMENTS_________________________

ARGS=("$@")
TASK=${ARGS[0]}
ENV_TYPE="${*: -1}"

if [[ ! "$ENV_TYPE" =~ ^(cpu|cuda|l4t)$ ]]; then
  ENV_TYPE="cpu"
fi

COMPOSE_FILE="docker-compose.yaml"
parse_common_flags "$COMPOSE_FILE" "${ARGS[@]}"

# The Gazebo window and RViz open by default
AREA=manip
GUI=true
RVIZ_VIEW=true
MOVEIT_RVIZ=false
for arg in "${ARGS[@]}"; do
  case $arg in
    --manip) AREA=manip ;;
    --nav) AREA=nav ;;
    --all) AREA=all ;;
    --headless) GUI=false ;;
    --no-rviz) RVIZ_VIEW=false ;;
    --moveit-rviz) MOVEIT_RVIZ=true ;;
  esac
done

usage() {
  cat <<USAGE
Usage: ./run.sh simulation [task] [flags]

Tasks:
  (none)           Open a shell in the sim container (ROS already sourced)
  --manip          Start the pick-and-place sim (pnp_table world, static base)
  --nav            Start the navigation sim (arena from the robocup2026_1 map)
  --all            Start both stacks in the arena world (no objects to pick there)
  --tm             Run the pick-and-place task manager (foreground)
  --nav-tm         Run the navigation task manager (foreground)
  --rviz           Open RViz on the running sim
  --status         Show the running sim processes and their memory
  --kill           Kill the sim processes, keep the container

Flags:
  --build          Fetch the object meshes and build the ROS workspace
  --build-image    Rebuild the Docker image first
  --headless       Do not open the Gazebo window
  --no-rviz        Do not open RViz
  --moveit-rviz    Also open MoveIt's own RViz
  --recreate       Recreate the container
  --stop | --down  Stop / remove the container
  --clean          Delete build/, install/ and log/

Examples:
  ./run.sh simulation --build --build-image
  ./run.sh simulation --nav
  ./run.sh simulation --nav-tm
  ./run.sh simulation --manip --headless --no-rviz
USAGE
}

case $TASK in
  -h|--help|help) usage; exit 0 ;;
esac

#_________________________SETUP_________________________

setup_common_env "simulation"
# Built on plain ros:jazzy, not the FRIDA base image: Gazebo Harmonic, gz_ros2_control
# and nav2 come from the ROS 2 Jazzy archive and nothing else in the image is shared.
add_or_update_variable .env "BASE_IMAGE" "ros:jazzy"
add_or_update_variable .env "IMAGE_NAME" "roborregos/home2:simulation-${ENV_TYPE}"
if [ "$ENV_TYPE" != "cpu" ]; then
  add_or_update_variable .env "DOCKER_RUNTIME" "nvidia"
fi
mkdir -p build install log cache/gz cache/ultralytics logs

COMPOSE="docker compose -f $COMPOSE_FILE"
SETUP="source /opt/ros/jazzy/setup.bash && source /usr/local/bin/cyclonedds_setup.sh && if [ -f install/setup.bash ]; then source install/setup.bash; fi"
PACKAGES="frida_interfaces frida_constants frida_gz_sim manipulation_general xarm6_ikfast_plugin nav_main map_context ira_laser_tools"
IGNORE="realsense_gazebo_plugin xarm_gazebo mujoco_ros2_control mujoco_spawn vamp vamp_moveit_plugin omnidriver sllidar_ros2 p9n_node p9n_interface"

ensure_container() {
  if [ -n "$BUILD_IMAGE" ] || [ -z "$(docker ps -q -f name=^home2-simulation$)" ]; then
    $COMPOSE up -d $BUILD_IMAGE
  fi
}

in_container() {
  local tty_flag="-i"
  [ -t 0 ] && tty_flag="-it"
  docker exec $tty_flag home2-simulation bash -c "$SETUP && $1"
}

in_background() {
  local name=$1
  local cmd=$2
  docker exec -d home2-simulation bash -c "$SETUP && exec $cmd > /workspace/sim_logs/$name.log 2>&1"
  echo "started $name (log: docker/simulation/logs/$name.log)"
}

build_ws() {
  # Capped parallelism: PCL/MoveIt translation units need ~2 GB each
  in_container "bash /workspace/src/simulation/frida_gz_sim/scripts/fetch_models.sh && MAKEFLAGS=-j${BUILD_JOBS:-4} colcon build --symlink-install --parallel-workers ${BUILD_WORKERS:-2} --packages-up-to $PACKAGES --packages-ignore $IGNORE --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
}

# Blocks until every pattern appears in its log, or fails after a timeout
wait_for_logs() {
  local deadline=$((SECONDS + ${READY_TIMEOUT:-300}))
  local pair
  for pair in "$@"; do
    until grep -q "${pair#*:}" "logs/${pair%%:*}.log" 2>/dev/null; do
      if [ $SECONDS -ge $deadline ]; then
        echo "timed out waiting for '${pair#*:}' in docker/simulation/logs/${pair%%:*}.log" >&2
        return 1
      fi
      sleep 2
    done
  done
}

# Per-area world and spawn: nav drives the base around the arena built from the map
sim_args() {
  case $AREA in
    manip) echo "gui:=$GUI" ;;
    nav)
      # Nothing consumes the ZED stream here, so render it slowly and leave the CPU to physics
      echo "gui:=$GUI world:=arena_robocup2026_1.sdf mobile_base:=true grasp_assist:=false" \
           "camera_rate:=2 spawn_x:=4.1776 spawn_y:=-11.6974 spawn_z:=0.02 spawn_yaw:=-1.9890" ;;
    all)
      # Both stacks on one machine: half the camera rate so nav2 keeps its control loop
      echo "gui:=$GUI world:=arena_robocup2026_1.sdf mobile_base:=true grasp_assist:=false" \
           "camera_rate:=5 spawn_x:=4.1776 spawn_y:=-11.6974 spawn_z:=0.02 spawn_yaw:=-1.9890" ;;
  esac
}

start_all() {
  kill_sim
  # The windows inside the container need access to the host X server
  if [ "$GUI" = true ] || [ "$RVIZ_VIEW" = true ] || [ "$MOVEIT_RVIZ" = true ]; then
    xhost +local: >/dev/null
  fi
  rm -f logs/sim.log logs/manipulation.log logs/vision.log logs/rviz.log logs/navigation.log
  in_background sim "ros2 launch frida_gz_sim sim.launch.py $(sim_args)"
  wait_for_logs "sim:activated xarm6_traj_controller"

  if [ "$AREA" = manip ] || [ "$AREA" = all ]; then
    in_background manipulation "ros2 launch frida_gz_sim sim_manipulation.launch.py show_rviz:=$MOVEIT_RVIZ"
    in_background vision "ros2 launch frida_gz_sim sim_vision.launch.py"
  fi
  if [ "$AREA" = nav ] || [ "$AREA" = all ]; then
    in_background navigation "ros2 launch frida_gz_sim sim_nav.launch.py"
  fi

  echo "waiting for the stack to come up..."
  # slam_toolbox loading the pose graph and nav2 activating take longer than manipulation
  [ "$AREA" = manip ] || READY_TIMEOUT=${READY_TIMEOUT:-420}
  case $AREA in
    manip) wait_for_logs "sim:Grasp attach ready" "manipulation:Manipulation core started" "vision:Loaded models" ;;
    nav) wait_for_logs "navigation:Finished Setup, Starting monitoring" ;;
    all) wait_for_logs "manipulation:Manipulation core started" "vision:Loaded models" "navigation:Finished Setup, Starting monitoring" ;;
  esac
  # RViz needs the planning scene, so it starts once move_group is up
  if [ "$RVIZ_VIEW" = true ]; then
    start_rviz
  fi
  echo "simulation ready"
}

start_rviz() {
  xhost +local: >/dev/null
  local config=sim.rviz
  [ "$AREA" = nav ] && config=sim_nav.rviz
  in_background rviz "ros2 launch frida_gz_sim sim_rviz.launch.py config:=$config"
}

kill_sim() {
  # Bracketed patterns keep pkill from matching (and killing) this very shell
  docker exec home2-simulation bash -c "pkill -INT -f '[r]os2 launch'; sleep 3; pkill -9 -f '[g]z sim'; pkill -9 -f '[/]workspace/install/'; pkill -9 -f '[/]opt/ros/jazzy/lib/'; true" >/dev/null 2>&1 || true
  # Starting a new stack while the old one still holds its DDS endpoints leaves the
  # /clock bridge dead, so wait for the processes to actually go
  local waited=0
  while docker exec home2-simulation bash -c "pgrep -f '[g]z sim|[/]opt/ros/jazzy/lib/ros_gz_bridge' >/dev/null" 2>/dev/null; do
    [ $waited -ge 20 ] && break
    sleep 1
    waited=$((waited + 1))
  done
}

#_________________________RUN_________________________

if [ "$UPLOAD_IMAGE" == "true" ]; then
  echo "Uploading simulation image to DockerHub (env: ${ENV_TYPE})..."
  ensure_and_upload_image "roborregos/home2:simulation-${ENV_TYPE}" "$COMPOSE_FILE"
fi

ensure_container
if [ "$BUILD" == "true" ]; then
  build_ws
fi

case $TASK in
  --manip|--nav|--all) start_all ;;
  --tm) in_container "ros2 run frida_gz_sim sim_pnp_task_manager.py" ;;
  --nav-tm) in_container "ros2 run frida_gz_sim sim_nav_task_manager.py" ;;
  --rviz) start_rviz ;;
  --status) docker exec home2-simulation bash -c "ps -eo pid,etime,rss,args --sort=-rss | grep -E 'gz sim|move_group|ros2|python3' | grep -v grep | cut -c1-160" ;;
  --kill) kill_sim ;;
  # --build and the other common flags have already run; a bare call opens a shell
  --build|--build-image|--clean|--recreate|-d|cpu|cuda|l4t|"")
    if [ "$BUILD" != "true" ]; then in_container "bash"; fi ;;
  *) usage; exit 1 ;;
esac
