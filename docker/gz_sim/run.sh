#!/bin/bash
# Gazebo pick-and-place simulation for FRIDA.
set -e
cd "$(dirname "$0")"

usage() {
  cat <<EOF
Usage: docker/gz_sim/run.sh [command] [flags]

Commands:
  shell            Open a shell in the sim container (default)
  build            Build the ROS workspace used by the sim
  up               Start the simulation in the background (logs in docker/gz_sim/logs)
  tm               Run the sim pick-and-place task manager (foreground)
  nav-tm           Run the sim navigation task manager (foreground)
  rviz             Open RViz on the running sim
  demo             build + up + tm
  status           Show the running sim processes
  stop             Kill the sim processes (container keeps running)
  down             Remove the container

Flags (the Gazebo window and RViz open by default):
  --areas <a>      What to start: manip (default), nav, or all
  --headless       Do not open the Gazebo window
  --no-rviz        Do not open RViz
  --moveit-rviz    Also open MoveIt's own RViz
  --build-image    Rebuild the Docker image first

Areas:
  manip            pnp_table world, static base, MoveIt + pick_and_place + detector
  nav              arena world built from the robocup2026_1 map, mobile base,
                   scan merger + slam_toolbox localization + nav2 + nav_central
  all              the arena world with the manipulation stack on top (experimental:
                   both stacks starve nav2's planner on a single machine)
EOF
}

CMD=${1:-shell}
[[ "$CMD" == --* ]] && CMD=shell || shift || true
GUI=true
RVIZ_VIEW=true
MOVEIT_RVIZ=false
BUILD_IMAGE=""
AREAS=manip
next_is_areas=false
for arg in "$@"; do
  if [ "$next_is_areas" = true ]; then AREAS=$arg; next_is_areas=false; continue; fi
  case $arg in
    --areas) next_is_areas=true ;;
    --areas=*) AREAS=${arg#*=} ;;
    --headless) GUI=false ;;
    --no-rviz) RVIZ_VIEW=false ;;
    --moveit-rviz) MOVEIT_RVIZ=true ;;
    --gui) GUI=true ;;
    --rviz) RVIZ_VIEW=true ;;
    --build-image) BUILD_IMAGE="--build" ;;
    -h|--help) usage; exit 0 ;;
  esac
done

export LOCAL_USER_ID=$(id -u)
export LOCAL_GROUP_ID=$(id -g)
mkdir -p ws/build ws/install ws/log cache/gz cache/ultralytics logs

COMPOSE="docker compose -f docker-compose.yaml"
SETUP="source /opt/ros/jazzy/setup.bash && source /usr/local/bin/cyclonedds_setup.sh && if [ -f install/setup.bash ]; then source install/setup.bash; fi"
PACKAGES="frida_interfaces frida_constants frida_gz_sim manipulation_general xarm6_ikfast_plugin nav_main map_context ira_laser_tools"
IGNORE="realsense_gazebo_plugin xarm_gazebo mujoco_ros2_control mujoco_spawn vamp vamp_moveit_plugin omnidriver sllidar_ros2 p9n_node p9n_interface"

ensure_container() {
  if [ -n "$BUILD_IMAGE" ] || [ -z "$(docker ps -q -f name=^home2-gzsim$)" ]; then
    $COMPOSE up -d $BUILD_IMAGE
  fi
}

in_container() {
  local tty_flag="-i"
  [ -t 0 ] && tty_flag="-it"
  docker exec $tty_flag home2-gzsim bash -c "$SETUP && $1"
}

in_background() {
  local name=$1
  local cmd=$2
  docker exec -d home2-gzsim bash -c "$SETUP && exec $cmd > /workspace/sim_logs/$name.log 2>&1"
  echo "started $name (log: docker/gz_sim/logs/$name.log)"
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
        echo "timed out waiting for '${pair#*:}' in docker/gz_sim/logs/${pair%%:*}.log" >&2
        return 1
      fi
      sleep 2
    done
  done
}

# Per-area world and spawn: nav drives the base around the arena built from the map
sim_args() {
  case $AREAS in
    manip) echo "gui:=$GUI" ;;
    nav)
      # Nothing consumes the ZED stream here, so render it slowly and leave the CPU to physics
      echo "gui:=$GUI world:=arena_robocup2026_1.sdf mobile_base:=true grasp_assist:=false" \
           "camera_rate:=2 spawn_x:=4.1776 spawn_y:=-11.6974 spawn_z:=0.02 spawn_yaw:=-1.9890" ;;
    all)
      # Both stacks on one machine: half the camera rate so nav2 keeps its control loop
      echo "gui:=$GUI world:=arena_robocup2026_1.sdf mobile_base:=true grasp_assist:=false" \
           "camera_rate:=5 spawn_x:=4.1776 spawn_y:=-11.6974 spawn_z:=0.02 spawn_yaw:=-1.9890" ;;
    *) echo "unknown --areas '$AREAS' (manip|nav|all)" >&2; exit 1 ;;
  esac
}

start_all() {
  stop_all
  # The windows inside the container need access to the host X server
  if [ "$GUI" = true ] || [ "$RVIZ_VIEW" = true ] || [ "$MOVEIT_RVIZ" = true ]; then
    xhost +local: >/dev/null
  fi
  rm -f logs/sim.log logs/manipulation.log logs/vision.log logs/rviz.log logs/navigation.log
  in_background sim "ros2 launch frida_gz_sim sim.launch.py $(sim_args)"
  wait_for_logs "sim:activated xarm6_traj_controller"

  if [ "$AREAS" = manip ] || [ "$AREAS" = all ]; then
    in_background manipulation "ros2 launch frida_gz_sim sim_manipulation.launch.py show_rviz:=$MOVEIT_RVIZ"
    in_background vision "ros2 launch frida_gz_sim sim_vision.launch.py"
  fi
  if [ "$AREAS" = nav ] || [ "$AREAS" = all ]; then
    in_background navigation "ros2 launch frida_gz_sim sim_nav.launch.py"
  fi

  echo "waiting for the stack to come up..."
  # slam_toolbox loading the pose graph and nav2 activating take longer than manipulation
  [ "$AREAS" = manip ] || READY_TIMEOUT=${READY_TIMEOUT:-420}
  case $AREAS in
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
  [ "$AREAS" = nav ] && config=sim_nav.rviz
  in_background rviz "ros2 launch frida_gz_sim sim_rviz.launch.py config:=$config"
}

stop_all() {
  # Bracketed patterns keep pkill from matching (and killing) this very shell
  docker exec home2-gzsim bash -c "pkill -INT -f '[r]os2 launch'; sleep 3; pkill -9 -f '[g]z sim'; pkill -9 -f '[/]workspace/install/'; pkill -9 -f '[/]opt/ros/jazzy/lib/'; true" >/dev/null 2>&1 || true
}

case $CMD in
  shell) ensure_container; in_container "bash" ;;
  build) ensure_container; build_ws ;;
  up) ensure_container; start_all ;;
  tm) ensure_container; in_container "ros2 run frida_gz_sim sim_pnp_task_manager.py" ;;
  nav-tm) ensure_container; in_container "ros2 run frida_gz_sim sim_nav_task_manager.py" ;;
  rviz) ensure_container; start_rviz ;;
  demo) ensure_container; build_ws; start_all; in_container "ros2 run frida_gz_sim sim_pnp_task_manager.py" ;;
  status) docker exec home2-gzsim bash -c "ps -eo pid,etime,rss,args --sort=-rss | grep -E 'gz sim|move_group|ros2|python3' | grep -v grep | cut -c1-160" ;;
  stop) stop_all ;;
  down) $COMPOSE down ;;
  -h|--help|help) usage ;;
  *) usage; exit 1 ;;
esac
