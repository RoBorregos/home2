#!/bin/bash
# Run the shelf-detection check over every scene, with and without objects.
# Inside the simulation container:  bash /workspace/src/simulation/frida_gz_sim/scripts/run_shelf_checks.sh [scene...]
# Results and videos: /workspace/sim_logs/shelf/<scene>_<objects>.{json,mp4,log}
# no `set -u`: the ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
OUT=/workspace/sim_logs/shelf
mkdir -p "$OUT"
SCENES=("$@")
[ ${#SCENES[@]} -eq 0 ] && SCENES=(competition closed_three open_rotated)

kill_sim() {
  pkill -INT -f '[r]os2 launch' ; sleep 3
  # a launch whose children were killed can hang: make sure the parent goes too
  pkill -9 -f '[r]os2 launch'
  pkill -9 -f '[g]z sim'; pkill -9 -f '[/]workspace/install/'; pkill -9 -f '[/]opt/ros/jazzy/lib/'
  for _ in $(seq 20); do pgrep -f '[g]z sim|[r]os_gz_bridge' >/dev/null || break; sleep 1; done
}

wait_log() {  # file pattern timeout
  local deadline=$((SECONDS + $3))
  until grep -q "$2" "$1" 2>/dev/null; do
    [ $SECONDS -ge $deadline ] && { echo "timeout waiting for '$2' in $1"; return 1; }
    sleep 2
  done
}

for scene in "${SCENES[@]}"; do
  for objects in 1 0; do
    tag="${scene}_${objects}"
    echo "=== $tag ==="
    kill_sim
    ros2 run frida_gz_sim shelf_world.py "$scene" "$objects" "/tmp/$tag.sdf"
    ros2 launch frida_gz_sim sim.launch.py gui:=false grasp_assist:=false \
      world:=/tmp/$tag.sdf > "$OUT/${tag}_sim.log" 2>&1 &
    wait_log "$OUT/${tag}_sim.log" "activated xarm6_traj_controller" 240 || continue
    ros2 launch frida_gz_sim sim_manipulation.launch.py show_rviz:=false \
      > "$OUT/${tag}_manipulation.log" 2>&1 &
    wait_log "$OUT/${tag}_manipulation.log" "Manipulation core started" 300 || continue
    wait_log "$OUT/${tag}_manipulation.log" "Shelf detector ready" 60 || continue
    # Gazebo depth is exact: drop the real ZED's 1.4 % range correction. Keep the
    # captured frames for offline tuning.
    ros2 param set /shelf_detector depth_scale 1.0
    ros2 param set /shelf_detector dump_path "$OUT/$tag.npz"
    sleep 5
    ros2 run frida_gz_sim sim_shelf_check.py --ros-args -p use_sim_time:=true \
      -p scene:="$scene" -p objects:=$([ "$objects" = 1 ] && echo true || echo false) \
      -p video:="$OUT/$tag.mp4" > "$OUT/${tag}_check.log" 2>&1
    echo "exit=$?"; grep -h "PASS\|FAIL" "$OUT/${tag}_check.log" | tail -1
  done
done
kill_sim
echo "=== summary ==="
grep -h "PASS\|FAIL" "$OUT"/*_check.log
