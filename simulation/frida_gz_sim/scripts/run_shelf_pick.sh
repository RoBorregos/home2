#!/bin/bash
# Shelf pick check (keyboard_input -11 flow) in one scene, with vision.
# Inside the simulation container:
#   bash /workspace/src/simulation/frida_gz_sim/scripts/run_shelf_pick.sh [scene] [level] [label] [noboxes]
# Results and video: /workspace/sim_logs/shelf/pick_<scene>[_noboxes].{json,mp4,log}
# "noboxes" keeps the shelf boxes out of the planning scene (A/B against them).
# no `set -u`: the ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
SCENE=${1:-competition}
LEVEL=${2:-2}
LABEL=${3:-bottle}
BOXES=${4:-boxes}
OUT=/workspace/sim_logs/shelf
mkdir -p "$OUT"
TAG="pick_${SCENE}_l${LEVEL}"
[ "$BOXES" = noboxes ] && TAG="${TAG}_noboxes"

kill_sim() {
  pkill -INT -f '[r]os2 launch' ; sleep 3
  # a launch whose children were killed can hang: make sure the parent goes too
  pkill -9 -f '[r]os2 launch'
  pkill -9 -f '[g]z sim'; pkill -9 -f '[/]workspace/install/'; pkill -9 -f '[/]opt/ros/jazzy/lib/'
  pkill -9 -f '[x]arm_velocity_sim'
  for _ in $(seq 20); do pgrep -f '[g]z sim|[r]os_gz_bridge' >/dev/null || break; sleep 1; done
}

wait_log() {  # file pattern timeout
  local deadline=$((SECONDS + $3))
  until grep -q "$2" "$1" 2>/dev/null; do
    [ $SECONDS -ge $deadline ] && { echo "timeout waiting for '$2' in $1"; return 1; }
    sleep 2
  done
}

kill_sim
ros2 run frida_gz_sim shelf_world.py "$SCENE" 1 "/tmp/$TAG.sdf" "pick:$LEVEL"
ros2 launch frida_gz_sim sim.launch.py gui:=false world:=/tmp/$TAG.sdf > "$OUT/${TAG}_sim.log" 2>&1 &
wait_log "$OUT/${TAG}_sim.log" "activated xarm6_traj_controller" 240 || exit 1
wait_log "$OUT/${TAG}_sim.log" "Grasp attach ready" 120 || exit 1
# The shelf frontal grasp drives straight in with the xArm cartesian velocity mode,
# which the plain sim does not serve (no /xarm/robot_states). xarm_velocity_sim.py
# (branch sim/task-harness) emulates it; start it before manipulation so the core
# finds the xArm services. Installed with frida_gz_sim, or a path in XARM_VELOCITY_SIM.
if ros2 pkg executables frida_gz_sim | grep -q xarm_velocity_sim.py; then
  ros2 run frida_gz_sim xarm_velocity_sim.py --ros-args -p use_sim_time:=true \
    > "$OUT/${TAG}_xarm_velocity.log" 2>&1 &
elif [ -n "$XARM_VELOCITY_SIM" ] && [ -f "$XARM_VELOCITY_SIM" ]; then
  python3 "$XARM_VELOCITY_SIM" --ros-args -p use_sim_time:=true \
    > "$OUT/${TAG}_xarm_velocity.log" 2>&1 &
else
  echo "no xArm velocity emulation: cartesian approaches will not run"
fi
sleep 3
ros2 launch frida_gz_sim sim_manipulation.launch.py show_rviz:=false > "$OUT/${TAG}_manipulation.log" 2>&1 &
ros2 launch frida_gz_sim sim_vision.launch.py > "$OUT/${TAG}_vision.log" 2>&1 &
wait_log "$OUT/${TAG}_manipulation.log" "Manipulation core started" 300 || exit 1
wait_log "$OUT/${TAG}_manipulation.log" "Shelf detector ready" 60 || exit 1
wait_log "$OUT/${TAG}_vision.log" "Loaded models" 600 || exit 1
# Gazebo depth is exact: drop the real ZED's 1.4 % range correction.
ros2 param set /shelf_detector depth_scale 1.0
[ "$BOXES" = noboxes ] && ros2 param set /shelf_detector publish_boxes false
# The sim octomap filter drops points above the dining table (z 0.76) so the
# noise-free cloud does not turn the object into a solid block of voxels around the
# gripper (see octomap_cloud_filter.py). On a shelf, drop only a box around the
# object to grasp: a height cut would also drop the other objects on that shelf and
# the arm would plan through them (Gazebo then stops it and the controller aborts).
BOX=$(python3 -c "
from frida_gz_sim.shelf_scenes import SCENES, pick_object_pose
x, y, z = pick_object_pose(SCENES['$SCENE'], $LEVEL - 1)
print(f'[{x - 0.08:.3f}, {y - 0.07:.3f}, {z + 0.005:.3f}, {x + 0.08:.3f}, {y + 0.07:.3f}, {z + 0.24:.3f}]')")
pkill -f '[o]ctomap_cloud_filter.py'
ros2 run frida_gz_sim octomap_cloud_filter.py --ros-args -p use_sim_time:=true \
  -p "drop_box:=$BOX" > "$OUT/${TAG}_octomap_filter.log" 2>&1 &
sleep 5
ros2 run frida_gz_sim sim_shelf_pick.py --ros-args -p use_sim_time:=true \
  -p scene:="$SCENE" -p level:="$LEVEL" -p label:="$LABEL" \
  -p video:="$OUT/$TAG.mp4" > "$OUT/${TAG}_check.log" 2>&1
echo "exit=$?"
grep -h "shelf pick: \(PASS\|FAIL\)" "$OUT/${TAG}_check.log" | tail -1
kill_sim
