# frida_gz_sim

Gazebo Harmonic (ROS 2 Jazzy) simulation of FRIDA. The real vision, manipulation
and navigation stacks run unchanged against a simulated xArm6, custom gripper,
ZED camera, holonomic base and RPLIDAR pair.

Two modes:

- `--manip`: pick and place on a static base in the `pnp_table` world.
- `--nav`: the RoboCup arena rebuilt from the `robocup2026_1` map, driving the
  base with slam_toolbox localization + nav2 + `nav_central`.

## Quick start

The sim is the `simulation` area, driven from the repo root like every other area.

```bash
git submodule update --init navigation/packages/ira_laser_tools
./run.sh simulation --build --build-image   # first time: image + workspace + object meshes

# pick and place
./run.sh simulation --manip                 # Gazebo window + RViz + manipulation + detector
./run.sh simulation --tm                    # clear the dining table onto the side table

# navigation
./run.sh simulation --nav                   # arena + nav2 + nav_central
./run.sh simulation --nav-tm                # drive a route of areas with move_to_location

./run.sh simulation --kill                  # stop the sim (container stays up)
```

Wait for `simulation ready` before running a task manager: `--manip` and `--nav`
block until the whole stack is live (about a minute for manip, two to three for nav).

## Commands

`./run.sh simulation [task] [flags]`:

| Task | What it does |
|---|---|
| (none) | Shell inside the container (ROS already sourced) |
| `--manip` | Start the pick-and-place sim, open the Gazebo window and RViz, wait until ready |
| `--nav` | Start the navigation sim the same way |
| `--all` | Start both stacks in the arena world (the arena has no objects to pick) |
| `--tm` | Run the pick-and-place task manager in the foreground |
| `--nav-tm` | Run the navigation task manager in the foreground |
| `--rviz` | Open RViz again on a running sim (`--manip` / `--nav` already open it) |
| `--status` | Show the running sim processes and their memory |
| `--kill` | Kill the sim processes, keep the container |

Flags: `--build` (object meshes + ROS workspace), `--build-image` (rebuild the
Docker image), `--headless` (no Gazebo window), `--no-rviz` (no RViz),
`--moveit-rviz` (also open MoveIt's own RViz), plus the usual `--stop`, `--down`,
`--recreate` and `--clean`. The Gazebo window and RViz open by default and X
access is granted for you; on a loaded machine `--headless --no-rviz` leaves more
CPU for the physics.

`--all` loads the arena world and starts both stacks; navigation passes the same
5/5 there. It is a coexistence check, not a full integration: the arena has no
objects to pick, and the extra load can push the real-time factor below 1.0 on a
smaller machine, which slows nav2's sim-time control loops.

Logs land in `docker/simulation/logs/`: `sim.log`, `manipulation.log`,
`vision.log`, `navigation.log`, `rviz.log`.

The container runs on `ROS_DOMAIN_ID=77`, so the sim never mixes with other FRIDA
containers on the host — including when you open a second terminal with
`./run.sh simulation`.

### Launch options

Run these in `./run.sh simulation` (a shell in the container) when you want to
change the defaults:

```bash
ros2 launch frida_gz_sim sim.launch.py gui:=true \
    world:=pnp_table.sdf image_width:=640 image_height:=360 camera_rate:=10 \
    grasp_assist:=false          # physics-only grasps, no weld
ros2 launch frida_gz_sim sim_manipulation.launch.py show_rviz:=true
SIM_YOLO_MODEL=yolo26m.pt ros2 launch frida_gz_sim sim_vision.launch.py
ros2 run frida_gz_sim sim_pnp_task_manager.py --ros-args -p max_objects:=1

# navigation: mobile base in the arena, then the real nav stack on top
ros2 launch frida_gz_sim sim.launch.py gui:=true world:=arena_robocup2026_1.sdf \
    mobile_base:=true grasp_assist:=false \
    spawn_x:=4.1776 spawn_y:=-11.6974 spawn_z:=0.02 spawn_yaw:=-1.9890
ros2 launch frida_gz_sim sim_nav.launch.py map_name:=robocup2026_1
ros2 run frida_gz_sim sim_nav_task_manager.py \
    --ros-args -p route:="kitchen/sink,bedroom/bed"
ros2 run frida_gz_sim sim_dock_test.py      # planned docking, all scenarios
ros2 run frida_gz_sim sim_dock_test.py --ros-args -p scenarios:="island_chair"
rviz2 -d $(ros2 pkg prefix frida_gz_sim)/share/frida_gz_sim/config/sim_approach.rviz
ros2 launch frida_gz_sim sim_rviz.launch.py config:=sim_nav.rviz
```

### Driving the robot by hand

From `./run.sh simulation`:

```bash
# move the arm to a named pose (radians): nav_pose
ros2 action send_goal /manipulation/move_joints_action_server \
  frida_interfaces/action/MoveJoints \
  '{joint_names: [joint1,joint2,joint3,joint4,joint5,joint6],
    joint_positions: [-1.5708, -1.2217, -0.7854, 0.0, 0.1745, 0.7854], velocity: 0.5}'

# table_stare (what the task manager looks from)
ros2 action send_goal /manipulation/move_joints_action_server \
  frida_interfaces/action/MoveJoints \
  '{joint_names: [joint1,joint2,joint3,joint4,joint5,joint6],
    joint_positions: [-1.5708, -1.3963, -1.2217, 0.0, 0.8727, 0.7854], velocity: 0.5}'

# gripper: 1 closes, 0 opens
ros2 service call /xarm/set_tgpio_digital xarm_msgs/srv/SetDigitalIO '{ionum: 1, value: 1}'

# what the detector sees right now
ros2 service call /vision/detection_handler frida_interfaces/srv/DetectionHandler '{label: all}'

# navigate to one area (what move_to_location calls)
ros2 service call /navigation/go_to_map_area frida_interfaces/srv/MoveLocation \
  '{location: kitchen, sublocation: sink}'

# drive the base by hand
ros2 topic pub -r 10 /sim/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```

### Looking at the sim

```bash
gz model --list                     # every model in the world
gz model -m mug -p                  # where an object actually is (ground truth)
gz topic -e -t /world/pnp_table/stats | grep real_time_factor
ros2 topic hz /zed/zed_node/rgb/color/rect/image
ros2 topic echo --once /gripper/grasp_state
```

RViz (`./run.sh simulation --rviz`) shows the robot, the camera point cloud, the detections
image and MoveIt's planning scene and planned paths. For the fixed overview
camera, add an Image display on `/sim/overview_camera/image` — handy when running
headless, since it watches the robot and both tables from outside.

## What runs

| Launch | Contents |
|---|---|
| `sim.launch.py` | Gazebo world, robot spawn, `gz_ros2_control`, ZED topic bridge, gripper bridge, grasp assist |
| `sim_manipulation.launch.py` | MoveIt + `pick_and_place/pick_and_place.launch.py`, all on sim time |
| `sim_vision.launch.py` | `image_orienter` + `object_detector_2d` with the COCO `yolo26s` model |
| `sim_nav.launch.py` | `laserscan_multi_merger` + `localization.launch.py` + `nav2_omni.launch.py` + `nav_central.py` + `table_docker.py` (scan only), all on sim time |

Sim-only pieces:

- `urdf/frida_gz.urdf.xacro` wraps `FRIDA_Real.urdf.xacro` with the Gazebo
  ros2_control plugin, ZED frames and an RGB-D sensor; `frida_gz_sim/description.py`
  applies the few URDF patches Gazebo needs.
- `xarm_sim_bridge.py` serves `/xarm/set_tgpio_digital` (the real gripper IO
  service) and publishes `/gripper/grasp_state`.
- `cloud_frame_fix.py` republishes the Gazebo cloud on the ZED topic in the
  frame its points use, throttled so large clouds do not starve DDS.
- `grasp_attach.py` welds the object between the fingers to the gripper when it
  closes and releases it when it opens. Two-finger pinches slip in Gazebo; this
  stands in for a firm real grasp. It adds one gz `DetachableJoint` per object at
  startup and detaches it right away, so wait for "Grasp attach ready" in
  `sim.log` before moving the arm (`--manip` does). Disable with
  `grasp_assist:=false`.
- The gripper and finger collisions are boxes in the sim URDF only: Gazebo's
  mesh contacts let objects tunnel into the palm. MoveIt keeps the real meshes.
- `sim_manipulation.launch.py` applies `SetParameter(use_sim_time=True)` to everything
  it launches. The manipulation launches no longer take a `use_sim_time` argument,
  and every node must read TF against `/clock` or the pick pipeline fails.
- `octomap_cloud_filter.py` feeds MoveIt's octomap a cloud with everything above the
  table surface removed, remapped onto `/sim/octomap_cloud` for `move_group` only.
  Gazebo's depth camera is noise-free, so each object becomes a solid block of
  occupied voxels and MoveIt reports the gripper in collision with the very object
  it is reaching for - every GPD candidate comes back "grasp pose unreachable"
  (confirmed with `/compute_ik` + `/check_state_validity`). The real ZED cloud is
  sparse enough that the same grasps clear. The table, floor and surroundings stay
  in the octomap, so the arm still avoids them.
- `sim_pnp_task_manager.py` uses the real `VisionTasks` / `ManipulationTasks`:
  look at the table, pick the closest graspable detection, place it next to the
  bowl on the side table, repeat.

Navigation-only pieces:

- `scripts/world_from_map.py` turns `robocup2026_1.pgm` + `.yaml` into
  `worlds/arena_robocup2026_1.sdf`: the occupied cells are merged into wall boxes
  **in map coordinates**, so the Gazebo world frame and the navigation map frame
  are the same and `areas_robocup2026_1.json` applies unchanged. Regenerate with
  `ros2 run frida_gz_sim world_from_map.py <map>.yaml worlds/<world>.sdf`.
- `mobile_base:=true` swaps the `world -> base_link` anchor for gz's
  `VelocityControl` (holonomic, driven from `/cmd_vel`) and `OdometryPublisher`
  (`/odom` plus the `odom -> base_link` TF), and adds a `gpu_lidar` on
  `lidar_front` and `lidar_rear`. Each lidar is blind over the ~90 degree wedge its
  own chassis covers, the same wedge the real driver drops with `ignore_array`.
- The mobile base has no collision geometry and the arena walls are visual only.
  `VelocityControl` overwrites the base velocity every step, so a contact cannot
  stop the robot - it only adds an impulse, and those accumulate until the robot is
  floating above the arena with its lidars over the walls. The gpu_lidar raycasts
  against visuals, so navigation is unaffected. Regenerate the world with
  `--collisions` if you ever drive the base another way. Dropping the base mesh
  contacts also took the real-time factor from 0.35 to 1.0.
- `cmd_vel_relay.py` converts nav2's `TwistStamped` `/cmd_vel` into the plain
  `Twist` the gz bridge takes. Subscribing also makes `/cmd_vel` visible to
  `nav_central`'s requirement check before nav2 is up.
- `initial_pose.py` publishes the known spawn pose on `/initialpose`; on the robot
  a human drops a 2D Pose Estimate and `nav_central` blocks until it arrives. It
  stops as soon as `nav_central` has it: slam_toolbox re-localizes on every
  `/initialpose`, so one that lands after the robot starts driving snaps its
  estimate back to the spawn pose and navigation never recovers.
- `nav2_sim_time.py` sets `use_sim_time` on nav2's lifecycle manager, which
  `nav2_omni.launch.py` hardcodes to false; left on wall time it drops the
  servers' bond heartbeats whenever the sim is not at 1.0 real-time factor.
- `config/nav2_sim_overlay.yaml` is deep-merged onto `nav2_omni_limp.yaml` by
  `nav2_omni.launch.py`: sim time everywhere, `odom_topic: /odom`, and the RGBD
  voxel layers dropped (the sim has no `/point_cloud`; nav runs off the 2D lidar).
- `config/mapper_params_sim.yaml` is `mapper_params_localization.yaml` with sim
  time and `map_start_pose` set to the spawn pose. Keep the two in sync.
- `sim_nav_task_manager.py` uses the real `NavigationTasks.move_to_location()`
  and grades each goal against the robot's ground-truth Gazebo pose.
- `sim_dock_test.py` tests the planned table approach (`table_docker` +
  `nav_main/approach_planner.py`). It spawns `TEST_FURNITURE` from
  `frida_gz_sim/nav.py`: a living-room island with a chair in front of the
  target object. It then docks with `dock_table(surface_type=..., target=...)` at
  seven places: the round dinner table, the counter, the island, the dishwasher,
  the cabinet, a legacy `offset` call, and `approach_point`. Each result is graded against the
  Gazebo pose and the true geometry in `frida_gz_sim/ground_truth.py`:
  - the gap is inside the surface's reach band,
  - the robot is square to the surface,
  - the target is straight ahead,
  - the footprint overlaps nothing.

  Results are written to `logs/dock_test_results.json`. `config/sim_approach.rviz`
  shows the candidate poses, the chosen pose, the MPPI path and the furniture.

## Debugging headless

- `/sim/overview_camera/image` is a fixed camera watching the robot and tables.
- `gz model -m <name> -p` prints an object's pose.
- The ZED topics match `frida_constants` (`/zed/zed_node/...`).

## Adding objects

Add a model under `models/` (primitive collisions, real mass), fetch its mesh in
`scripts/fetch_models.sh`, include it in `worlds/pnp_table.sdf` and register its
height and radius in `frida_gz_sim/objects.py` so grasp assist can attach it.
Objects shorter than ~8 cm are hard for the GPD pick: the octomap of the table
blocks grasp poses low enough to reach them.

## Depends on the rest of the repo

The sim adds no changes outside `simulation/frida_gz_sim/` and `docker/simulation/`,
so it can break when the packages it wraps change. What it relies on:

- `frida_description`: `urdf/omnibase/FRIDA_Real.urdf.xacro` accepting
  `ros2_control_plugin`, the `Custom` gripper (whose `gripper_ros2_control.xacro` must
  stay limited to "Fake" plugins, since the sim declares its own gripper
  `ros2_control`), and the `zed` link/joint names.
- `pick_and_place.launch.py`, `arm_pkg` MoveIt launches, `perception_3d` and the
  `object_detector_2d` registry (`yolo_generic`).
- `task_manager` `ManipulationTasks` / `VisionTasks` / `NavigationTasks` and the
  `frida_constants` topic and frame names.
- `nav_main`: `omni_setup/localization.launch.py` and `omni_setup/nav2_omni.launch.py`
  keeping their `map`, `params_file`, `nav2_overlay_file`, `use_static_map_server`
  and `map_yaml` arguments, `nav_central.py`'s `/navigation/go_to_map_area` service
  and its `default_base` / `nav_type` / `areas_map_name` parameters, and the
  `nav2_omni_limp.yaml` node and plugin names the sim overlay patches.
- `map_context`: `maps/robocup2026_1.*` and `maps/areas/areas_robocup2026_1.json`.
- `ira_laser_tools` (a submodule; initialise it before building).

After merging into this branch, rebuild (`./run.sh simulation --build`) and run
once (`./run.sh simulation --manip` + `--tm`); launch-argument removals in those
packages are the most likely breakage.

## Known limits

- HRI is not simulated.
- The base is velocity-driven and collision-free, so it does not physically collide
  with the arena: the lidars see the walls and nav2 avoids them, but a bad command
  drives through one instead of bumping into it. Check the Gazebo pose, not just
  the nav result.
- Odometry comes straight from Gazebo's `OdometryPublisher` plugin (ground truth),
  not from the real STM32/ODrive/EKF chain, and the wheel joints in `robot.xacro`
  are fixed (cosmetic only) — the base moves as a single rigid body via
  `VelocityControl`. This sim validates the nav2 stack (planner, costmaps,
  behavior tree, nav_central), but not STM firmware, per-wheel IK, encoder
  odometry, or wheel slip (e.g. the 3-wheel limp behavior described in
  `nav2_omni_limp.yaml`). Bugs specific to that layer will not show up here.
- The arena walls are the mapped occupancy grid extruded to 1.2 m, so the sim only
  contains what the lidar saw when the map was made: no furniture above lidar
  height, no people, no doors.
- `laundry/shelf` and `kitchen/counter` sit about 0.4 m from an obstacle, inside
  nav2's 0.45 m inflation radius. They are reachable on the robot (the costmap is
  softer than the real clearance) but flaky in sim; the default route avoids them.
- `grasp_attach.py` assumes `base_link` is the world origin, which only holds with
  a static base, so grasp assist is off in the nav worlds.
- Placement uses the real heatmap place, which prefers the point closest to the
  robot. The sim aims it at the bowl (`close_to`) so objects do not land on the
  table edge; tall objects can still tip over on release.
- Detection uses the generic COCO `yolo26s` model (`SIM_YOLO_MODEL` to change it),
  not the competition model, because the rendered objects are Fuel meshes.
- Both bottles are detected as `bottle`, so the task manager's per-label attempt
  limit can retire one of them before it is ever tried.
