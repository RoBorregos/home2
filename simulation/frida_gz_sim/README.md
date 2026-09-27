# frida_gz_sim

Gazebo Harmonic (ROS 2 Jazzy) simulation of FRIDA for pick and place. The real
vision and manipulation stacks run unchanged against a simulated xArm6, custom
gripper and ZED camera.

## Quick start

```bash
cd ~/Documents/home2-gz-sim                # the sim lives on the sim/gazebo-pnp branch
docker/gz_sim/run.sh build --build-image   # first time: image + workspace + object meshes
docker/gz_sim/run.sh up                    # Gazebo window + RViz + manipulation + detector
docker/gz_sim/run.sh tm                    # clear the dining table onto the side table
docker/gz_sim/run.sh stop                  # stop the sim (container stays up)
```

Wait for `simulation ready` before running `tm`: `up` blocks until Gazebo,
manipulation and the detector are all live (about a minute).

## Commands

`docker/gz_sim/run.sh <command> [flags]`:

| Command | What it does |
|---|---|
| `build` | Fetch object meshes and build the ROS workspace (add `--build-image` to rebuild the Docker image) |
| `up` | Start Gazebo + manipulation + vision, open the Gazebo window and RViz, wait until ready |
| `tm` | Run the pick-and-place task manager in the foreground |
| `demo` | `build` + `up` + `tm` |
| `rviz` | Open RViz again on a running sim (`up` already opens it) |
| `shell` | Shell inside the container (ROS already sourced) |
| `status` | Show the running sim processes and their memory |
| `stop` | Kill the sim processes, keep the container |
| `down` | Remove the container |

The Gazebo window and RViz open by default and X access is granted for you.
Flags: `--headless` (no Gazebo window), `--no-rviz` (no RViz), `--moveit-rviz`
(also open MoveIt's own RViz), `--build-image` (rebuild the image first). On a
loaded machine, `up --headless --no-rviz` leaves more CPU for the physics.

Logs land in `docker/gz_sim/logs/`: `sim.log`, `manipulation.log`, `vision.log`,
`rviz.log`.

The container runs on `ROS_DOMAIN_ID=77`, so the sim never mixes with other FRIDA
containers on the host — including when you open a second terminal with
`run.sh shell`.

### Launch options

Run these in `run.sh shell` when you want to change the defaults:

```bash
ros2 launch frida_gz_sim sim.launch.py gui:=true \
    world:=pnp_table.sdf image_width:=640 image_height:=360 camera_rate:=10 \
    grasp_assist:=false          # physics-only grasps, no weld
ros2 launch frida_gz_sim sim_manipulation.launch.py show_rviz:=true
SIM_YOLO_MODEL=yolo26m.pt ros2 launch frida_gz_sim sim_vision.launch.py
ros2 run frida_gz_sim sim_pnp_task_manager.py --ros-args -p max_objects:=1
```

### Driving the robot by hand

From `run.sh shell`:

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
```

### Looking at the sim

```bash
gz model --list                     # every model in the world
gz model -m mug -p                  # where an object actually is (ground truth)
gz topic -e -t /world/pnp_table/stats | grep real_time_factor
ros2 topic hz /zed/zed_node/rgb/color/rect/image
ros2 topic echo --once /gripper/grasp_state
```

RViz (`run.sh rviz`) shows the robot, the camera point cloud, the detections
image and MoveIt's planning scene and planned paths. For the fixed overview
camera, add an Image display on `/sim/overview_camera/image` — handy when running
headless, since it watches the robot and both tables from outside.

## What runs

| Launch | Contents |
|---|---|
| `sim.launch.py` | Gazebo world, robot spawn, `gz_ros2_control`, ZED topic bridge, gripper bridge, grasp assist |
| `sim_manipulation.launch.py` | MoveIt + `pick_and_place/pick_and_place.launch.py`, all on sim time |
| `sim_vision.launch.py` | `image_orienter` + `object_detector_2d` with the COCO `yolo26s` model |

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
  `sim.log` before moving the arm (`run.sh up` does). Disable with
  `grasp_assist:=false`.
- The gripper and finger collisions are boxes in the sim URDF only: Gazebo's
  mesh contacts let objects tunnel into the palm. MoveIt keeps the real meshes.
- `sim_manipulation.launch.py` applies `SetParameter(use_sim_time=True)` to everything
  it launches. The manipulation launches no longer take a `use_sim_time` argument,
  and every node must read TF against `/clock` or the pick pipeline fails.
- `sim_pnp_task_manager.py` uses the real `VisionTasks` / `ManipulationTasks`:
  look at the table, pick the closest graspable detection, place it next to the
  bowl on the side table, repeat.

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

The sim adds no changes outside `simulation/frida_gz_sim/` and `docker/gz_sim/`, so
it can break when the packages it wraps change. What it relies on:

- `frida_description`: `urdf/omnibase/FRIDA_Real.urdf.xacro` accepting
  `ros2_control_plugin`, the `Custom` gripper (whose `gripper_ros2_control.xacro` must
  stay limited to "Fake" plugins, since the sim declares its own gripper
  `ros2_control`), and the `zed` link/joint names.
- `pick_and_place.launch.py`, `arm_pkg` MoveIt launches, `perception_3d` and the
  `object_detector_2d` registry (`yolo_generic`).
- `task_manager` `ManipulationTasks` / `VisionTasks` and the `frida_constants` topic
  and frame names.

After merging into this branch, rebuild (`run.sh build`) and run once
(`run.sh up` + `run.sh tm`); launch-argument removals in those packages are the
most likely breakage.

## Known limits

- The arm base is static; navigation and HRI are not simulated.
- Placement uses the real heatmap place, which prefers the point closest to the
  robot. The sim aims it at the bowl (`close_to`) so objects do not land on the
  table edge; tall objects can still tip over on release.
- Detection uses the generic COCO `yolo26s` model (`SIM_YOLO_MODEL` to change it),
  not the competition model, because the rendered objects are Fuel meshes.
