# frida_gz_sim

Gazebo Harmonic (ROS 2 Jazzy) simulation of FRIDA for pick and place. The real
vision and manipulation stacks run unchanged against a simulated xArm6, custom
gripper and ZED camera.

## Quick start

```bash
docker/gz_sim/run.sh build --build-image   # first time: image + workspace + object meshes
docker/gz_sim/run.sh up                    # Gazebo + MoveIt/pick_and_place + object detector
docker/gz_sim/run.sh tm                    # clear the dining table onto the side table
```

`docker/gz_sim/run.sh demo` does build + up + tm in one go. Logs go to
`docker/gz_sim/logs/`. Add `--gui` to `up` for the Gazebo window; `docker/gz_sim/run.sh rviz` opens RViz
(robot, camera cloud, detections image, MoveIt planning scene and planned paths). `stop` kills the sim
processes and `down` removes the container.

The container uses `ROS_DOMAIN_ID=77`, so it never mixes with other FRIDA
containers on the host. From another terminal: `docker/gz_sim/run.sh shell`.

## What runs

| Launch | Contents |
|---|---|
| `sim.launch.py` | Gazebo world, robot spawn, `gz_ros2_control`, ZED topic bridge, gripper bridge, grasp assist |
| `sim_manipulation.launch.py` | MoveIt on sim time + `pick_and_place/pick_and_place.launch.py` |
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

## Known limits

- The arm base is static; navigation and HRI are not simulated.
- Placement uses the real heatmap place, which prefers the point closest to the
  robot. The sim aims it at the bowl (`close_to`) so objects do not land on the
  table edge; tall objects can still tip over on release.
- Detection uses the generic COCO `yolo26s` model (`SIM_YOLO_MODEL` to change it),
  not the competition model, because the rendered objects are Fuel meshes.
