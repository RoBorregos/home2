# Model-free shelf detection

Measures the shelf in front of the robot with the eye-in-hand ZED2. There are no
per-arena heights: a shelf pick or place without a scan from where the robot stands
fails.

## Pieces

| Piece | Where |
| --- | --- |
| Geometry (pure numpy, tested) | `manipulation/packages/perception_3d/perception_3d/shelf_geometry.py` |
| `DetectShelf` service, hosted by `grasp_generator` (same depth image, intrinsics and TF) | `manipulation/packages/perception_3d/perception_3d/shelf_service.py` |
| Interfaces | `frida_interfaces/manipulation/srv/DetectShelf.srv`, `msg/ShelfLevel.msg` |
| Constants / survey poses | `manipulation_constants.py` (`DETECT_SHELF_SERVICE`, `SHELF_SURVEY_POSES`), `xarm_configurations.py` (`shelf_survey_high/low`) |
| Manual test | `keyboard_input.py` options -14 (survey + detect), -15 (capture here), -16 (detect) |
| Task manager | `ManipulationTasks.detect_shelf_levels()`; `pickandplace_task_manager.py` `_ensure_shelf_heights()`, `gpsr_single_tasks.py` `_measure_shelf()` (once per stop) |
| RViz | `arm_pkg/config/frida_moveit.rviz`: the MoveIt view plus "Shelf", the last scan's boards (latched) |

## How it works

1. The caller moves the camera (two survey poses) and calls the service with
   `capture=true` at each; frames accumulate in `base_link`.
2. `detect=true`: crop to the shelf footprint, 1 cm voxels, PCA normals oriented by
   camera height. Up-facing horizontal points -> Z histogram -> one peak per board
   surface. Peaks narrower than 60 % of the shelf width or with gaps are objects, not
   boards. Down-facing peaks give the board thickness and the ceiling (`top_z`).
   Vertical planes at the lateral/far extremes are the side and back walls. A wide
   down-facing plane with nothing right above it is a board above the camera.
3. `add_collision_objects=true` publishes boards and walls as boxes (`frida_shelf_*`),
   grown 2 cm into the free space except the board tops. With `capture=false,
   detect=false` it only re-publishes the last result. It refuses when odom ->
   base_link moved more than 3 cm / 3 deg since the scan.

Tunables are `grasp_generator` parameters under `shelf.`: the `ShelfParams` fields,
`min_range`/`max_range` (0.3/2.5 m), `depth_scale` (1.0 for a simulated camera) and
`dump_path` (save the frames as `.npz`). Debug topics: `/manipulation/shelf_scan_cloud`,
`/manipulation/shelf_geometry`.

## Survey poses

joint1 stays at -90 and joint5 > 0 tilts the camera down.

| Pose | Joints (deg) | Camera in base_link | Sees |
| --- | --- | --- | --- |
| `shelf_survey_high` | -90, -45, -90, 0, 20, 45 | (0.19, 0.06, 1.40), 23 deg down | upper boards from above, the underside of a board above the camera |
| `shelf_survey_low` | -90, -105, 0, 0, 15, 45 | (0.21, 0.06, 0.95), 48 deg down | under the upper boards into the low compartments |

Lower than ~0.95 m the camera would end up closer than the ZED2 minimum depth to a
shelf 15-20 cm away.

## Shelf boxes in pick and place

- `manipulation_core` wipes the scene at the start of every pick; a shelf pick
  (`scan_environment=true`) then restores the last scan (`restore_shelf_scene()`) and
  is refused without one. The wipes after the pick remove the boxes again.
- The shelf place restores them before reaching and always removes them at the end.
- `fix_position_to_plane` restores them before facing a shelf level
  (`table_or_shelf=false`), since the arm may be holding the object.
- A shelf pick clusters the object inside its compartment (`pick._shelf_band()`),
  passed as `min_height`/`max_height` of `PickPerceptionService`; the default band
  merged the object with the boards of a closed cabinet.
- The task manager scans on every cabinet visit (`dock_at` resets the flag). A failed
  scan keeps the previous heights; with none yet, the cabinet place fails.

## Shelf pick

A shelf pick first tries a frontal grasp from the object's cluster
(`pick.shelf_frontal_grasps`), with GPD as the fallback:

- **Grasp.** Fingertips just past the object's middle, at mid height, clear of the
  board (6 cm) and the ceiling; fingers close sideways, the ZED points up.
- **Approach.** From the front, tilted 10, 0 and 20 deg down; 30 and 45 deg more with
  nothing above.
- **Exit.** Two straight MoveIt paths: 2 cm up, then 15 cm back (`_shelf_exit`,
  `planner_id="cartesian"`), collision checked and cut where the IK jumps more than
  0.5 rad between 5 mm waypoints. When cut, the return to `table_stare` is planned
  around the boxes. The old xArm cartesian velocity lift tripped C24 on the real robot
  at the wrist singularity (joint5 -13 deg) and with the elbow stretched (joint3
  -155 deg), and the stop opened the gripper.

## Shelf place

- **Release over the board.** The held object's centre stays 8 cm behind the board
  front and away from the back and sides of the scan (`_shelf_place_bounds`).
- **Keep the object upright.** The approach of `AIM_STRAIGHT_FRONT_QUAT`, rolled so the
  object's "up" as held at the pick points up (`_shelf_place_quat`).
- When a rung fails, the next one is 2 cm higher; nothing is released short of it.

## Speed

| Step | Value |
| --- | --- |
| Survey moves, wait at each pose | 0.75, 0.3 s (`SHELF_SURVEY_VELOCITY`, `SHELF_SURVEY_SETTLE`) |
| Wait at a level's viewing pose | 1.5 s (`SHELF_LEVEL_SETTLE`) |
| Shelf exit | straight MoveIt paths at 0.3 |
| Return after the pick and the place | `table_stare` at 0.75 |
| Release | 0.5 s |

Raise the two settle times first if the robot misses detections or scans a blurred
cloud. Two more speed-ups are `manipulation_core` parameters, off by default:

```bash
ros2 param set /manipulation_core shelf_acceleration 0.5    # default 0.3, every planned shelf move
ros2 param set /manipulation_core shelf_place_straight true # straight into the compartment
```

`shelf_place_straight` falls back to the free plan on the same rung when the line is
cut.

## Known limits

- An apple is too short for the frontal grasp (6 cm board clearance) and falls back to
  GPD.
- The shelf place puts `link_eef` 10 cm above the board whatever the grasp height, so a
  tall, thin object grasped low can tip on release.
- The lowest shelf (~0.39 m) has no frontal grasp with the base 15-20 cm away.
- The task manager still assigns objects to levels with fixed 0.20/0.05 m thresholds.
- Levels are numbered from the lowest board seen; a board seen only from below gets
  the default thickness (1.8 cm). Thin posts of an open bookshelf count as walls.
