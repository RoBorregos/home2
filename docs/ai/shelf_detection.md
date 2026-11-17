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
| Sim check | `simulation/frida_gz_sim`: `shelf_scenes.py`, `scripts/sim_shelf_check.py`, `scripts/run_shelf_checks.sh` |
| RViz | robot: `arm_pkg/config/frida_moveit.rviz` (MoveIt view + "Shelf" markers, latched); sim: `ros2 launch frida_gz_sim sim_rviz.launch.py config:=shelf.rviz` |

## How it works

1. The caller moves the camera (two survey poses) and calls the service with
   `capture=true` at each; frames accumulate in `base_link`.
2. `detect=true`: crop to the shelf footprint, 1 cm voxels, PCA normals oriented by
   camera height. Up-facing horizontal points -> Z histogram -> one peak per board
   surface. Peaks narrower than 60 % of the shelf width or with gaps are objects, not
   boards. Down-facing peaks give the board thickness and the ceiling (`top_z`).
   Vertical planes at the lateral/far extremes are the side and back walls.
3. `add_collision_objects=true` publishes boards and walls as boxes
   (`frida_shelf_*`) to the MoveIt scene. With `capture=false, detect=false` it only
   re-publishes the last result. It refuses when odom -> base_link moved more than
   3 cm / 3 deg since the scan (the boxes are in base_link and would otherwise
   follow the robot to other furniture); without odometry (static sim base) there
   is no check.

## Shelf boxes in pick and place

- `manipulation_core` wipes the scene at the start of every pick. For a shelf pick
  (`scan_environment=true`) it then calls `RobotArm.restore_shelf_scene()`, which
  re-publishes the last scan, and refuses the pick when there is none or the base
  moved since it. The usual wipes after the pick remove the boxes again.
- The shelf place does the same before reaching and always removes the boxes at the
  end (`remove_shelf_scene()`).
- `fix_position_to_plane` puts them back before facing a shelf level
  (`table_or_shelf=false`): the core removes them after the pick, and in the sim the
  move to look at the place level, holding the bottle, was planned through the cabinet.
- `keyboard_input` -11/-12 use the measured heights only. If there is no scan, or the
  base moved since it, they survey the shelf first.
- A shelf pick clusters the object inside its measured compartment
  (`pick._shelf_band()`: the board under the object + 1 cm up to the underside of the
  board above - 1 cm), passed as the new optional `min_height`/`max_height` of
  `PickPerceptionService` -> `ClusterObjectFromPoint` (0 = the old point.z -/+ 0.3 m).
  With the old band the object merged with the boards and walls of a closed cabinet:
  6038 points, over the 6000 cluster limit, so "No object to cluster". Cropped: 253
  points, the bottle alone.
- The task manager scans on every cabinet visit (`dock_at` resets the flag), so the
  restored boxes always match where the base stopped. A failed scan keeps the
  previous heights; with none yet, the cabinet place fails.

Tunables are `grasp_generator` parameters under `shelf.`: the `ShelfParams` fields
(physical tolerances only), `min_range`/`max_range` (0.3/2.5 m, the ZED2 limits),
`depth_scale` (default `ZED_DEPTH_SCALE`; 1.0 for a simulated camera) and `dump_path`
(save the captured frames as `.npz` for offline tuning). Debug topics:
`/manipulation/shelf_scan_cloud` and `/manipulation/shelf_geometry` (latched, so RViz
always shows the last scan).

## Survey poses

Found in the Gazebo sim with MoveIt FK over a joint grid; joint1 stays at -90 and
joint5 > 0 tilts the camera down.

| Pose | Joints (deg) | Camera in base_link | Sees |
| --- | --- | --- | --- |
| `shelf_survey_high` | -90, -45, -90, 0, 20, 45 | (0.19, 0.06, 1.40), 23 deg down | upper boards from above, the underside of a board above the camera |
| `shelf_survey_low` | -90, -105, 0, 0, 15, 45 | (0.21, 0.06, 0.95), 48 deg down | under the upper boards into the low compartments |

The camera cannot go lower than ~0.95 m without reaching past x = 0.25 m, which puts
it closer than the ZED2 minimum depth to a shelf docked 15-20 cm away. From 1.35 m
alone the lowest boards show only 8-11 cm strips, hence the second view. The high
view looks only 23 deg down so the top of its image points ~11 deg up and it sees
the underside of boards above ~1.52 m. The closed top of the competition cabinet
(~1.40 m, from a robot photo) sits at camera height and is seen edge-on: it stays
undetected, and the arm cannot reach that high anyway. With joint5 = 40 (43 deg
down) the sim batch found the same boards in 7.7-10 s, against 13.6-16.5 s after the
change (the sim load also differed between the runs); reverting is an option.

Boards above the camera are only seen from below. A wide down-facing plane with no
surface right above it becomes a level with `seen_from_below = true`, height =
underside + default thickness, so it still bounds the compartment under it.

## Testing

Laptop: `pytest manipulation/packages/perception_3d/test/test_shelf_geometry.py`.

Simulation (see `simulation/frida_gz_sim/README.md`, "Shelf detection check"): three
different cabinets, each with and without objects, robot 15-20 cm away.
Results on the Orin, 2026-11-16:

| Scene | Truth (m) | Detected (m) | Scan |
| --- | --- | --- | --- |
| competition, objects / empty | 0.095 0.39 0.68 1.05 | 0.096 0.391 0.681 1.051 | 7.7-10.0 s |
| closed_three (closed top), objects / empty | 0.30 0.62 0.95 1.27 | 0.301 0.621 0.951 1.271 | 8.4-10.7 s |
| open_rotated (no side walls, 4 deg, 5 cm off), objects / empty | 0.12 0.45 0.80 1.15 | 0.121 0.451 0.801 1.151 | 8.7-11.4 s |

The scan time is the two arm moves plus captures; detection itself takes ~0.3 s.
Gazebo depth is noise-free, so the robot will be noisier: validate there.

Shelf pick (`run_shelf_pick.sh competition 2 bottle`, the -11 flow), 2026-11-16:

- Verified: level 2 faced at the measured 0.391 m, 7 shelf boxes in the scene for
  the whole pick, the bottle isolated by the compartment band, GPD candidates
  generated.
- The pick itself still fails: every GPD candidate is "grasp pose unreachable",
  with and without the shelf boxes (A/B with `noboxes`). The boxes are not the
  cause.
- MoveIt IK probe at the bottle (frontal approach): collision-free grasps exist at
  z 0.44-0.52 m with and without the boxes. The boxes only remove grasps within
  ~5 cm of the compartment ceiling and inside the level above, which the real
  boards would also block. Their contact with a 90-degree wrist roll is
  gripper/board 2, the same the octomap reports for the real board.
- Sim-only workarounds in the check:
  - The octomap filter is cut just above the object's board.
  - The ZED image is rotated 180 deg if YOLO misses the object.
  - From the level pose the wrist rolls the camera image upside down. Relevant
    for the real robot too.

Orin (manipulation area): rebuild `frida_interfaces` (new msg/srv, see the cache
wipe notes), then `./run.sh manipulation --build`. With the robot docked at the
cabinet run `keyboard_input` and use -15/-16 first (no motion), then -14. Validate
`shelf_survey_high/low` keep the camera >= 0.3 m from the shelf front.

## Shelf frontal grasp (pick)

A shelf pick first tries a frontal grasp built from the object's cluster
(`pick.shelf_frontal_grasps`), run by the cartesian strategy:

- **Grasp point.** The fingertip point is just past the object's middle, at mid
  height, and clear of the board and the ceiling.
- **Approach.** It comes in from the front, tilted 10, 0 and 20 deg down; a shelf with
  nothing above also tries 30 and 45 deg.
- **Gripper.** The fingers close sideways and the ZED points up.
- **Exit.** After closing, two straight MoveIt paths from where the gripper is: 2 cm
  up, then 15 cm back along the way in (`_shelf_exit`, `planner_id="cartesian"`).
  They are collision checked against the boxes and the held object, timed within the
  joint limits, and cut where the IK would jump more than 0.5 rad between 5 mm
  waypoints (a branch flip at a singularity). When a path is cut, the return to
  `table_stare` is planned around the boxes instead. The old xArm cartesian velocity
  lift tripped C24 (speed exceeds limit) on the real robot twice, at the wrist
  singularity (joint5 -13 deg) and with the elbow stretched (joint3 -155 deg), and the
  stop opened the gripper. At that second pose the MoveIt path is complete, its
  largest joint step between waypoints is 3.7 deg and it takes 1.3 s. GPD remains the
  grasp fallback.

The shelf boxes are grown 2 cm into the free space (`ShelfParams.padding`). Only the
board tops stay where they are. Planning flush against the measured faces left no
room for error, and in Gazebo the gripper ran into the cabinet front on paths MoveIt
accepted. `ManipulationTasks.pick_object` waits 150 s for shelf picks; a successful
one took ~65 s and the old 60 s limit cancelled it with the object in the gripper.

Sim results on 2026-11-16 (`run_shelf_pick.sh`, YCB mustard bottle, detection by the
sim's COCO YOLO). PASS means the bottle ended carried, at least 20 cm above its board.

| Cabinet | Shelf | Result | Scan + pick |
| --- | --- | --- | --- |
| competition_high (0.90 / 1.30 m) | 0.90 m | PASS once, then hung at the pre-grasp | 77 s |
| competition_high | 1.30 m (open top) | PASS, needed the steep approaches | 80 s |
| competition_tall (photo) | 0.69 m | PASS with padding (without it, 6 of 7 candidates hit the front) | 83 s |
| competition_tall | 1.05 m | PASS | 68 s |
| competition_tall | 0.39 m | FAIL: no frontal IK with the base 15-20 cm away | - |

The sim needs `xarm_velocity_sim.py` from the `sim/task-harness` branch for the
cartesian approach (`XARM_VELOCITY_SIM`); without it there is no `/xarm/robot_states`.

## Shelf place

Two fixes in `pipelines/place.py`, found with `PLACE_LEVEL=N run_shelf_pick.sh`
(pick on one shelf, place on another, as the task manager does):

- **Release over the board.** With a shelf scan (`RobotArm.shelf_extent()`), the
  release keeps the held object's centre at least 8 cm behind the board front and
  away from the back and sides (`_shelf_place_bounds`). Before, on the 1.30 m shelf,
  the reach clamp plus the 7 cm pull put the object on the front edge and it fell.
- **Keep the object upright.** The shelf place used a fixed EE quaternion that
  turned a frontally picked bottle upside down. It now keeps that approach but rolls
  so the object's "up", as held at the pick, points up (`_shelf_place_quat`; pick
  poses are `gripper_grasp_frame` poses, the place targets `link_eef`, 45 deg apart).
  After a top-down pick the old quaternion is kept.

Results on 2026-11-16, high cabinet, both directions PASS. The bottle ended upright
on the target board, inside the cabinet. Each run was a pick (36-41 s) plus a place
(34-37 s), 112 s in total with the scan.

## Benchmark (sim, 2026-11-17)

`competition_high` cabinet (boards 0.90 / 1.30 m), YCB mustard bottle, sim COCO YOLO,
on the Orin. Scan + pick on one shelf + place on the other, as `run_shelf_pick.sh`
with `PLACE_LEVEL`. Times in seconds; "-" where the step did not run.

| Approach | Passed | Scan | Pick | Place | Exit | Return after the pick |
| --- | --- | --- | --- | --- | --- | --- |
| Before the shelf scan: arena heights, no boxes, GPD | 2/4 | - | 46.7-68.1 | 37.5-41.2 | 2.8-4.8 | 14.4-19.1 |
| Shelf scan + frontal grasp, cartesian velocity exit (2026-11-16) | 2/4 | 13.8-15.7 | 35.7-49.2 | 36.9-46.2 | 2.1-11.1 | 12.7-19.3 |
| Now | 1/4 | 11.1-11.7 | 22.6-26.7 | 32.1-34.7 | 2.7-3.1 | 6.3-6.4 |
| Now, `shelf_acceleration` 0.5 + `shelf_place_straight` | 2/3, no aborts | 11.2-11.4 | 24.7-28.2 | 28.4-29.6 | 3.5-7.9 | 3.6-5.8 |

- Before the scan, every shelf-1 pick failed: without the compartment band the object
  merged with the boards ("Error clustering object", too many points).
- Now, the failures: twice the sim YOLO did not see the bottle from the level pose;
  twice a move to look at the 1.30 m level aborted on the controller's goal tolerance
  (once holding the bottle, which ended on the shelf); once the bottle tipped over on
  release. One exit was cut at a branch flip and the return planned around the boxes.
- The first two rows ran with the robot's own containers also on the Orin; "Now" ran
  with them stopped, so the times compare only roughly.

## Speed

Velocity is the MoveIt scaling factor (0-1); the constants are in
`manipulation_constants.py`, `pick.py` and `place.py`.

| Step | Before | Now |
| --- | --- | --- |
| Survey moves | 0.5 | 0.75 (`SHELF_SURVEY_VELOCITY`) |
| Wait at each survey pose | 0.7 s | 0.3 s (`SHELF_SURVEY_SETTLE`) |
| Wait at a level's viewing pose (task managers, keyboard) | 2-3 s | 1.5 s (`SHELF_LEVEL_SETTLE`) |
| Shelf exit | xArm cartesian velocity, 50 mm/s | one straight MoveIt path, 0.3 (1.3 s) |
| Return after a shelf pick | `front_stare` at 0.3, then `table_stare` at 0.5 | `table_stare` at 0.75, planned with the boxes |
| Place: release | 1.0 s | 0.5 s |
| Place: return | 0.5 | 0.75 |
| Straight-line paths on Jazzy | retimed 3x slower (a Humble workaround) | timed by MoveIt |
| Place: halfway release when the final pose fails | released 12 cm short of the board | removed; the next rung (+2 cm) is tried |

Unchanged: pre-grasp 0.5, the cartesian approach into the shelf at 40 mm/s, the
place moves at 0.5 / acceleration 0.3, and the gripper close settle (profiles).
`SHELF_LEVEL_SETTLE` and `SHELF_SURVEY_SETTLE` are the first numbers to raise if the
robot misses detections or scans a blurred cloud.

Two more speed-ups are `manipulation_core` parameters, off by default so a test
starts from the validated behaviour; switch them at runtime, one at a time:

```bash
ros2 param set /manipulation_core shelf_acceleration 0.5    # default 0.3 (PICK_ACCELERATION)
ros2 param set /manipulation_core shelf_place_straight true # default false
```

- `shelf_acceleration`: MoveIt acceleration scaling for every planned move of a
  shelf pick or place (pre-grasp, exit, return, pre-place, place).
- `shelf_place_straight`: the place goes from the pre-place into the compartment as
  one straight MoveIt path (0.3, collision checked); where the line is cut, the same
  rung uses the usual free plan. In the sim the line was cut on every rung into the
  0.90 m board (29-38 %) and went through into the 1.30 m one (3.6-3.8 s, from 6.9 s).

## Known limits / not done yet

- Real robot, 2026-11-16: the scan found the 0.95 and 1.31 m boards (a 0.78 m one only
  sometimes); a can was picked from both and placed, and a cereal box picked from
  1.31 m and placed on 0.95 m and back. Open there: an apple is too short for the
  frontal grasp (6 cm board clearance), the cluster height on the 1.31 m shelf varied
  between runs (2-8 cm), and a sprite_can false positive.
- The lowest reachable shelf (~0.39 m) has no frontal grasp with the base 15-20 cm
  away; the base would need to stand further back or the approach to change.
- A pre-grasp can still hang in the sim (controller blocked, likely the oversized
  Gazebo gripper collision box), costing 7-20 s per failed candidate.
- Time: see "Speed" below; the "face the level" move before the pick is still there.
- The task manager still assigns objects to levels with the old 0.20/0.05 m
  thresholds instead of `level_for_height`, and still sweeps each level to find
  objects (the slow part of the original 60-120 s).
- The shelf place puts `link_eef` 10 cm above the board whatever the grasp height: a
  tall, thin object grasped low drops a few centimetres and can tip (sim bottle). The
  pick height is known and could set it.
- The move to look at a 1.30 m level sometimes aborts in the sim (goal tolerance), with
  or without the boxes; the sim load (load average ~20 on the Orin) may be the cause.
- Thin posts of an open bookshelf are reported as side walls (conservative).
- A level is numbered from the lowest board seen: if the bottom board is hidden,
  numbering shifts. The low survey pose sees it in every sim scene.
- A board seen only from below gets the default thickness (1.8 cm), so its surface
  height can be off by the real thickness difference.
- The `competition_tall` sim scene comes from a photo: depth (0.35 m) is assumed.
