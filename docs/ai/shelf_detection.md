# Model-free shelf detection

Replaces the per-arena heights in `xarm_utils/config/shelf_levels.json` (now a
fallback) with a live scan from the eye-in-hand ZED2.

## Pieces

| Piece | Where |
| --- | --- |
| Geometry (pure numpy, tested) | `manipulation/packages/perception_3d/perception_3d/shelf_geometry.py` |
| ROS node, `DetectShelf` service | `manipulation/packages/perception_3d/scripts/shelf_detector.py` |
| Interfaces | `frida_interfaces/manipulation/srv/DetectShelf.srv`, `msg/ShelfLevel.msg` |
| Constants / survey poses | `manipulation_constants.py` (`DETECT_SHELF_SERVICE`, `SHELF_SURVEY_POSES`), `xarm_configurations.py` (`shelf_survey_high/low`) |
| Manual test | `keyboard_input.py` options -14 (survey + detect), -15 (capture here), -16 (detect) |
| Task manager | `ManipulationTasks.detect_shelf_levels()`; `pickandplace_task_manager.py` `_ensure_shelf_heights()` (flag `use_dynamic_shelf_detection`) |
| Sim check | `simulation/frida_gz_sim`: `shelf_scenes.py`, `scripts/sim_shelf_check.py`, `scripts/run_shelf_checks.sh` |

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
  re-publishes the last scan. The usual wipes after the pick remove it again.
- The shelf place restores it before reaching and always removes it at the end
  (`remove_shelf_scene()`). The slab at the JSON `compartment_height` is only used
  when no scan can be restored.
- `keyboard_input` -11/-12 use the measured heights only. If there is no scan, or the
  base moved since it, they survey the shelf first.
- A shelf pick clusters the object inside its measured compartment
  (`pick._shelf_band()`: the board under the object + 1 cm up to the underside of the
  board above - 1 cm), passed as the new optional `min_height`/`max_height` of
  `PickPerceptionService` -> `ClusterObjectFromPoint` (0 = the old point.z -/+ 0.3 m).
  With the old band the object merged with the boards and walls of a closed cabinet:
  6038 points, over the 6000 cluster limit, so "No object to cluster". Cropped: 253
  points, the bottle alone.
- `publish_boxes:=false` on `shelf_detector` keeps the boxes out of the scene (the
  place then falls back to the JSON ceiling slab). A field switch, also used for the
  A/B check below.
- The task manager scans on every cabinet visit (`dock_at` resets the flag), so the
  restored boxes always match where the base stopped. A failed scan keeps the
  previous heights; before the first success that is still the arena JSON.

Tunables are ROS parameters mirroring `ShelfParams` (physical tolerances only),
plus `min_range`/`max_range` (0.3/2.5 m, the ZED2 limits), `depth_scale` (default
`ZED_DEPTH_SCALE`; 1.0 for a simulated camera) and `dump_path` (save the captured
frames as `.npz` for offline tuning). Debug topics: `/manipulation/shelf_scan_cloud`,
`/manipulation/shelf_geometry`.

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
- **Exit and fallback.** After closing, the arm lifts 2 cm and backs out 15 cm in a
  straight line (`_shelf_exit`). GPD remains the fallback.

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

## Known limits / not done yet

- Not yet run on the real robot (ZED noise, real hand-eye calibration).
- The lowest reachable shelf (~0.39 m) has no frontal grasp with the base 15-20 cm
  away; the base would need to stand further back or the approach to change.
- A pre-grasp can still hang in the sim (controller blocked, likely the oversized
  Gazebo gripper collision box), costing 7-20 s per failed candidate.
- Time: scan ~13 s + pick ~35-45 s, versus the 20-30 s goal. The cheapest cuts are
  skipping the "face the level" move, one return pose instead of front_stare plus
  table_stare, and faster accelerations at the shelf.
- The task manager still assigns objects to levels with the old 0.20/0.05 m
  thresholds instead of `level_for_height`, and still sweeps each level to find
  objects (the slow part of the original 60-120 s).
- Thin posts of an open bookshelf are reported as side walls (conservative).
- A level is numbered from the lowest board seen: if the bottom board is hidden,
  numbering shifts. The low survey pose sees it in every sim scene.
- A board seen only from below gets the default thickness (1.8 cm), so its surface
  height can be off by the real thickness difference.
- The `competition_tall` sim scene comes from a photo: depth (0.35 m) is assumed.
