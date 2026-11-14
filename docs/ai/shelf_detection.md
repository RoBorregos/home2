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

## How it works

1. The caller moves the camera (two survey poses) and calls the service with
   `capture=true` at each; frames accumulate in `base_link`.
2. `detect=true`: crop to the shelf footprint, 1 cm voxels, PCA normals oriented by
   camera height. Up-facing horizontal points -> Z histogram -> one peak per board
   surface. Peaks narrower than 60 % of the shelf width or with gaps are objects, not
   boards. Down-facing peaks give the board thickness and the ceiling (`top_z`).
   Vertical planes at the lateral/far extremes are the side and back walls.
3. `add_collision_objects=true` publishes boards, walls and top as boxes
   (`frida_shelf_*`) to the MoveIt scene. With `capture=false, detect=false` it only
   re-publishes the last result (the pick/place core wipes the scene after each task).

Tunables are ROS parameters mirroring `ShelfParams` (physical tolerances only).
Debug topics: `/manipulation/shelf_scan_cloud`, `/manipulation/shelf_geometry`.

## Testing

Laptop: `pytest manipulation/packages/perception_3d/test/test_shelf_geometry.py`.

Orin (manipulation area): rebuild `frida_interfaces` (new msg/srv, see the cache
wipe notes), then `./run.sh manipulation --build`. With the robot docked at the
cabinet run `keyboard_input` and use -15/-16 first (no motion), then -14. Validate
`shelf_survey_high/low` keep the camera >= 0.3 m from the shelf front.

## Not done yet

Task manager integration (replace `get_shelf_levels(arena)` with the service,
assign detections to levels with `level_for_height`, restore the shelf boxes before
each shelf pick/place).
