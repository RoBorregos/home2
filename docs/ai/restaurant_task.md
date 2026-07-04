# Restaurant task — architecture & conventions (RoboCup@Home 2026, sec. 5.5)

Read this before touching `restaurant_task_manager.py` or the restaurant vision/nav
pieces. Rulebook: `master.pdf` §5.5 (15 min, unknown venue, single restart, any
contact with people/furniture = e-stop, pre-mapping = disqualification).

## Task flow (one cycle per calling customer)

```
WAIT_FOR_BUTTON → START (save bar/origin pose)
→ WAIT_FOR_CALL      camera pan sweep [0,-45,45] + omni in-place rotations (120°×2)
                     then small forward steps; caller must be seen TWICE within
                     0.8 m (persistence) to filter phone-raisers
→ APPROACH_CUSTOMER  nav.approach_point(caller, standoff 2.0) — costmap-free goal,
                     base ends FACING the caller; re-detect + re-approach if > 3.5 m
→ SCAN_TABLES        vision.customer_tables(); EVERYTHING converted to MAP frame
                     immediately (robot moves later; camera-frame points would be
                     mis-transformed). Serve table = closest to the caller.
                     3 failed scans → fall back to serving the caller point itself.
→ TAKE_ORDERS        approach_point(table, 0.6); per customer: look_at(map point)
                     then hri.take_order() (2 items, partial 1-item orders kept)
→ NAVIGATE_TO_BAR    return_to_origin(inverse) + dock (nav_pose first, offset 0.32)
→ SAY_ORDER_TO_BARMAN
→ DELIVER_ORDER      per item: pick at bar → carry_pose → saved approach_pose or
                     approach_point → dock → place; bar round-trip between items
→ back to WAIT_FOR_CALL (cycle state reset) — loops until the operator stops it.
```

## Contracts used (all through subtask managers)

- **nav** (`nav_central`): `GetRobotPose`, `GoToPose`, `ApproachPoint`, `DockTable`
  (all registered under `Task.RESTAURANT` in `nav_tasks.py`; blocking spins use
  `NAV_GOAL_TIMEOUT` = 90 s so a dead nav service can't hang the FSM).
- **vision**: `Customer` (`/vision/get_customer`, waving+sitting; the table scan
  passes `include_non_waving=True`), `CustomerTables` (`/vision/customer_tables`),
  `DetectionHandler` for bar objects. Client timeouts: get_customer 20 s,
  customer_tables 60 s — must exceed the nodes' inner moondream budgets
  (sitting/calling checks ~5 s each, table points 20 s, customers 25 s).
- **hri**: `take_order()` (asks 2 items, confirms each, returns partial `[first]`
  if the second fails), display via `publish_display_topic`.
- **manipulation**: `carry_pose` (drive), `table_stare` (pick/place), `pan_to`.

## Key invariants (things that will break silently if undone)

1. **Map frame at scan time.** Detections are camera-frame `PointStamped` with
   stamp 0 (= "latest" TF). They are only valid while the robot is still at the
   detection pose — `SCAN_TABLES` converts to map immediately; `look_at()`
   recomputes bearings from map points at use time. Never store camera-frame
   points across a navigation.
2. **Omni base rotation.** `approach_point` leaves the base facing the target and
   `look_at()` rotates the base when the bearing exceeds ±90° pan — don't aim the
   arm past ±90° in a crowded venue.
3. **Wrist camera feeds the costmap.** The STVL layer marks tabletops from
   `/point_cloud` (lidar only sees table LEGS). Return the arm to `carry_pose`
   before every nav goal; the restaurant overlay keeps voxels 15 s
   (`nav2_omni_restaurant.yaml`, deep-merged over `nav2_omni.yaml` via the
   `nav2_overlay_file` arg of `nav2_omni.launch.py`).
4. **Display is an MJPEG stream** (web_video_server): single-shot image publishes
   are invisible. `restaurant_commands` republishes the annotated tables image at
   10 Hz; `customer_node` republishes at 10 Hz on `/vision/customer` a side-by-side
   composite of ALL confirmed callers (scored: detect calling customer ×2), and
   labels every caller in the full frame on the tracker topic.
5. **Docking**: always through the task manager's `_dock_to_table()` — it tucks
   the arm in `nav_pose` first (arm must be inside the footprint at the 0.32 m
   dock offset), then `dock_table(offset=0.32)`.

## False-customer filtering (public raising phones in the recording zone)

Layered in `customer_node.py` + task manager:
- range gate `max_customer_range` (param, default 5 m) + depth validity;
- waving AND sitting (keypoints, moondream fallback for sitting);
- moondream arbitration `verify_calling_with_moondream` (param, default on):
  "raising a hand to call a waiter, not holding a phone";
- two-hit persistence (task manager sweep);
- optional `restrict_search_sector` param on the task manager node: reject
  detections behind the bar's initial facing (enable on site if the public zone
  is behind the start pose).

## Venue tuning params (task manager node, set with `ros2 param set`)

- `bar_return_yaw_deg` (default 180): heading when arriving back at the bar,
  degrees CCW relative to the START orientation — 180 turn around, 90 face
  left, -90 face right, 0 face as started. Pick per venue by where the barman
  stands.
- `restrict_search_sector` (default false): see above.

## Run images

Confirmed-caller frames/crops and table-scan images are saved to
`/workspace/src/restaurant_runs/` (= `<repo>/restaurant_runs/` on the host,
gitignored; `save_dir` param on both vision nodes). They double as referee
evidence — the rulebook grants partial points for clearly identifying a detected
caller even if the table is never reached.

## Known TODOs / verify on the robot

- `/point_cloud` must actually be published by the ZED bring-up — without it the
  costmap has NO tabletop protection (invisible failure; lidar covers most arena
  furniture but not restaurant tables).
- `carry_pose` (joint5 = -12°) must see the room at table height — the whole
  customer search runs in this pose.
- Tray transport (2×200 extra) not implemented; per-item delivery only.
