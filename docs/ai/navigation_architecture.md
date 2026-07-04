# FRIDA — Navigation Stack & Task-Manager Architecture

> **Purpose of this file.** This is a context primer for AI agents (Claude) and new
> contributors working on **navigation** and its **integration with the task manager**.
> Read this *instead of* re-discovering the stack from scratch every session. It is
> intentionally dense: service/action/topic/file names are the high-value parts —
> use them as search anchors (`grep`/`Read`) to jump straight to the code.
>
> Scope: `navigation/`, `task_manager/`, the `frida_interfaces/navigation` +
> `frida_interfaces/task_manager` contracts, and how everything is launched.
> Last mapped: 2026-06 against branch `Hric_flag`.

---

## 0. TL;DR (read this first)

- **FRIDA** is a RoboCup@Home service robot. The codebase is split into **Areas**:
  `vision`, `hri`, `navigation`, `manipulation`, and `integration` (the task manager).
  Each area is a set of ROS 2 packages that runs in its **own Docker container** and
  talks to the others over **CycloneDDS** (optionally zero-copy SHM via iceoryx/RouDi).
- **The task manager is the brain.** A per-task FSM node calls into four
  **subtask managers** (`nav`, `vision`, `manipulation`, `hri`). Each subtask manager
  is a thin ROS client wrapper. Navigation logic itself lives in the `navigation/` area.
- **The navigation entrypoint node is `nav_central`** (`navigation/packages/nav_main/scripts/nav_central.py`,
  ~1000 lines). It wraps **Nav2** + a **SLAM backend** and exposes a small, stable
  **service/action API**. Everything outside navigation talks to *that API*, never to
  Nav2 directly.
- **The task-manager ↔ navigation contract is 5 services** (see §2). If you only
  remember one thing: `MoveLocation` is "drive to a named place from `areas.json`".
- **Two robot bases / two SLAM backends** are supported by the same node:
  - **omnibase** (holonomic, current/primary) → `slam_toolbox` (2D lidar) + MPPI-Omni.
  - **dashgo** (differential, legacy) → `rtabmap` (RGBD) + MPPI-DiffDrive.
- **You run everything with `./run.sh`** from the repo root; it auto-detects the
  platform (cpu / cuda / l4t-Jetson) and brings up the right containers. See §6.

```
┌─────────────────────────────────────────────────────────────────────┐
│  TASK MANAGER  (integration area)                                    │
│  per-task FSM node  ──►  SubtaskManager                              │
│                            ├─ nav   (NavigationTasks)  ◄── §2        │
│                            ├─ vision (VisionTasks)                   │
│                            ├─ manipulation (ManipulationTasks)       │
│                            └─ hri   (HRITasks)                       │
└───────────────┬─────────────────────────────────────────────────────┘
                │  ROS 2 services (the 5 nav services)
                ▼
┌─────────────────────────────────────────────────────────────────────┐
│  NAVIGATION area                                                     │
│  nav_central  ──►  Nav2 (planner + MPPI controller + BT navigator)  │
│      │             SLAM: slam_toolbox (omni) | rtabmap (dashgo)      │
│      ├─ table_docker        (perpendicular table approach)          │
│      ├─ adaptive_goal_pub   (re-route around blocked goals)         │
│      ├─ node_monitor        (CPU/mem/GPU health)                    │
│      └─ map_context/nav_ui  (areas.json, keepout masks, RViz UI)    │
│  Drivers: omnidriver | dashgo_driver · sllidar · ira_laser_tools    │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 1. The layered control model

There is **no central blackboard or global state machine across areas**. Control is:

1. **A task FSM** (one Python `rclpy.Node` per competition task, e.g. `gpsr_task_manager.py`).
   It loops `rclpy.spin_once(); node.run()` and `run()` is a big `if/elif self.current_state`.
2. **`SubtaskManager`** (`task_manager/task_manager/utils/subtask_manager.py`) bundles the
   four area clients and is shared by the task node:
   ```python
   self.subtask_manager.nav.move_to_location("kitchen", "table")
   self.subtask_manager.manipulation.pick_object("apple")
   self.subtask_manager.hri.say("Going to the kitchen")
   ```
3. **Subtask managers** (`task_manager/task_manager/subtask_managers/*_tasks.py`) are
   ROS clients. They convert a Python method call into a ROS service/action call,
   block on the future, and return a uniform `(Status, payload)` tuple.
4. **Area nodes** (e.g. `nav_central`) do the real work and own the hardware/stack.

**Uniform return convention:** every subtask-manager method returns
`(Status, payload)` where `Status` is the enum in `task_manager/utils/status.py`
(`EXECUTION_SUCCESS=1`, `EXECUTION_ERROR=0`, `TARGET_NOT_FOUND=2`, `TIMEOUT=4`,
`MOCKED=5`, `TERMINAL_ERROR=-1`, `SERVICE_CHECK=3`). FSM states branch on this.

**Two decorators make area calls resilient** (`task_manager/utils/decorators.py`):
- `@service_check("client_attr", fallback_return, timeout=...)` — short-circuits with
  `fallback_return` if the service/action server isn't up. Prevents hard hangs.
- `@mockable(return_value=..., delay=...)` — when the subtask manager is built with
  `mock_data=True`, returns a canned value (optionally after a delay) **without touching
  ROS**. This lets you run/debug a whole task's logic with zero hardware. `return_value`
  may be a `lambda self: ...`.

Stacked, they read top-down "mock first, else check service, else call":
```python
@mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=5)
@service_check("move_to_location_srv", (Status.EXECUTION_ERROR, "Service not started"))
def move_to_location(self, location, sublocation): ...
```

**Logging:** use `CLog` (`task_manager/utils/colored_logger.py`), not `print`.
`CLog.nav(node, "INFO"/"SUCCESS"/"WARN"/"ERROR", msg)` — nav is cyan, manip yellow,
vision magenta, hri green, fsm white.

---

## 2. ⭐ The Task-Manager ↔ Navigation contract (the connection)

This is the single most important section. Everything the task manager can ask
navigation to do goes through **`NavigationTasks`**
(`task_manager/task_manager/subtask_managers/nav_tasks.py`). It opens exactly **five
service clients**, whose names come from `frida_constants.navigation_constants`:

| `NavigationTasks` method | ROS service constant | Srv type (`frida_interfaces`) | What it does |
|---|---|---|---|
| `move_to_location(location, sublocation="")` | `MOVE_LOCATION_SERVICE` | `MoveLocation` | **Drive to a named area/sublocation** from `areas.json`. Empty sublocation → `"safe_place"`. Returns `(Status, error_str)`. |
| `get_path_info(location_b, sublocation_b, location_a="", sublocation_a="")` | `NAV_QUERY_SERVICE` | `NavQuery` | **Plan-only**: real path distance (m) between two areas (or from current pose) **without moving**. Returns `(Status, {"distance": float})`. Used to pick the nearest of several goals. |
| `dock_table(offset=0.0)` | `DOCK_TABLE_SERVICE` | `DockTable` | **Perpendicular-approach** the table/shelf in front of the robot for manipulation. `offset` = desired front gap (m). |
| `check_door()` | `CHECK_DOOR_SERVICE` | `CheckDoor` | Blocks until the door in front opens (lidar-based). Returns when open. |
| `retrieve_areas()` | `AREAS_SERVICE` | `MapAreas` | Dump the live `areas.json` (named locations + sublocations) as a dict. Falls back to the on-disk backup if the service is down. |

Notes that matter when editing this boundary:
- **`areas.json` is the shared map vocabulary.** Locations like `"kitchen"` and
  sublocations like `"table"`/`"safe_place"` are defined in
  `frida_constants/map_areas/areas.json` (loaded as a backup in `NavigationTasks.__init__`,
  served live by `nav_central`'s `MapAreas` service). Task code refers to places **by
  name**, never by raw coordinates.
- The service **names** are constants (`navigation_constants.py`) — both `nav_central`
  (server) and `nav_tasks.py` (client) import the same symbol, so renaming is one-sided-safe.
- There is also a `Move.action` defined in `frida_interfaces/navigation/`, but the
  **task-manager path uses the `MoveLocation` *service***, not that action. `nav_central`
  itself uses Nav2's `NavigateToPose` action internally (see §3).
- All five methods are `@mockable`/`@service_check`-wrapped, so a task can run with
  `mock_data=True` and navigation stubbed out.

---

## 3. `nav_central` — the navigation orchestrator

File: `navigation/packages/nav_main/scripts/nav_central.py` (node class `Nav_Central`).
It is the **only** thing other areas talk to. It wraps Nav2 + SLAM, monitors health,
auto-restarts failed components, and chooses base/backend from launch params.

### 3.1 Server API it exposes (the "north" interface to the rest of the robot)

**Services**
| Service (constant) | Type | Role |
|---|---|---|
| `MOVE_LOCATION_SERVICE` | `MoveLocation` | Resolve name→pose, retreat-if-docked, send Nav2 goal, report success. |
| `NAV_QUERY_SERVICE` | `NavQuery` | Plan a path (no motion) and return distance/time. Uses `ComputePathToPose`. |
| `DOCK_TABLE_SERVICE` | `DockTable` | Forward to the `table_docker` dock service with a per-call front offset. |
| `CHECK_DOOR_SERVICE` | `CheckDoor` | Watch `/scan` for the door in front to open. |
| `AREAS_SERVICE` | `MapAreas` | Serve `areas.json`. |
| `RESUME_NAV_SERVICE` | `std_srvs/Empty` | Manually resume Nav2 (+ rtabmap) after a pause. |

**Actions (Nav2, re-exposed / used internally)**
| Action (constant) | Type | Role |
|---|---|---|
| `GOAL_NAV_ACTION_SERVER` | `nav2_msgs/NavigateToPose` | Primary "reach this pose" goal. |
| `COMPUTE_PATH_ACTION_SERVER` | `nav2_msgs/ComputePathToPose` | Path-planning only (backs `NavQuery`). |

**Topics published** (latched, `TRANSIENT_LOCAL`)
- `/nav/current_goal` (`geometry_msgs/PoseStamped`) — current goal; consumed by the arm
  pointer so the camera can aim at where the robot is heading.
- `/nav/goal_active` (`std_msgs/Bool`) — whether a goal is currently being pursued.

**Topics subscribed**
- `/nav/arm_ready` (`std_msgs/Bool`) — arm pointer says "camera aimed, proceed".
- `INITIAL_POSE_TOPIC` (`PoseWithCovarianceStamped`) — initial pose from RViz / `nav_ui`.
- `SCAN_TOPIC` = `/scan` (`LaserScan`) — used by the door check.

### 3.2 Clients it calls (the "south" interface to the stack it manages)

- **SLAM control:** `RTAB_PAUSE_SERVICE` / `RTAB_RESUME_SERVICE` (dashgo/rtabmap path).
- **Nav2 lifecycle:** `NAV2_LIFECYCLE_SERVICE` (`nav2_msgs/ManageLifecycleNodes`) to
  pause/resume the Nav2 lifecycle (e.g. when TF/sensors drop out — see monitoring).
- **Table docking:** `DOCK_SERVICE` / `UNDOCK_SERVICE` (`std_srvs/Trigger`) on the
  `table_docker` node, plus `/table_docker/set_parameters` to set the per-location offset.
  `nav_central` **auto-undocks before any new goal** (`_retreat_if_docked`).
- **Costmap clearing:** `/global_costmap/clear_entirely_global_costmap` and
  `/local_costmap/clear_entirely_local_costmap` (recovery).

### 3.3 Self-healing / monitoring

A background monitor (`_monitoring`) watches the **TF tree** (baseline'd at startup):
static frames must keep being published and dynamic frames must stay fresh (≤ ~2 s).
If a required frame (e.g. `link_eef`, the end-effector) or a sensor disappears, it
**pauses Nav2** (and rtabmap) and resumes when things recover, instead of letting Nav2
plan on stale data. This is why navigation "freezes then continues" sometimes — it is
deliberate, not a crash.

---

## 4. Dual base / dual backend (one node, two robots)

Selected by launch args `default_base:={omnibase|dashgo}` and `nav_type:={2d|3d}`.

| Aspect | **omnibase** (primary) | **dashgo** (legacy) |
|---|---|---|
| Base kinematics | Holonomic / mecanum (vx, vy, wz) | Differential drive (vx, wz) |
| Wheel driver | `omnidriver` (ODrive) → `/odrive/odom`, `/odrive/imu` | `dashgo_driver` → `dashgo_odom`, `imu` |
| SLAM / map | `slam_toolbox` async (2D lidar) | `rtabmap` RGBD (SuperPoint+SuperGlue, GTSAM) |
| Localization | `slam_toolbox` localization vs `.posegraph` | `rtabmap` loop closure vs `.db` |
| Nav2 controller | **MPPI `Omni`** model, vx/vy≤0.7, wz≤1.5 | **MPPI `DiffDrive`**, vx≤0.5, vy=0 |
| Lidar | RPLIDAR C1 (360°) | SLLIDAR S-series |
| Odom→base_link | EKF over odrive odom+imu | EKF over dashgo odom+imu |
| `map→odom` published by | slam_toolbox | rtabmap CoreWrapper |

**Planner** in both cases is Nav2 `GridBased` (NavFn/Smac-style global planner); the
**controller** is always **`nav2_mppi_controller::MPPIController`** with the motion model
swapped. Critics include Obstacles/Cost/Goal/PathAlign/PathFollow/PathAngle/
PreferForward/GoalAngle/Constraint. The omni goal tolerance is loose (`xy_goal_tolerance≈0.35`)
because we navigate to *zones*, not exact points.

> Practical note (from field tuning): on the Orin, nav nodes are pinned to **cpuset 0–3**
> and the rest of the stack to 4–11. MPPI and slam_toolbox are CPU-bound and **cannot**
> move to GPU; nvblox-style costmapping is the only part that could. The MPPI goal-tuning
> + cpuset partition is what fixed the earlier freeze-then-stutter behaviour.

---

## 5. Navigation packages & key files

All under `navigation/packages/`. Ignore `build/ install/ log/ __pycache__/`.

### 5.1 `nav_main` (the core package)

**Scripts** (`nav_main/scripts/`)
- `nav_central.py` — orchestrator (see §3). **Start here for any nav change.**
- `table_docker.py` — perpendicular dock to a table/shelf for the omnibase.
  Services `DOCK_SERVICE` / `UNDOCK_SERVICE` / `DOCK_PREVIEW_SERVICE` (`Trigger`);
  RANSAC line-fit on accumulated scan/cloud locked in `odom`, then closed-loop PID on
  yaw/lateral/distance with a live-lidar safety stop. Publishes `/table_docker/markers`.
- `adaptive_goal_publisher.py` — when a goal cell is blocked, finds a reachable nearby
  goal (radial sample → raycast → BFS). Subscribes `/global_costmap/costmap`,
  `/adaptive_nav/original_goal`, `/vision/tracking_results`; publishes `/goal_update`,
  `/adaptive_nav/goal_blocked`. Pairs with the `navigate_adaptive_goal.xml` BT.
- `node_monitor.py` — publishes `system/node_monitor` (`MonitorReport`) with per-node
  CPU/mem/GPU (pynvml / jtop / sysfs). Its watch-list is configurable (default mentions
  amcl/bt_navigator/controller_server/etc. — that list is **not** proof AMCL is active;
  the real localizer is slam_toolbox/rtabmap).
- `launch_nav.py` — TUI log multiplexer for the launched processes (press 0–5 to switch
  between nav_central / nav2 / rtabmap / hardware / all / errors views).

**Launch** (`nav_main/launch/`)
- `task_launch/general_navigation.launch.py` — **main runtime entrypoint.** Starts
  `nav_central` + `nav_ui`, then includes the base-specific bringup (omni_setup/* or
  dashgo_base/*) and the SLAM/localization launch.
- `task_launch/mapping.launch.py` — build/update a map (SLAM mapping mode, `nav_central`
  with `mapping=true`, skips loading Nav2).
- `task_launch/{gpsr_hric,restaurant}.launch.py` — per-task nav presets.
- `omni_setup/` — `omni_basics` (EKF + lidar merge), `lidar_setup` (merge 2+ lidars→/scan),
  `slam` (slam_toolbox mapping), `localization` (slam_toolbox localization), `nav2_omni`
  (Nav2 container, MPPI-Omni).
- `dashgo_base/` — `nav_basics` (DashgoDriver+EKF+lidar), `rtabnav2` (rtabmap + Nav2),
  `dualshock_cmd_vel` (PS controller teleop), `monitor`.

**Config** (`nav_main/config/`)
- `omni_config/nav2_omni.yaml` — omnibase Nav2 (MPPI-Omni; obstacle layer from 360° scan +
  `rgbd_obstacle_layer` from ZED cloud; odom `/odrive/odom`).
- `omni_config/nav2_omni_keepout.yaml` — keepout/virtual-obstacle filter overlay
  (editable from `nav_ui`).
- `omni_config/mapper_params_online_async.yaml` / `mapper_params_localization.yaml` —
  slam_toolbox mapping vs localization.
- `nav2_standard.yaml` — dashgo Nav2 (MPPI-DiffDrive).
- `rtabmap/rtabmap_{mapping,localization,follow,default,original}_config.yaml` — rtabmap
  modes (RGBD remapped to ZED, SuperPoint features, GTSAM optimizer, DB in
  `/workspace/rtabmapdbs/`).

**Behavior trees** (`nav_main/bt/`)
- `navigate_to_pose_w_replanning_and_recovery.xml` — default goal BT (replan @2 Hz,
  4 retries, recovery = clear costmaps + wait + backup).
- `navigate_adaptive_goal.xml` — uses a `GoalUpdater` (fed by `adaptive_goal_publisher`)
  for blocked-goal scenarios.
- `follow_dynamic_point.xml` — person-following (dynamic point goal).

### 5.2 Driver / support packages

- `omnidriver/` — ODrive bridge for the holonomic base; consumes `cmd_vel` (TwistStamped),
  publishes `/odrive/odom`, `/odrive/imu`.
- `dashgo_driver/` — serial bridge for the legacy Dashgo differential base.
- `sllidar_ros2/` *(submodule)* — RPLIDAR driver → `/scan`.
- `ira_laser_tools/` *(submodule)* — `laserscan_multi_merger`: merge several `/scan*`
  into one `/scan`.
- `map_context/` — `nav_ui.py`: RViz-side map annotation UI; owns the **areas** JSON,
  serves static maps, and edits keepout masks.
- `PlayStation-JoyInterface-ROS2/` *(submodule)* — DualShock teleop (`/joy` → `cmd_vel`).

### 5.3 TF tree (omnibase)
```
map ── (slam_toolbox) ──► odom ── (EKF) ──► base_link ──► {laser, camera_link, link_eef, ...}
```
`link_eef` (arm end-effector) is monitored by `nav_central`; if it vanishes nav pauses.

---

## 6. How it is built, launched & deployed

Everything is containerized; `./run.sh` (root) + `lib.sh` orchestrate it. **Never run
nodes natively for normal use — go through `run.sh`.**

### 6.1 `run.sh`
- **Platform autodetect:** `/etc/nv_tegra_release` → `l4t` (Jetson); else `nvidia-smi`
  → `cuda`; else `cpu`. Picks the base compose `docker/{cpu,cuda,l4t}.yaml` and sets
  `DOCKER_RUNTIME` (`nvidia` for cuda/l4t, `runc` for cpu).
- **Single area:** `./run.sh navigation` (also `vision|hri|manipulation|integration|zed|display|roudi|simulation`).
- **Build interfaces once:** `./run.sh frida_interfaces` → prebuilt into
  `docker/frida_interfaces_cache/{build,install,log}`, mounted read-only by every area
  (saves a 2–3 min rebuild per container).
- **Competition tasks** (`--hric --ppc --gpsr --dlc --restaurant --finals`): launch
  *all* needed areas in parallel inside a `screen` session named `frida`
  (`screen -r frida` to attach; remote areas like HRI are SSH'd to the other Orin).
- **Common flags:** `--build` (colcon build inside container), `--build-image`
  (rebuild image), `--recreate`, `-d` (detached); global `--stop` / `--down` / `--clean`.

### 6.2 Navigation container specifics (`docker/navigation/`)
- Mounts the lidar + base serial devices (e.g. `/dev/ttyUSBlidar2`, `/dev/ttyUSBStm32`);
  `setup-USB.sh` stabilizes the device symlinks first.
- `network_mode: host`, `ipc: host`, `cpuset: 0-3`.
- Reads `MAP_NAME` (default `lab_23_march.db`; set via `./run.sh --update-map <file>`).
- Sub-flags pick the launch file: `--mapping` → `mapping.launch.py`, `--move` →
  `omni_basics.launch.py` (drivers only), default/task → `general_navigation.launch.py`.

### 6.3 DDS / inter-process comms
- **CycloneDDS** is the middleware (`cyclonedds.xml`; interface set by
  `scripts/setup_cyclonedds.sh`, stored in `/etc/cyclonedds.env`).
- **Zero-copy SHM** via **iceoryx RouDi** is enabled only when `CYCLONE_SHM=1`
  (auto on Jetson/L4T). `lib.sh ensure_roudi()` starts the `roudi` container (memory
  pools in `roudi_config.toml`) when SHM + a heavy area (zed/vision/navigation/display)
  is up. On cpu/cuda dev machines it's plain UDP loopback (`CYCLONE_SHM=0`).
- All containers use `network_mode: host` + `ipc: host` so DDS/SHM crosses container
  and machine boundaries.

---

## 7. Custom interface reference (`frida_interfaces/`)

### `frida_interfaces/navigation/`
- **srv**: `MoveLocation` (`location, sublocation → success, error`),
  `NavQuery` (`location_a, sublocation_a, location_b, sublocation_b → success, distance_meters, error`),
  `DockTable` (`offset → success, error`),
  `MapAreas` (`→ areas` JSON string).
- **action**: `Move` (`location → success, feedback`) — *defined but the task-manager
  bridge uses the `MoveLocation` service instead.*
- **msg**: `MonitorReport` (`NodeStatus[]`), `NodeStatus` (`name, cpu_usage, memory_usage, gpu_usage`).

### `frida_interfaces/task_manager/` (srv only)
`CheckDoor` (`→ status`), `FollowFace` (`follow_face → success`),
`HearMultiThread` (`start/stop → stopped`), `PointTransformation` (`point, target_frame → transformed_point`),
`PositionGet` (`→ posex, posey, status`), `ReturnLocation` (`→ location, nearest_locations[]`),
`WaitForControllerInput` (`button, timeout → success`).

> The other interface groups (`hri/`, `vision/`, `manipulation/`) are large (~20–28 srv
> each) and out of scope here; browse them when touching those areas. **Always** prefer
> these custom interfaces over ad-hoc messages (project convention).

---

## 8. Where to start for common changes

| I want to… | Start at |
|---|---|
| Add a new nav capability the task manager can call | add srv in `frida_interfaces/navigation`, server in `nav_central.py`, client method in `nav_tasks.py`, name constant in `frida_constants/navigation_constants.py` |
| Change how "go to X" behaves | `nav_central.py` `MoveLocation` handler → Nav2 goal send |
| Tune driving (speed, smoothness, obstacle avoidance) | `config/omni_config/nav2_omni.yaml` (MPPI critics/limits) |
| Change SLAM / localization | `omni_setup/{slam,localization}.launch.py` + `mapper_params_*.yaml` |
| Add/rename a place | `frida_constants/map_areas/areas.json` (+ re-annotate via `nav_ui`) |
| Table approach for manipulation | `table_docker.py` + `dock_table()` in `nav_tasks.py` |
| New competition task that drives around | new `*_task_manager.py` FSM in `task_manager/scripts/`, call `self.subtask_manager.nav.*` |
| Recovery / blocked-goal behavior | BTs in `nav_main/bt/` + `adaptive_goal_publisher.py` |
| Run it | `./run.sh navigation` (+ `./run.sh integration --<task>`) |

## 9. Conventions & gotchas

- **Don't talk to Nav2 directly from outside navigation** — use the 5 services in §2.
- **Places are names, not coordinates** — everything routes through `areas.json`.
- **`mock_data=True`** lets you exercise a whole task without hardware; keep new
  subtask-manager methods `@mockable`/`@service_check`-wrapped so this stays true.
- **Python nodes**: inherit `rclpy.node.Node`, set up pubs/subs/srvs in `__init__`, keep
  callbacks thin, use `self.get_logger()` / `CLog` (never `print`). Format with **ruff**;
  run `pre-commit install`. (See `docs/ai/coding_standards.md`, `docs/ROS2.md`.)
- **Submodules**: `sllidar_ros2`, `ira_laser_tools`, `PlayStation-JoyInterface-ROS2`
  are git submodules — `git submodule update --init --recursive` after cloning.
- **Build order**: `frida_interfaces` must be built/cached before any area that uses it.

---

*See also: `docs/ai/architecture.md`, `docs/ai/tech_stack.md`, `docs/ai/coding_standards.md`,
`docs/ROS2.md`, `docs/cyclonedds-setup.md`, `docs/Run/Areas/nav.md`.*

## Addendum (2026-07-03): point-based navigation services

Beyond the original 5-service contract, `nav_central` also serves (constants in
`navigation_constants.py`, srvs in `frida_interfaces/navigation/srv/`):

- `GO_TO_POSE_SERVICE` (`GoToPose`) — arbitrary map-frame PoseStamped through the full
  goal pipeline (retreat-if-docked, resume slam/nav2, clear costmaps, send goal, pause).
- `GET_ROBOT_POSE_SERVICE` (`GetRobotPose`) — robot pose from TF map->base_link.
- `APPROACH_POINT_SERVICE` (`ApproachPoint`) — **approach a person/object seen by
  vision**: target PointStamped in ANY TF frame (base_link/camera/map) + standoff.
  nav_central transforms to map, walks a ring around the target (robot side first,
  ±10°…±180°, then +0.15/+0.30 m radius) and picks the first cell that is FREE on the
  **global costmap** (TRANSIENT_LOCAL sub on `/global_costmap/costmap`; falls back to
  the SLAM `/map`, grid logic mirrors `person_goal_smoother._snap_to_free`), then runs
  the same pipeline as GoToPose, facing the target. Within standoff → turns in place.
  Task side: `nav_tasks.approach_point(point, standoff=0.65)` (thin client; accepts
  PointStamped / Point / (x, y)). Typical GPSR use:
  `nav.approach_point(person_point_from_vision)`.
  - **Parallel-side finish** (doing_laundry basket pick): request fields
    `align` (`""`/`"face"` = face the target; `"left"`/`"right"` = finish with the
    base PARALLEL to the target, target abeam on that side) and `final_distance`
    (with align left/right, closed-loop strafe until base_link is that many meters
    from the target). The alignment (`nav_central._align_parallel`) is direct
    `cmd_vel` (TwistStamped, like `table_docker`) on map→base_link TF feedback —
    costmap-independent, so it can get close to a target that is itself marked as
    an obstacle. Omnibase only (strafe). Task side:
    `nav.approach_point(pt, standoff=0.65, align="right", final_distance=0.45)`.
  - **Direct near-target mode (2026-07-04)**: with align left/right and the target
    within `APPROACH_DIRECT_ALIGN_RANGE` (2 m), the Nav2 ring-search leg is skipped
    entirely — only SLAM is resumed (live map TF) and `_align_parallel` does the
    whole thing: rotate in place until the target is abeam, then drive straight in
    holonomically (vx+vy+wz, 0.12 m/s cap). This is the doing_laundry basket
    approach from the washing machine: short clear corridor, and a ring goal next
    to an obstacle-marked basket would get costmap-rejected anyway.

- `MOVE_RELATIVE_SERVICE` (`MoveRelative`) — **short relative displacement in the
  CURRENT base frame** (`dx` fwd+, `dy` left+, `dyaw` CCW rad). Direct cmd_vel
  closed loop on **odom** TF (`nav_central._move_relative`) — bypasses Nav2 AND
  the costmaps, nothing checks obstacles along the way, so keep it to short
  deliberate sidesteps. Use case: doing_laundry strafes 1 m left after grabbing
  the basket (held on the right) so it clears the washing machine before normal
  navigation. Task side: `nav.move_relative(dx=0.0, dy=1.0)`.
  - **Lidar depth stop** (`stop_at_front_distance` > 0, forward drives only):
    the front lidar sector (±10°, 0 rad = straight ahead per the DOOR_CHECK
    calibration) is monitored every cycle and the drive ends early (success)
    when it reads at or below the target. Response returns `traveled` meters so
    the caller can back out symmetrically (`nav.move_relative(dx=-traveled)`).
    This is the washing-machine arm insert: `dx` is a vision-derived cap, the
    lidar reading against the machine's front face is the real depth stop that
    keeps the gripper off the back of the drum.

## Addendum (2026-07-03): washing-machine insert-and-pick (doing_laundry)

> **Branch note (2026-07-04, `laundry_robocup_final`)**: this flow lives on
> `Doing_papu2` and was NOT ported here — this branch's WM pick is a plain
> `manipulation.pick_object("clothes")`. `MoveRelative` itself (incl. the lidar
> stop) IS on this branch.

Ported/adapted from branch `move-centroid-newapr` (that branch's own
`RelativeMove.srv` was never registered — superseded by `MoveRelative` above).
Flow in `doing_laundry_task_manager.pick_clothes_in_washing_machine`:
1. **Square up**: `nav.dock_table(offset=WM_ALIGN_OFFSET)` — table_docker
   RANSAC-fits the machine front and aligns perpendicular at a standoff
   (alignment only, no approach).
2. **Center**: `vision.get_moondream_point_3d("round container entrance…")`
   (served by `trash_detection_node` via `MOONDREAM_POINT_3D_TOPIC`: moondream
   bbox → depth-ROI mean, camera frame), snapshot to base_link, strafe
   `move_relative(dy=opening.y)` so the drum sits on the base centerline, re-detect.
3. **Aim**: `manipulation.align_arm_toward_centroid(opening)` —
   `AlignArmToCentroid` action on `align_arm_server` (frida_motion_planning):
   FK + j1/j2/j5 sweep so arm shaft + gripper axis point at the target, from
   pre-pose `washing_machine_arrow_pose`.
4. **Insert**: `move_relative(dx=cap, stop_at_front_distance=WM_TARGET_LIDAR_DISTANCE)`
   — straight drive, lidar depth stop (see above).
5. **Grab**: `rotate_wrist_pitch(+90°)`, close gripper, `-90°`, back out `traveled`.
Calibrate `WM_TARGET_LIDAR_DISTANCE` on the robot (park at perfect depth, read
the /scan front sector). Test: `task_manager/scripts/test/test_move_to_washing_machine.py`.

## Addendum (2026-07-03): wall_aligner — precision washing-machine align/close

> **Branch note (2026-07-04, `laundry_robocup_final`)**: wall_aligner was NOT
> ported to this branch (no `wall_aligner.py`, no AlignToWall/CloseToWall srvs,
> no `nav.align_washing_machine`/`close_washing_machine`). It lives on
> `Doing_papu2`.

`navigation/packages/nav_main/scripts/wall_aligner.py` (launched with the
omnibase set in `general_navigation.launch.py`). **Independent of
table_docker** and philosophically opposite: table_docker fits ONCE and locks
the face in odom; wall_aligner locks nothing — every control cycle it re-fits
the NEAREST straight segment the lidar sees in front (cluster the front-sector
scan by point gaps → PCA line fit per cluster → keep straight `rms≤0.015` and
long `≥0.30 m` clusters → take the closest) and servos on that live fit.
Endings are verified with a stationary multi-scan median fit → cm/sub-degree
repeatability. Scan sub uses sensor QoS (/scan is BEST_EFFORT).

Two services (constants in `navigation_constants.py`, srvs in
`frida_interfaces/navigation/srv/`):
- `WALL_ALIGN_SERVICE` (`AlignToWall`) — rotate in place (optional `center`
  strafe to the segment midpoint) until base +x is along the face normal
  (`yaw_tol` 0.008 rad). Response reports the verified perpendicular distance:
  **calibration = park at the perfect depth, call align, read `distance`**.
- `WALL_CLOSE_SERVICE` (`CloseToWall`) — creep (max 0.06 m/s, bidirectional)
  until the perpendicular distance to the live fitted line equals the request
  (`dist_tol` 0.008 m), re-squaring yaw on the live fit every cycle and
  holding lateral position on odom. Gates: aborts if >~5° off square (align
  first), `max_travel` cap, `min_distance` floor, raw front-beam hard stop.

Task side (`nav_tasks.py`): `nav.align_washing_machine(center=False)` →
`(Status, {"distance", "yaw_error", "segment_length"})`;
`nav.close_washing_machine(distance, max_travel=0.0)` →
`(Status, {"traveled", "final_distance"})` (signed traveled → back out with
`move_relative(dx=-traveled)`). Test:
`task_manager/scripts/test/test_wall_align.py` (no arg = align + print
calibration distance; arg = close target). All tolerances/speeds are live
ROS params on `wall_aligner`. RViz markers: `/wall_aligner/markers`.
New srvs ⇒ rebuild the interface cache (`./run.sh frida_interfaces`) and
restart nav + integration containers.

## Addendum (2026-07-03): live-obstacle toggle for carried loads

`SET_OBSTACLE_AVOIDANCE_SERVICE` = `/navigation/set_obstacle_avoidance`
(`std_srvs/SetBool`, served by `nav_central`). `data=False` sets
`obstacle_layer.enabled=false` + `rgbd_obstacle_layer.enabled=false` on **both**
costmaps via their `set_parameters` services and clears them; `data=True` restores.
The **static layer stays on**, so mapped walls/furniture are still avoided — the
robot only goes blind to LIVE obstacles. Use case: an object carried by the arm
(laundry basket/bag) hangs in the lidar/ZED view and gets marked as an obstacle
glued to the robot, boxing the planner in. Task side:
`nav.set_obstacle_avoidance(False)` before a carry, **always** re-enable after
(see `doing_laundry_task_manager.navigate_holding`, which wraps the carry in
try/finally). Changing `ApproachPoint.srv` fields requires rebuilding the
interface cache: `./run.sh frida_interfaces`, then restart the nav + integration
containers.

## Addendum (2026-07-04): doing_laundry basket approach rework

Flow (`doing_laundry_task_manager`): navigate to `laundry/washing_machine` →
`vision.detect_objects("laundry_basket")` from there (both basket spots are in
view) → project to map (`_project_to_map`, zero-stamp TF) →
`nav.approach_point(pt, standoff=0.65, align="right", final_distance=0.45)`.
The basket is within the 2 m direct range, so nav_central skips Nav2: rotate
until the basket is abeam on the RIGHT, then a single closed-loop holonomic
drive to 0.45 m (see "Direct near-target mode" above). After
`pick_object("laundry_basket")` (grab on the right): `move_relative(dy=1.0)`
sidestep LEFT to clear the machine, then `NAVIGATE_TO_BASKET_SIDE` — the
nearer annotated `basket_left`/`basket_right` as an INTERMEDIATE waypoint
(best-effort, skipped/abandoned gracefully) — then the carry to the table.
The annotations are NOT a fallback: a scan that fails 3× recenters on the
washing machine and rescans; an approach that fails 3× discards the point and
rescans. The task only moves on the detected basket point.

Vision side: `laundry_basket.point3d` comes from a **depth-cluster centroid**,
not the bbox-center pixel (`get_cluster_point` in
`object_detector_2d/scripts/vision_3D_utils.py`, wired in
`base_detector_node.extract_3d`, gated by param `CLUSTER_POINT3D_LABELS`,
default `["laundry_basket"]`). It clusters the bbox depth ROI along depth
(0.15 m gaps) and returns the pixel-centroid + median depth of the NEAREST
cluster with ≥5 % support — the basket's front face — because the center pixel
lands in the opening (clothes inside / floor behind the rim) and shifts the
point tens of cm. `final_distance` is therefore base_link → **front face**,
not basket center; calibrate 0.45 with that in mind.

On-robot checks pending (2026-07-04): the limp-base `vy` strafe quality during
`_align_parallel` phase 2 (capped at 0.12 m/s; loosen `TOL_LIN` if it
oscillates), and the 0.45 m side-pick gap. New srv fields ⇒
`./run.sh frida_interfaces` + restart nav & integration.
