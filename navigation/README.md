# Navigation

Navigation handles mapping (SLAM), localization, path planning and following,
semantic areas, table docking and person following for FRIDA. It runs on `ROS 2`
(Jazzy) and Nav2 inside a single centralized container.

> FRIDA currently uses an **omnidirectional (holonomic) base**, which is why it is
> the default across this area. The previous **differential-drive base** (EAI
> Dashgo) is kept for backward compatibility and can still be selected, but the
> omnibase is the supported setup.

## Tree structure

```bash
home2/
│
│frida_constants/                       # Shared constants for the project
├── frida_constants/
│   └── navigation_constants.py         # Topic/service/action names, timeouts, limits
│
│frida_interfaces/                      # Custom ROS interfaces
├── navigation/
│   ├── action/
│   │   └── Move.action
│   ├── msg/
│   │   ├── MonitorReport.msg
│   │   └── NodeStatus.msg
│   └── srv/                            # ApproachPoint, DockTable, GetAreaForPoint,
│                                       # GetRobotPose, GoToPose, MapAreas, MoveLocation,
│                                       # NavQuery, PlanPatrol
│
│navigation/
├── packages/                           # ROS 2 packages for the area
│   ├── nav_main/                       # Core navigation: SLAM + Nav2 orchestration
│   │   ├── bt/                         # Behavior Trees (navigate, follow point)
│   │   ├── config/                     # Nav2 + RTABMap + slam_toolbox params
│   │   │   ├── nav2_standard.yaml
│   │   │   ├── nav2_following.yaml
│   │   │   ├── nav2_restaurant.yaml
│   │   │   ├── omni_config/            # Holonomic (omnibase) Nav2 profiles
│   │   │   └── rtabmap/               # RTABMap RGBD SLAM configs
│   │   ├── launch/
│   │   │   ├── omni_setup/             # Omni base bringup (slam_toolbox)
│   │   │   └── task_launch/            # Top-level launches per task
│   │   │       ├── general_navigation.launch.py
│   │   │       ├── mapping.launch.py
│   │   │       ├── restaurant.launch.py
│   │   │       ├── hric.launch.py
│   │   │       └── gpsr_hric.launch.py
│   │   ├── nav_main/                   # Importable Python package
│   │   │   └── semantic/               # Semantic nav core (no ROS, unit-testable)
│   │   │       ├── areas.py            # Point -> room / furniture
│   │   │       ├── surfaces.py         # Furniture typing, arm pose and dwell
│   │   │       ├── route.py            # Patrol ordering (NN + 2-opt per room)
│   │   │       ├── viewpoints.py       # Costmap check and relocation
│   │   │       └── staleness.py        # When each surface was last scanned
│   │   └── scripts/                    # ROS nodes (Python)
│   │       ├── nav_central.py          # Central navigation orchestrator
│   │       ├── semantic_nav_node.py    # Patrol routes + scan bookkeeping
│   │       ├── semantic_nav_selftest.py # Offline self-test of the core
│   │       ├── table_docker.py         # Perpendicular table/shelf docking
│   │       ├── person_goal_smoother.py # Person-following goal bridge
│   │       ├── adaptive_goal_publisher.py
│   │       ├── node_monitor.py
│   │       └── launch_nav.py
│   │
│   ├── map_context/                    # Maps, areas and the UIs
│   │   ├── maps/                       # Saved maps (.pgm/.yaml, RTABMap .db, posegraph)
│   │   │   └── areas/                  # areas_<map>.json (tagged semantic areas)
│   │   ├── scripts/
│   │   │   ├── nav_ui.py               # Live nav monitor + control panel (Qt)
│   │   │   ├── map_area_tagger.py      # Area/location tagging UI (Qt)
│   │   │   └── simulate_position.py
│   │   ├── src/map_service.cpp         # C++ map/areas service
│   │   └── launch/simulate_map.launch.py
│   │
│   └── omnidriver/                     # Holonomic (ODrive) base driver + dashboard
│
├── README.md                           # This file
└── ...
│
docker/navigation/                      # Docker image, compose and entrypoint
├── Dockerfile.{cpu,cuda,l4t}
├── docker-compose.yaml
└── run.sh                              # Area entrypoint (called by root run.sh)
```

## Concepts

The SLAM backend is chosen with the `nav_type` launch argument. The robot always
runs on the omnibase:

| `nav_type` | SLAM backend | Base |
| --- | --- | --- |
| `2d` (default) | `slam_toolbox` (lidar) | Holonomic (`omnidriver`) |
The **`nav_central`** node is the brain of the area. It:

- Waits for required topics/TF, starts the SLAM backend and (optionally) Nav2.
- **Monitors** the system continuously — if topics/TF drop it pauses SLAM/Nav2
  and resumes them automatically when they come back, reloading RTABMap if it
  crashes.
- Exposes the navigation services other areas call (see the table below).
- Pauses SLAM/Nav2 while idle to save CPU and resumes them for each goal.

## Setup with Docker

### Requirements

- Docker Engine + Docker Compose
- NVIDIA Container Toolkit (for CUDA / L4T images)
- The robot's USB devices connected (lidar, STM32) for real hardware

### Building and entering the container

From the repo root (`home2`), the general `./run.sh` script forwards to
`docker/navigation/run.sh`. It auto-detects the environment (cpu / cuda / l4t),
sets up USB devices and CycloneDDS, builds `nav_main` and opens a shell:

```bash
# From home2/
./run.sh navigation
```

Inside the container the workspace lives at `/workspace` and the repo is mounted
at `/workspace/src`. To build manually:

```bash
colcon build --symlink-install --packages-up-to nav_main \
  --packages-ignore frida_interfaces frida_constants --cmake-args -Wno-dev
source install/setup.bash
```

### Selecting the map

The active map is read from the `MAP_NAME` variable (e.g. `lab_23.db`). Instead
of exporting it by hand, set it once with the root `run.sh`:

```bash
./run.sh --update-map lab_23.db   # persists export MAP_NAME in your .bashrc/.zshrc
source ~/.bashrc                  # (or ~/.zshrc) so the current shell sees it
./run.sh navigation --gpsr
```

From `MAP_NAME`, the launch files derive the `areas_<map>.json`, the slam_toolbox
posegraph and any keepout mask. To override it for a single session without
touching your rc file, pass `map_name:=<file>` to the launch (see below).

## Running navigation

Once inside the container (or via the task flags of `run.sh`), the top-level
launches are:

```bash
# Build a new map (SLAM only, no localization)
ros2 launch nav_main mapping.launch.py

# Localization + Nav2 — used by every competition task (gpsr/ppc/dlc/hric/safety)
ros2 launch nav_main general_navigation.launch.py

# Restaurant — maps the unknown venue live while serving, with table docking
ros2 launch nav_main restaurant.launch.py
```

Common launch arguments:

```bash
# Override the map for this session only
ros2 launch nav_main general_navigation.launch.py map_name:=lab_23.db
```

The equivalent shortcuts from the root `run.sh` are:

```bash
./run.sh navigation --mapping     # ros2 launch nav_main mapping.launch.py
./run.sh navigation --gpsr        # general_navigation.launch.py
./run.sh navigation --restaurant  # restaurant.launch.py
./run.sh navigation --tagger      # ros2 run map_context map_area_tagger.py
./run.sh navigation --move        # omni_basics.launch.py (base + teleop only)
```

### Building a map (mapping workflow)

1. Launch mapping: `ros2 launch nav_main mapping.launch.py` (or
   `./run.sh navigation --mapping`). This starts `nav_central` in mapping mode,
   the SLAM backend and the **`nav_ui`** control panel.
2. Drive the robot around the arena to cover the whole space.
3. In `nav_ui`, use **Save Map** to persist it:
      - `<name>.posegraph` + `<name>.data` + a `<name>.yaml`/`<name>.pgm` grid.
   Maps are written to `navigation/packages/map_context/maps/`.

### Tagging areas (the tagger)

Semantic areas ("kitchen", "bedroom", locations like "sink", "shelf" …) are what
`nav_central` navigates to by name. Define them with the **map area tagger**:

```bash
ros2 run map_context map_area_tagger.py   # or: ./run.sh navigation --tagger
```

The tagger loads a map image (`.pgm`/`.png`), lets you click to place named
locations and draw polygon area boundaries (pre-seeded with the RoboCup @Home
arena rooms/objects), and exports them to
`map_context/maps/areas/areas_<map>.json`. Each location stores a full pose
`[x, y, z, qx, qy, qz, qw]`. You can also hand-edit the `.pgm` to paint virtual
obstacles or draw a keepout mask that Nav2 will respect.

### Docking to a table / shelf

`table_docker.py` (started automatically by `general_navigation` /`restaurant`
on the omnibase) performs a **perpendicular approach** to a table or shelf: Nav2
brings the robot to a static "near" pose, then the docker detects the surface's
front face from lidar/point-cloud (line for flat tables, circle for round ones),
locks it, and closed-loop drives the holonomic base until the arm is
`target_distance` from the surface.

You can activate docking in three ways:

- **From the `nav_ui` panel** — press the **Dock** button (optionally set a
  front offset). It calls `nav_central`'s `DockTable` service so a per-call
  offset can be applied.
- **From a service call** (see below).
- **Automatically** — `nav_central` calls the undock service before every new
  location goal so the robot backs off a docked surface before planning.

```bash
# Preview the detected face/orientation without moving
ros2 service call /navigation/preview_dock std_srvs/srv/Trigger {}

# Dock (offset 0.0 uses the docker default)
ros2 service call /navigation/dock_table frida_interfaces/srv/DockTable "{offset: 0.0}"

# Undock / back off so Nav2 can plan the next goal
ros2 service call /navigation/undock_from_surface std_srvs/srv/Trigger {}
```

## Navigation services

`nav_central` exposes the interface the rest of FRIDA uses. Names come from
[`frida_constants/navigation_constants.py`](../frida_constants/frida_constants/navigation_constants.py).

| Service | Type | Purpose |
| --- | --- | --- |
| `/navigation/go_to_map_area` | `MoveLocation` | Go to a named area/sublocation from `areas.json` |
| `/navigation/go_to_pose` | `GoToPose` | Go to a map-frame pose |
| `/navigation/approach_point` | `ApproachPoint` | Approach a point at a standoff distance |
| `/navigation/get_robot_pose` | `GetRobotPose` | Current pose from TF (map → base_link) |
| `/navigation/query_path` | `NavQuery` | Path distance between two areas (no motion) |
| `/navigation/dock_table` | `DockTable` | Dock to the surface in front (with offset) |
| `/navigation/is_door_open` | `CheckDoor` | Door open/closed via lidar |
| `/navigation/areas_json` | `MapAreas` | Return `areas.json` as a string |
| `/navigation/follow_person` | `SetBool` | Start/stop person following |
| `/navigation/resume_nav` | `Empty` | Resume paused Nav2 |
| `/navigation/get_area_for_point` | `GetAreaForPoint` | Which room and furniture a map point belongs to |
| `/navigation/plan_patrol` | `PlanPatrol` | Ordered route over the tagged furniture poses (no motion) |

### Example service calls

```bash
# Go to a named area
ros2 service call /navigation/go_to_map_area frida_interfaces/srv/MoveLocation \
  "{location: 'kitchen', sublocation: 'sink'}"

# Ask how far the kitchen sink is from the current pose (no motion)
ros2 service call /navigation/query_path frida_interfaces/srv/NavQuery \
  "{location_a: '', sublocation_a: '', location_b: 'kitchen', sublocation_b: 'sink'}"

# Start / stop following a person
ros2 service call /navigation/follow_person std_srvs/srv/SetBool "{data: true}"
ros2 service call /navigation/follow_person std_srvs/srv/SetBool "{data: false}"
```

```bash
# Which room / furniture is this point in? (the point may be in any TF frame)
ros2 service call /navigation/get_area_for_point frida_interfaces/srv/GetAreaForPoint \
  "{point: {header: {frame_id: map}, point: {x: -0.4, y: -12.8, z: 0.75}}}"

# Patrol route over every tagged piece of furniture (plans only, never drives)
ros2 service call /navigation/plan_patrol frida_interfaces/srv/PlanPatrol "{mode: full}"
```

## Semantic navigation (patrol routes and area lookup)

`semantic_nav_node.py` turns the furniture already tagged in `areas_<MAP_NAME>.json` into a patrol
route, and keeps track of when each surface was last looked at. It is the navigation half of the
semantic map (issue #1268): vision owns the objects, navigation owns *where to stand to see them*.

**Why it exists.** The object detector only reaches **2.0 m**, so patrolling is not visiting rooms —
it is parking in front of each piece of furniture. The exploration used until now drives to each
area's `safe_place`, i.e. the middle of the room, where nothing on a tabletop is in range.

**The poses are not invented.** A sublocation entry in `areas_<MAP_NAME>.json` is already a *robot*
pose: the tagger stores the operator's click as the base position and the drag as the heading, and
`nav_central.go_to_area` sends it straight to Nav2. The node selects those poses, checks each one
against the global costmap (relocating it around its furniture when something now blocks it), orders
them nearest-first with 2-opt grouped by room, and annotates each with an arm "stare" pose and a dwell
time.

**Nav plans, the caller drives.** `PlanPatrol` returns a list; task_manager iterates it, navigates to
each pose, asks manipulation for `arm_poses[i]` and waits `dwell_s[i]` so vision can confirm what is
there. The node has no `cmd_vel` publisher and no action client — it cannot move the robot.

Modes: `full` (every surface), `quick` (one per room), `revisit` (stalest first — what the robot has
not looked at in a while, for Finals).

### Furniture metadata (optional)

Each sublocation may carry a `<name>_meta` sibling that overrides what is otherwise inferred from the
name. Edit it in the tagger: right-click a location → *Edit Furniture Metadata…*

```json
"kitchen": {
  "dinner_table": [x, y, z, qx, qy, qz, qw],
  "dinner_table_meta": {"type": "surface", "height": 0.75, "extent": [0.9, 1.4],
                        "arm_pose": "table_stare", "dwell_s": 3.0, "center": [-0.37, -13.63]}
}
```

Without it the type comes from the name (`table` → surface, `shelf`/`cabinet` → shelf,
`refrigerator`/`sink` → appliance, `trash`/`basket` → floor, `bed`/`sofa` → low surface), which is
correct for every furniture name currently tagged in `map_context/maps/areas/`.

### Topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/navigation/patrol/markers` | `MarkerArray` | Viewpoints in RViz, coloured by how recently each was scanned |
| `/navigation/patrol/staleness` | `String` (JSON) | Age of the last scan per surface |

### Self-test

Runs the whole core offline — no ROS, no robot, no simulation — against the real maps:

```bash
python3 navigation/packages/nav_main/scripts/semantic_nav_selftest.py
python3 navigation/packages/nav_main/scripts/semantic_nav_selftest.py --map robocup_c1
ros2 run nav_main semantic_nav_selftest.py        # from the installed tree
```

`WARN` lines are map-data problems the code absorbs but that should be fixed with the tagger: a
furniture pose that now sits inside an obstacle, or two pieces tagged at the same spot.

## Nav_ui — live monitor & control panel

`nav_ui.py` (launched automatically with mapping / general navigation) is the Qt
panel used on the field. It renders the map, robot pose, costmaps and planned
trajectories, and lets the operator:

- Set the **initial pose** (required before localized navigation starts —
  `nav_central` waits for it).
- Send goals, change the active map, save maps, resume paused navigation.
- Trigger docking with a configurable offset.

## Robot bases

| Package | Base | Status | Notes |
| --- | --- | --- | --- |
| `omnidriver` | Holonomic ODrive base | Current | `odrive_serial_twist` (cmd_vel → wheels), `odrive_dashboard` web dashboard, `simple_rx` |

## External repositories

- **Dashgo base driver & launch files**: Moved to [RoBorregos/Dashgo](https://github.com/RoBorregos/Dashgo.git)