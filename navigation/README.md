# Navigation

Navigation handles mapping (SLAM), localization, path planning and following,
semantic areas, table docking and person following for FRIDA. It runs on `ROS 2`
(Humble) and Nav2 inside a single centralized container.

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
│   └── srv/                            # ApproachPoint, DockTable, GetRobotPose,
│                                       # GoToPose, MapAreas, MoveLocation, NavQuery
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
│   │   │   ├── dashgo_base/            # Diff-drive base bringup (RTABMap)
│   │   │   ├── omni_setup/             # Omni base bringup (slam_toolbox)
│   │   │   └── task_launch/            # Top-level launches per task
│   │   │       ├── general_navigation.launch.py
│   │   │       ├── mapping.launch.py
│   │   │       ├── restaurant.launch.py
│   │   │       ├── hric.launch.py
│   │   │       └── gpsr_hric.launch.py
│   │   └── scripts/                    # ROS nodes (Python)
│   │       ├── nav_central.py          # Central navigation orchestrator
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
│   ├── dashgo_driver/                  # Diff-drive base driver (STM32) + EKF
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

The base and SLAM backend are chosen with launch arguments. The omnibase row is
the current setup; the dashgo row is the legacy diff-drive path:

| `default_base` | `nav_type` | SLAM backend | Base | Status |
| --- | --- | --- | --- | --- |
| `omnibase` (default) | `2d` | `slam_toolbox` (lidar) | Holonomic (`omnidriver`) | Current |
| `dashgo` | `3d` | `RTABMap` (RGBD) | Diff-drive (`dashgo_driver`) | Legacy |

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

Rebuild the ROS packages / image when needed:

```bash
./run.sh navigation --build          # colcon build inside the container
./run.sh navigation --build-image    # rebuild the Docker image
./run.sh navigation --clean          # remove build/install/log artifacts
./run.sh navigation --recreate       # recreate the container
./run.sh navigation --down           # stop and remove the container
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
# Use the diff-drive base + 3D RTABMap SLAM instead of the omnibase default
ros2 launch nav_main general_navigation.launch.py default_base:=dashgo nav_type:=3d

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
   - omnibase / slam_toolbox → `<name>.posegraph` + `<name>.data` + a
     `<name>.yaml`/`<name>.pgm` grid.
   - dashgo / RTABMap → the RTABMap `<name>.db` + a 2D `<name>.yaml`/`.pgm`.
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
| `/navigation/go_to_map_area` | `MoveLocation` | Navigate to a named area/sublocation from `areas.json` |
| `/navigation/go_to_pose` | `GoToPose` | Navigate to an arbitrary map-frame pose |
| `/navigation/approach_point` | `ApproachPoint` | Approach a person/object seen by vision at a standoff |
| `/navigation/get_robot_pose` | `GetRobotPose` | Current robot pose from TF (map → base_link) |
| `/navigation/query_path` | `NavQuery` | Path distance between two areas without moving |
| `/navigation/dock_table` | `DockTable` | Dock to the surface in front (with offset) |
| `/navigation/is_door_open` | `CheckDoor` | Detect whether the door ahead is open via lidar |
| `/navigation/areas_json` | `MapAreas` | Return the tagged `areas.json` as a string |
| `/navigation/follow_person` | `SetBool` | Start/stop person-following navigation |
| `/navigation/resume_nav` | `Empty` | Manually resume paused SLAM + Nav2 |

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
| `dashgo_driver` | EAI Dashgo diff-drive | Legacy | STM32 serial driver + `ekf.launch.py` odometry fusion; kept for the old diff-drive base |


