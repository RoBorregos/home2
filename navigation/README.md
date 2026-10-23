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

### Docking to a table / shelf (planned approach)

`table_docker.py` (started automatically by `general_navigation` /`restaurant`
on the omnibase) docks at a table, counter or shelf **without hardcoded
distances**. Callers name the **surface type**, and optionally the object to work
on. The docker then works through four steps:

1. **Detect.** It finds the front face from lidar/point cloud (a line for flat
   surfaces, a circle for round tables) and locks it in `odom`.
2. **Base placement.** It samples candidate base poses along the face, or
   around the round table, inside that type's **reach band** `[gap_min, gap_max]`.
   - The full footprint of each candidate is checked against the local costmap,
     so a chair next to the table rejects the poses it overlaps.
   - The remaining candidates are scored on gap, lateral offset from the object,
     and travel.
3. **Motion.** MPPI (Nav2 `controller_server`) drives to a pre-dock pose in
   front of the best candidate. The path is chosen in this order:
   1. the straight line, when the swept footprint is free;
   2. otherwise a footprint-aware A* on the local costmap plus the remembered
      obstacles;
   3. as a last resort, the Nav2 planner.

   On arrival the docker re-detects and re-plans.
4. **Straight-in.** A holonomic controller closes the last centimetres against
   the locked face. Once docked, it re-detects up close and corrects. The live
   lidar keeps every point outside the footprint.

Three safeguards keep the approach from trusting a single snapshot:

- **Obstacle memory.** During a dock, every lethal costmap cell seen since the
  request started is kept. The memory is cleared when the dock ends. nav2's
  costmap loses thin obstacles such as chair legs once rays pass beside them or
  they fall into the lidar's near blind zone. The plan, the swept-path checks and
  a guard during MPPI all use the remembered cells.
- **Same-surface check.** A re-detection more than 10 cm *closer* than the
  locked face is ignored, so the docker does not switch to a chair in front of
  the table. A face up to 35 cm *farther* is accepted only if the lidar shows the
  strip straight ahead is free up to it, for example the back of a cabinet niche
  once the side panels are out of view. Close-range re-detections of flat faces
  look only at the ±25° strip the arm will work on.
- **Face fusion.** The orientation comes from the longest stretch of face seen.
  The distance comes from the closest view, plus the nearest lidar point straight
  ahead.

The planning code is `nav_main/approach_planner.py` (pure Python, unit tested in
`nav_main/test/test_approach_planner.py`).

Distances live **per surface type** in
[`config/approach_profiles.yaml`](packages/nav_main/config/approach_profiles.yaml):

| Type | Preferred gap (robot front → surface) | Band |
| --- | --- | --- |
| `table`, `counter`, `round_table` | 0.03 m | 0.02 to 0.15 m |
| `cabinet`, `shelf` | 0.17 m | 0.10 to 0.25 m |
| `dishwasher` | 0.19 m | 0.12 to 0.25 m |
| `serving_table` | 0.19 m | 0.12 to 0.25 m |

The same file holds three more settings:

- the band for `approach_point`, which is the distance from the base centre to
  the person or object,
- the scoring weights, separate for docking and for `approach_point`,
- `footprint_padding`, the margin that candidate poses keep from every obstacle
  other than the surface being docked at.

You can activate docking in three ways:

- **From the `nav_ui` panel**: pick the surface type and press
  **Approach Table**.
- **From a service call** (see below), or `nav_tasks.dock_table(surface_type=...,
  target=PointStamped)` in a task manager.
- **Automatically for undocking**: `nav_central` calls the undock service before
  every new location goal, so the robot backs off a docked surface before
  planning.

```bash
# Preview: detect + plan, show candidates in RViz (/approach_planner/candidates), no motion
ros2 service call /navigation/preview_dock std_srvs/srv/Trigger {}

# Dock at a counter, standing in front of an object seen by vision
ros2 service call /navigation/dock_table frida_interfaces/srv/DockTable \
  "{surface_type: counter, target: {header: {frame_id: base_link}, point: {x: 0.9, y: 0.3}}}"

# Undock / back off so Nav2 can plan the next goal
ros2 service call /navigation/undock_from_surface std_srvs/srv/Trigger {}
```

The old `offset` field still works when `surface_type` is empty: it is converted
to the gap it used to produce.

## Navigation services

`nav_central` exposes the interface the rest of FRIDA uses. Names come from
[`frida_constants/navigation_constants.py`](../frida_constants/frida_constants/navigation_constants.py).

| Service | Type | Purpose |
| --- | --- | --- |
| `/navigation/go_to_map_area` | `MoveLocation` | Go to a named area/sublocation from `areas.json` |
| `/navigation/go_to_pose` | `GoToPose` | Go to a map-frame pose |
| `/navigation/approach_point` | `ApproachPoint` | Approach a point: planned pose in the standoff band, footprint-checked, with fallbacks |
| `/navigation/get_robot_pose` | `GetRobotPose` | Current pose from TF (map → base_link) |
| `/navigation/query_path` | `NavQuery` | Path distance between two areas (no motion) |
| `/navigation/dock_table` | `DockTable` | Planned dock at the surface in front (surface type + optional target) |
| `/navigation/is_door_open` | `CheckDoor` | Door open/closed via lidar |
| `/navigation/areas_json` | `MapAreas` | Return `areas.json` as a string |
| `/navigation/follow_person` | `SetBool` | Start/stop person following |
| `/navigation/resume_nav` | `Empty` | Resume paused Nav2 |

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

## Testing

[`task_manager/scripts/test/test_navigation_manager.py`](../task_manager/scripts/test/test_navigation_manager.py)
first checks the basics — ODrive axis errors and states, bus voltage, the merged
`/scan`, the `map -> base_link` TF and the `nav_central` services — and then runs
the `NavigationTasks` functions, which do move the robot.

```bash
ros2 run task_manager test_navigation_manager.py
```

Failures print the reason, and the values read (voltages, axis states, scan
quality, how far `Explore Zone` moved and how close `Return to Origin` stopped)
are listed at the end. `Explore Zone` fails if the robot reports the goal reached
without advancing at least 0.5 m of its 1 m step.

`wheels:=true` spins the base in place while reading `/odrive/vel_est`, so it
reports any wheel that does not turn or that turns the wrong way. It needs a
clear floor and only `omni_basics.launch.py`:

```bash
ros2 run task_manager test_navigation_manager.py --ros-args -p wheels:=true
```

In the Gazebo sim (`./run.sh simulation --nav`, see
[`simulation/frida_gz_sim`](../simulation/frida_gz_sim/README.md)) there is no
ODrive, so `sim:=true` skips the motor, voltage and wheel checks:

```bash
./run.sh simulation   # shell in the sim container
ros2 run task_manager test_navigation_manager.py --ros-args -p sim:=true
```

| Parameter | Default | |
| --- | --- | --- |
| `basics` | `true` | Run the base/lidar/TF/service checks |
| `sim` | `false` | Gazebo sim: skip the ODrive motor, voltage and wheel checks |
| `wheels` | `false` | Spin the base to test each wheel |
| `dock` | `false` | Include `dock_table` |
| `mocked` | `false` | Use the subtask manager mocks, skips the basics |
| `clear_logs` | `true` | Hide the ROS logs while each test runs |
| `cmd_vel_topic` / `stamped_cmd_vel` | `/cmd_vel` / `true` | Where the base listens and whether it expects `TwistStamped` |

## Nav_ui — live monitor & control panel

`nav_ui.py` (launched automatically with mapping / general navigation) is the Qt
panel used on the field. It renders the map, robot pose, costmaps and planned
trajectories, and lets the operator:

- Set the **initial pose** (required before localized navigation starts —
  `nav_central` waits for it).
- Send goals, change the active map, save maps, resume paused navigation.
- Trigger docking for a chosen surface type.

## Robot bases

| Package | Base | Status | Notes |
| --- | --- | --- | --- |
| `omnidriver` | Holonomic ODrive base | Current | `odrive_serial_twist` (cmd_vel → wheels), `odrive_dashboard` web dashboard, `simple_rx` |

## External repositories

- **Dashgo base driver & launch files**: Moved to [RoBorregos/Dashgo](https://github.com/RoBorregos/Dashgo.git)