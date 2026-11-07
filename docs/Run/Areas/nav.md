# Navigation Docs

## How to Run

1. Connect to `roborregos_home` on Wi-Fi.

On your computer:
```bash
ssh orin@192.168.31.10
cd home2
./run.sh navigation # Now you should be on the navigation Docker
```

2. Run

---

## Running Nav Basics

The Nav Basics include the following components:
- Dashgo driver
- RP Lidar fixed
- EKF filter for odom transform
- (Optional) URDF state publisher to link `base_link` and lidar

### Arguments
- `publish_tf` (default: `true`): Active URDF publish state.
  - **WARNING:** Set to `false` if MoveIt config is active.

### Run Command
```bash
ros2 launch nav_main nav_basics.launch.py
```

---

## Running AMCL Localization

The AMCL launch includes the following components:
- Nav Basics launch (**Do not** launch `nav_basics` in another terminal if AMCL launch is running)
- Nav2 Map Server
- Nav2 AMCL
- Nav2 Lifecycle Manager

### Arguments
- `publish_tf` (default: `true`): Active URDF publish state.
  - **WARNING:** Set to `false` if MoveIt config is active.
- `map` (default: Path for lab map): Provide an **absolute path** to import the map.

### Run Command
```bash
ros2 launch nav_main nav_amcl.launch.py
```

---

## Running Navigation Node

> **IMPORTANT:** You must run a SLAM before. Examples include `nav_amcl` or `rtabmap` (in development).

The Navigation Node includes the following components:
- Nav2 Server

### Arguments
- `Custom yaml config`

### Run Command
```bash
ros2 launch nav_main navigation_launch.py
```
---

## Mapping Evaluation: ZED Obstacles in the Static Map (#1318)

The lidars sit at z = 0.136 m, so table tops, chair seats and shelves never reach the slam_toolbox map. To evaluate adding them from the ZED depth, run offline on a bag. slam_toolbox keeps the poses and localization; RTAB-Map only builds a grid from the ZED (`zed_grid.launch.py`, `config/rtabmap/rtabmap_grid_only.yaml`).

### 1. Record (robot, inside the navigation container)
- Save under `/tmp` (mounted to the host); the container home is not persisted, so a bag there is lost on exit. Copy it off the Orin afterwards.
- Run **only** `omni_basics.launch.py` + teleop, **no SLAM**. Otherwise the bag carries a live `map->odom` that fights the replay.
- Put the arm in the navigation pose (the ZED is on the gripper). Clear people out of the arena if possible.
- Lab tables are 0.70-0.80 m tall. With the arm in `NAV_POSE`, check that the ZED sees the table tops: `ros2 run tf2_ros tf2_echo base_link zed_camera_link`, then watch `/point_cloud` in RViz as you drive up to a table. If the camera sits at table height looking straight ahead, it sees the top edge-on and the grid barely marks it.
- Drive ~5 min past every table/chair. Write down a furniture checklist (item, rough position, height): that's the metric.

```bash
ros2 bag record -o /tmp/bags/mapping_1318 --storage-preset-profile zstd_fast \
  /scan /scan_front /scan_rear /odrive/odom /odrive/imu /tf /tf_static /point_cloud \
  /zed/zed_node/rgb/image_rect_color /zed/zed_node/rgb/camera_info /zed/zed_node/depth/depth_registered
ros2 bag info /tmp/bags/mapping_1318   # depth + point_cloud counts must be non-zero (SHM drops big msgs)
```

### 2. Replay (baseline + ZED grid in the same run)
```bash
ros2 launch nav_main slam.launch.py use_sim_time:=true
ros2 launch nav_main zed_grid.launch.py use_sim_time:=true
ros2 bag play /tmp/bags/mapping_1318 --clock
# when it ends, save both grids (absolute paths):
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/abs/path/baseline'}}"
ros2 run nav2_map_server map_saver_cli -t /rtabmap/map -f /abs/path/zed_grid
```

Full RTAB-Map (lidar + depth, own poses) for comparison: skip `slam.launch.py` and pass `params_file:=<nav_main share>/config/rtabmap/rtabmap_mapping_config.yaml`. SuperPoint needs the Orin image.

### 3. Merge and Compare
```bash
ros2 run map_context merge_grids.py /abs/path/baseline.yaml /abs/path/zed_grid.yaml /abs/path/merged
ros2 run map_context merge_grids.py --test   # self-check
```
Score `baseline` vs `merged` against the checklist: furniture present, and false obstacles (people/noise baked in). If the floor shows up as obstacles, raise `Grid/MaxGroundHeight`. The merged `.pgm` is served by map_server in `localization.launch.py`; the `.posegraph` is unchanged.
