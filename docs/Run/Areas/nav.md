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