# Receptionist

## Vision
Run container. From the root directory (home2), run:
```bash
./run.sh vision --receptionist
```

## HRI
Run containers. From the root directory (home2), run:
```bash
./run.sh hri --receptionist
```

## Navigation

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

## Manipulation
Run container. From the root directory (home2), run:
```bash
./run.sh manipulation
```

Note: ollama models are downloaded and mounted on the repo. Run the repo at the path `/home/orin/home2` to avoid having to re-download the models.

### Arm bringup
To test on the real robot, run the launch to bring up the robot and MoveIt planning:
```bash
ros2 launch arm_pkg frida_moveit_config.launch.py robot_ip:=192.168.31.180
```

### Motion Planning
Check that the motion planning interface is running:
```bash
ros2 run frida_motion_planning motion_planning_server.py
```

### Follow face
To follow a face, run the following command:
```bash
ros2 run task_manager follow_face_node.py
```