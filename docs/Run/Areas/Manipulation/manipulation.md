# Manipulation

The Manipulation area handles all physical interaction with objects: motion planning for the UFACTORY xArm6 robot arm, 3D scene perception, grasp detection, and pick/place/pour task execution. It exposes high-level action servers that the task manager calls with an object name and receives a success/failure result.

## Architecture

```
Vision Area (/vision/detections, depth camera)
        │
[perception_3d]
 ├─ plane_service        (RANSAC support plane)
 ├─ read_cluster         (object clustering from pointcloud)
 ├─ pick_primitives      (collision cylinders for MoveIt scene)
 └─ flat_grasp_estimator (PCA grasp pose for cutlery)
        │
[pick_and_place pipeline]
  ├─ manipulation_core   ← top-level entry point (ManipulationAction)
  │    ├─ PickManager  → pick_server   (PickMotion)
  │    ├─ PlaceManager → place_server  (PlaceMotion)
  │    └─ PourManager  → pour_server   (PourMotion)
  │
  ├─ heatmapPlace_Server  (surface placement selection)
  └─ gpd_service          (GPD grasp detection from pointcloud)
        │
[frida_motion_planning]
  └─ motion_planning_server  ← all arm movement goes here
       ├─ MoveToPose / MoveJoints action servers
       ├─ Collision scene management
       └─ MoveItPlanner → pymoveit2 → ROS2 control → xArm6
```

All arm movement must go through `motion_planning_server`. Do not call MoveIt MoveGroup directly.

## Packages

### frida_motion_planning

The motion planning service layer. `motion_planning_server.py` is the single node that all other packages use to move the arm. It wraps pymoveit2, manages the MoveIt collision scene, and exposes gripper control. Default planner: `RRTConnect`; velocity: 0.15 m/s; planning time: 0.5 s.

It exposes two action servers (`MoveToPose` and `MoveJoints`) and services for collision object management, gripper open/close, servo mode toggle, and pre-recorded trajectory playback.

Utility classes in `frida_motion_planning/utils/`: `MoveItPlanner` (plan + execute), `MoveItServo` (continuous Cartesian servo), `XArmServices` (xArm mode switching and gripper).

### pick_and_place

Task orchestration. `manipulation_core` is the top-level entry point — it receives a `ManipulationAction` goal and coordinates perception, grasp selection, and motion execution through dedicated servers:

- **pick_server**: Executes pick motions. For normal objects, selects the best grasp from GPD poses. For cutlery (spoon/fork/knife), uses **force-guarded descent**: switches xArm to Cartesian velocity mode (mode 5), descends at 20 mm/s, and stops when joint effort exceeds 6.5 N — necessary because flat objects have uncertain Z positions.

- **place_server**: Executes place motions. Handles table-top and shelf placement differently based on `PlaceParams.is_shelf`. Opens gripper and detaches the collision object after placement.

- **pour_server**: Executes pouring motions using joint-space control to tilt the object.

### perception_3d

3D scene understanding. The C++ nodes (`plane_service`, `read_cluster`, `pick_primitives`) run as a pipeline: detect the support plane via RANSAC, cluster objects above it, and add cylinder collision primitives to the MoveIt scene.

`flat_grasp_estimator.py` is specific to cutlery: it extracts the depth ROI of detected spoon/fork/knife bounding boxes, runs PCA to find the long-axis orientation, and stabilizes the table height with a 15-sample rolling buffer (15 mm outlier rejection). `PickManager` waits for a stable pose on `/manipulation/flat_grasp_pose` before sending the `PickMotion` goal.

### place

`heatmapPlace_Server.py` determines where on a surface to place a held object. It converts the surface pointcloud to a 2D occupancy grid (15 mm resolution), computes a heat map (attraction to occupied surface, 300 mm kernel) minus a cool map (repulsion from empty space, 150 mm kernel), and returns the highest-scoring point. Supports closeness preference and directional hints (`"front"`, `"back"`, `"left"`, `"right"`) via a JSON string field in the service request.

### arm_pkg

Robot configuration and GPD-based grasp detection. `gpd_service` (C++) takes a pointcloud and returns ranked 6-DOF grasp poses using the GPD library. Also contains `moveit_configs_builder.py` and `moveit_configs_builder_sim.py` for building MoveIt configurations for real vs simulation environments.

### frida_pymoveit2

Adapter layer that defines xArm6-specific constants used throughout `frida_motion_planning`: move group name (`xarm6_arm`), end-effector link (`link_eef`), base link (`base_link`), and joint names.

### xarm6_ikfast_plugin

Analytical IKFast IK solver integrated as a MoveIt2 plugin. Faster and more reliable than numerical IK for xArm6's geometry.

### manipulation_general

Top-level package with competition-task launch files (`gpsr.launch.py`, `ppc.launch.py`, `restaurant.launch.py`, etc.) that combine the full manipulation stack with task-specific parameters.

## Key Interfaces (frida_interfaces/manipulation)

The main action and service interfaces other areas need to know about:

- `ManipulationAction` — top-level task: send object name + target location
- `PickMotion` / `PlaceMotion` / `PourMotion` — mid-level task actions
- `MoveToPose` / `MoveJoints` — low-level arm motion (Cartesian and joint-space)
- `HeatmapPlace` — get optimal placement point from a surface pointcloud
- `GraspDetection` — get ranked grasp poses from a pointcloud (GPD)
- Collision scene services: `AddCollisionObjects`, `RemoveCollisionObject`, `AttachCollisionObject`, `GetCollisionObjects`

## Configuration

- **`arm_pkg/config/xarm_params.yaml`**: set `debug: true` on the real robot to publish all required xArm services.
- **`frida_constants` (manipulation_constants)**: defines `CUTLERY_NAMES`, effort threshold (6.5 N), descent speed (20 mm/s), pick velocity (0.1–0.3 m/s), and placement max distance (0.35 m).
- All nodes accept `use_sim_time`: set `true` for MuJoCo simulation, `false` for real robot.

## Running

Start the manipulation area container:
```bash
./run.sh manipulation
```

With flags:
```bash
./run.sh manipulation --build          # build ROS 2 packages inside the container
./run.sh manipulation --build-image    # rebuild the Docker image
```

Run a competition task directly:
```bash
./run.sh --ppc       # Pick and Place challenge
./run.sh --gpsr      # General Purpose Service Robot
./run.sh --restaurant
```

For the full pick-and-place pipeline including vision integration, see `docs/Run/Areas/Manipulation/pick_and_place.md`.

> **Note**: Manipulation packages include git submodules. Before building for the first time, run:
> ```bash
> git submodule update --init --recursive
> ```

> **Note**: On the real robot (xArm6 at `192.168.31.180`), set `debug: true` in `arm_pkg/config/xarm_params.yaml` to publish all required services.
