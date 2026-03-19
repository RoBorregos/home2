# VAMP Motion Planning Integration for FRIDA

VAMP (Vectorized And Motion Planning) is an ultra-fast motion planner that replaces OMPL for the FRIDA robot arm (xArm6). It uses SIMD-vectorized collision checking to plan trajectories in **10–60ms** vs. hundreds of milliseconds with OMPL.

**Paper:** Kingston et al., "Vamp: Vectorized Motion Planning" (2023) — [arXiv:2309.14545](https://arxiv.org/abs/2309.14545)

---

## Architecture

```
MoveIt2 (move_group)
    └── VampPlannerManager (C++ plugin)
            └── VampPlanningContext
                    │  async ROS2 service call
                    ▼
            vamp_server.py (Python)
                    │
                    ├── vamp.frida_real.rrtc()       ← RRT-Connect planning
                    ├── vamp.frida_real.simplify()   ← SHORTCUT + B-Spline smoothing
                    └── vamp.frida_real.validate()   ← Collision checking
```

The C++ plugin acts as a bridge between MoveIt and the Python server. The Python server handles all VAMP calls because the Python bindings are more stable than the C++ API for custom robots.

**Key files:**

| File | Purpose |
|------|---------|
| `manipulation/packages/vamp_moveit_plugin/scripts/vamp_server.py` | Python planning server |
| `manipulation/packages/vamp_moveit_plugin/src/vamp_planning_context.cpp` | C++ MoveIt bridge |
| `manipulation/packages/vamp_moveit_plugin/resources/frida_real.hh` | FRIDA collision model (82 spheres) |
| `robot_description/frida_description/urdf/xarm/spherized_xarm/xarm6.urdf` | Spherized URDF used to generate the model |
| `docker/manipulation/setup_vamp.sh` | Container setup script |
| `docker/manipulation/generate_frida_collisions.sh` | HOST script to regenerate `frida_real.hh` |

---

## Setup from Scratch (New Developer)

### Prerequisites
- Docker installed and running
- The repository cloned with submodules: `git clone --recursive <repo>`

### Step 1 — Generate the collision model (HOST, one-time)

This runs **outside Docker** and uses the Cricket Docker image to generate `frida_real.hh`:

```bash
cd <repo_root>
bash docker/manipulation/generate_frida_collisions.sh
```

This script:
1. Checks that the spherized URDF exists at `robot_description/frida_description/urdf/xarm/spherized_xarm/xarm6.urdf`
2. Runs Cricket (via Docker) to generate `frida_real.hh` with 82 collision spheres
3. Copies the `.hh` to `manipulation/packages/vamp_moveit_plugin/resources/`

> If you skip this step, VAMP will still compile but won't know FRIDA's geometry and all states will be invalid.

### Step 2 — Build and enter the container

```bash
./run.sh manipulation
```

On first run, `setup_vamp.sh` automatically:
- Copies `frida_real.hh` into the VAMP submodule
- Registers `frida_real` in `vamp/cmake/Python.cmake`
- Builds VAMP via `colcon build`
- Copies the compiled `.so` to `vamp/src/vamp/_core/`
- Exports `PYTHONPATH` so `import vamp` works

### Step 3 — Launch and plan

In terminal 1 (inside container):
```bash
source /workspace/install/setup.bash
ros2 launch arm_pkg frida_fake_moveit_config.launch.py
```

In terminal 2 (inside container):
```bash
export PYTHONPATH="/workspace/src/manipulation/packages/vamp/src:$PYTHONPATH"
ros2 run vamp_moveit_plugin vamp_server.py
```

Then open RViz, set the planning group to `xarm6`, set a goal state, and click **Plan**.

---

## Regenerating `frida_real.hh` (Robot Geometry Changed)

Run this whenever the robot's URDF or meshes change (new gripper, camera mount, etc.):

### Step 1 — Re-spherize with FOAM (HOST)

FOAM converts STL meshes into sphere approximations. The spherized URDF is already committed at `robot_description/.../xarm6.urdf`, but if the robot changes you need to re-run FOAM:

```bash
# From the repo root on the HOST
mkdir -p /tmp/foam_frida/meshes/xarm
cp robot_description/frida_description/urdf/TMR2025/frida_real.urdf /tmp/foam_frida/
cp robot_description/frida_description/urdf/TMR2025/meshes/xarm/*.stl /tmp/foam_frida/meshes/xarm/

docker run --rm -it \
  --entrypoint /bin/bash \
  -v ~/roborregos/vampTest/foam:/foam \
  -v /tmp/foam_frida:/workspace \
  foam-image \
  -c "cd /foam && pip install -e . -q && python3 scripts/generate_sphere_urdf.py \
    --filename /workspace/xarm6_only.urdf \
    --output /workspace/xarm6_spherized.urdf \
    --branch 8 --depth 1 --shrinkage 1.0"

# Copy result back
cp /tmp/foam_frida/xarm6_spherized.urdf \
   robot_description/frida_description/urdf/xarm/spherized_xarm/xarm6.urdf
```

> **Note:** `branch=8` controls spheres per link. Higher = more accurate but slower planning. 8 gives ~82 total spheres which balances accuracy and speed well.

### Step 2 — Regenerate the C++ header with Cricket

```bash
bash docker/manipulation/generate_frida_collisions.sh
```

### Step 3 — Rebuild VAMP

Inside the container:
```bash
rm -rf /workspace/src/manipulation/packages/vamp/src/impl/vamp/robots/frida_real.hh
bash /workspace/src/docker/manipulation/setup_vamp.sh
```

The setup script detects that the `.so` doesn't contain `frida_real` and triggers a full rebuild automatically.

---

## Configurable Parameters

All parameters are ROS2 parameters that can be overridden at launch:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_iterations` | `50000` | Max RRT-Connect iterations per attempt |
| `range` | `0.05` | RRT step size (radians). Smaller = safer near obstacles |
| `security_margin` | `0.08` | Padding added to all obstacles (meters) |
| `validation_step_size` | `0.05` | Interpolation resolution for path validation |
| `min_waypoint_distance` | `0.08` | Min joint-space distance between waypoints |
| `smoothing_window` | `5` | Window size for Python fallback smoother |
| `smoothing_passes` | `3` | Passes for Python fallback smoother |
| `max_retries` | `3` | Planning attempts before giving up |
| `retry_range_multiplier` | `1.5` | Range multiplier on retry |
| `retry_iterations_multiplier` | `2.0` | Iteration multiplier on retry |

Override at runtime:
```bash
ros2 run vamp_moveit_plugin vamp_server.py \
  --ros-args \
  -p security_margin:=0.05 \
  -p max_iterations:=100000
```

---

## Adding a New Robot to VAMP

### Step 1 — Spherize the robot with FOAM

You need a URDF with only the arm links (no base, no gripper) and the STL meshes accessible.

```bash
docker run --rm -it \
  --entrypoint /bin/bash \
  -v /path/to/foam:/foam \
  -v /path/to/your/robot:/workspace \
  foam-image \
  -c "cd /foam && pip install -e . -q && python3 scripts/generate_sphere_urdf.py \
    --filename /workspace/robot_arm_only.urdf \
    --output /workspace/robot_spherized.urdf \
    --branch 8 --depth 1"
```

### Step 2 — Generate the C++ header with Cricket

Create a config JSON:
```json
{
    "name": "MyRobot",
    "urdf": "resources/myrobot/spherized.urdf",
    "srdf": "resources/myrobot/myrobot.srdf",
    "end_effector": "link_eef",
    "resolution": 32,
    "template": "resources/templates/fk_template.hh",
    "subtemplates": [{"name": "ccfk", "template": "resources/templates/ccfk_template.hh"}],
    "output": "/tmp/myrobot.hh"
}
```

Run Cricket:
```bash
docker run --rm -it \
  --entrypoint /bin/bash \
  -v /path/to/cricket:/workspace \
  cricket-tool \
  -c "/tmp/build/fkcc_gen /tmp/myrobot_config.json"
```

### Step 3 — Register the robot in VAMP

Copy the `.hh` to VAMP:
```bash
cp /tmp/myrobot.hh \
   manipulation/packages/vamp/src/impl/vamp/robots/myrobot.hh
```

Add to `manipulation/packages/vamp/cmake/Python.cmake`:
```cmake
list(APPEND VAMP_ROBOT_MODULES myrobot)
list(APPEND VAMP_ROBOT_STRUCTS MyRobot)
```

Rebuild VAMP and verify:
```python
import vamp
print(hasattr(vamp, 'myrobot'))  # Should print True
```

### Step 4 — Update vamp_server.py

Replace all references to `vamp.frida_real` with `vamp.myrobot`.

---

## Troubleshooting

### `frida_real not found` on import
```
AttributeError: module 'vamp' has no attribute 'frida_real'
```
**Fix:** The `.so` was compiled without `frida_real`. Run:
```bash
bash /workspace/src/docker/manipulation/setup_vamp.sh
```
If that doesn't help, force a rebuild:
```bash
rm /workspace/src/manipulation/packages/vamp/src/vamp/_core/_core_ext*.so
bash /workspace/src/docker/manipulation/setup_vamp.sh
```

### `Start INVALID (self-collision/joint-limits)`
VAMP thinks the robot's current state is in collision with itself or out of joint limits.

**Fix:** Check that `frida_real.hh` has reasonable sphere radii (should be 0.02–0.09m, not 0.1m uniform):
```python
import vamp
r = vamp.frida_real
print('n_spheres:', r.n_spheres())      # Should be 82
print('min_radius:', r.min_max_radii()) # Should be ~(0.02, 0.09)
```
If spheres look wrong, regenerate `frida_real.hh` using the steps above.

### `VAMP timeout after 10.0 s`
The C++ plugin sent a request but never received a response.

**Cause:** `vamp_server.py` is not running.
**Fix:** Start the server in a separate terminal:
```bash
export PYTHONPATH="/workspace/src/manipulation/packages/vamp/src:$PYTHONPATH"
ros2 run vamp_moveit_plugin vamp_server.py
```

### `Path has internal collision`
VAMP found a path but it collides after post-processing.

**Fix:** Increase `security_margin` so VAMP plans further from obstacles:
```bash
ros2 run vamp_moveit_plugin vamp_server.py --ros-args -p security_margin:=0.10
```

### `Exec format error: vamp_server.py`
The installed script lost its execute permission.

**Fix:**
```bash
chmod +x /workspace/install/vamp_moveit_plugin/lib/vamp_moveit_plugin/vamp_server.py
```

### MoveIt shows "Failed" but server shows "SUCCESS"
MoveIt's internal path validator rejected the trajectory.

**Fix:** Ensure `check_solution_paths: false` is set in `frida_moveit_common.launch.py`:
```python
{"check_solution_paths": False},
```

### `Segmentation fault` on second planning request
The C++ executor is conflicting with move_group's executor.

**Fix:** Ensure `vamp_planning_context.cpp` uses a dedicated child node for the service client (not `rclcpp::spin_until_future_complete(node_, ...)`). This is already fixed in the current code.

---

## Design Decisions

**Why Python server + C++ bridge instead of pure C++?**
VAMP's Python bindings are more stable for custom robots. The C++ API requires recompiling VAMP itself when adding new robots. The Python server can be updated without recompiling.

**Why SHORTCUT+BSPLINE instead of a custom smoother?**
The native VAMP simplifier is guaranteed collision-free by construction (Kingston et al. 2023). A moving-average filter is faster to implement but can push waypoints into obstacles.

**Why 82 spheres?**
More spheres = more accurate collision checking = fewer false positives from MoveIt. 26 uniform spheres (the previous model) were too coarse and caused MoveIt to reject valid trajectories. 82 spheres from FOAM's mesh-based approximation match the actual robot geometry closely enough for MoveIt's FCL checker to agree.

**Why `security_margin=0.08`?**
The sphere model slightly underestimates the robot's volume. A margin of 8cm compensates for this and ensures VAMP plans paths that MoveIt's mesh-based checker also considers valid. You can reduce this as the sphere model improves.