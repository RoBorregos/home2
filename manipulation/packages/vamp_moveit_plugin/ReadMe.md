# VAMP Motion Planning Integration for FRIDA

VAMP (Vectorized And Motion Planning) is an ultra-fast motion planner for the FRIDA robot arm (xArm6). It uses SIMD-vectorized collision checking against a sphere-approximation of the robot to plan trajectories in **20–500 ms** wall-clock through the full MoveIt pipeline. When the sphere approximation misses a collision that MoveIt's mesh-based FCL checker catches, the plugin **automatically falls back to OMPL** — the user always gets either a valid plan or a clean failure, never a silently-bad trajectory.

**Paper:** Kingston et al., "Vamp: Vectorized Motion Planning" (2023) — [arXiv:2309.14545](https://arxiv.org/abs/2309.14545)

---

## Architecture

```
MoveIt2 (move_group)
    └── pipeline "vamp"
            └── VampPlannerManager (C++ plugin)
                    │  initialize(): loads OMPL plugin as fallback
                    └── VampPlanningContext::solve()
                            │
                            │  (1) Try VAMP via ROS2 service
                            ▼
                    vamp_server.py (Python)
                            ├── vamp.frida_real.rrtc()       ← RRT-Connect, sphere collision
                            ├── vamp.frida_real.simplify()   ← SHORTCUT + B-Spline
                            └── vamp.frida_real.validate()   ← sphere collision check
                            │
                            ▼
                    (2) Post-validate trajectory against scene with FCL (real meshes)
                            │
                            ├── PASS → return VAMP plan to MoveIt
                            │
                            └── FAIL → (3) runOmplFallback()
                                            │  up to 4 retries (RRTConnect is stochastic)
                                            ├── ompl_interface::OMPLPlanner::solve()
                                            ├── TOTG densify + time-parameterize
                                            ├── validate densified result against scene
                                            └── return first attempt that survives
```

VAMP's value: fast sphere-based planning for the typical case. The sphere model can never be perfect (it's an approximation by design), so the layered checks below catch the rare cases where the model and reality disagree:

1. **VAMP's own collision check** — sphere-vs-sphere, fast, approximate
2. **FCL post-validation inside the plugin** — mesh-based, runs in our context before returning, so MoveIt's outer pipeline never sees a bad plan from us
3. **OMPL fallback** — same FCL collision model that the post-validator uses, slower (~150–300 ms typical) but accurate. Plans whenever VAMP misses
4. **Outer MoveIt FCL post-validation** — final check on whatever we return, included for free

Callers and RViz see a single planner "vamp"; the layered behavior is internal.

The C++ plugin acts as a bridge between MoveIt and the Python VAMP server. The Python server handles all VAMP calls because the Python bindings are more stable than the C++ API for custom robots.

**Key files:**

| File | Purpose |
|------|---------|
| `manipulation/packages/vamp_moveit_plugin/scripts/vamp_server.py` | Python planning server |
| `manipulation/packages/vamp_moveit_plugin/src/vamp_planning_context.cpp` | C++ MoveIt bridge |
| `manipulation/packages/vamp_moveit_plugin/resources/frida_real.hh` | FRIDA collision model (68 spheres, full body) |
| `manipulation/packages/vamp_moveit_plugin/scripts/probe_cuboid.py` | Standalone diagnostic for sphere coverage + collision detection |
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

Inside the container:
```bash
source /workspace/install/setup.bash
ros2 launch arm_pkg frida_fake_moveit_config.launch.py
```

This **also starts `vamp_server.py` automatically** (the "vamp" pipeline is the default planner, so its backend must be running). To disable it — e.g. to exercise the OMPL fallback, or to run the server separately/remotely — pass:
```bash
ros2 launch arm_pkg frida_fake_moveit_config.launch.py start_vamp_server:=false
```
and then launch the server yourself in another terminal:
```bash
ros2 run vamp_moveit_plugin vamp_server.py   # PYTHONPATH is set by .bash_aliases
```

The same `start_vamp_server` argument exists on the real `frida_moveit_config.launch.py`.

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

### `VAMP plan failed FCL post-validation against current PlanningScene`
Expected log — not an error. VAMP's sphere model said the plan was clean but FCL caught a real collision. The plugin should immediately log `Falling back to OMPL (attempt 1/4)...` and recover automatically.

**When to act:** if the fallback ALSO fails repeatedly for cases that should be solvable (e.g. an obstacle clearly outside the robot's path), the cause is usually URDF geometry off (a sensor link mounted in the wrong place). Use `probe_cuboid.py` to inspect sphere positions and compare against the visual mesh in RViz.

### `Exec format error: vamp_server.py`
The installed script lost its execute permission.

**Fix:**
```bash
chmod +x /workspace/install/vamp_moveit_plugin/lib/vamp_moveit_plugin/vamp_server.py
```

### MoveIt shows "Failed" but server shows "SUCCESS"
MoveIt's internal path validator rejected the trajectory. **This should now self-recover** — the plugin's `runOmplFallback()` fires whenever VAMP's plan fails FCL post-validation. Expect to see in the logs:

```
VAMP plan failed FCL post-validation against current PlanningScene
Falling back to OMPL (attempt N/4)...
OMPL fallback SUCCESS | ...
```

**When to act:** if you see `OMPL fallback FAILED after 4 attempts`, neither planner can find a valid path. That usually means the goal is genuinely unreachable (start/goal in true collision, obstacle blocks everything). Re-evaluate the request — VAMP+OMPL agree there's no plan.

### `Segmentation fault` on second planning request
The C++ executor is conflicting with move_group's executor.

**Fix:** Ensure `vamp_planning_context.cpp` uses a dedicated child node for the service client (not `rclcpp::spin_until_future_complete(node_, ...)`). This is already fixed in the current code.

---

## Design Decisions

**Why Python server + C++ bridge instead of pure C++?**
VAMP's Python bindings are more stable for custom robots. The C++ API requires recompiling VAMP itself when adding new robots. The Python server can be updated without recompiling.

**Why SHORTCUT+BSPLINE instead of a custom smoother?**
The native VAMP simplifier is guaranteed collision-free by construction (Kingston et al. 2023). A moving-average filter is faster to implement but can push waypoints into obstacles.

**Why 68 spheres?**
The current model covers the full FRIDA: arm links (24 spheres across link_base + link1..6), gripper + fingers (20 spheres — kept tight for picking precision), base_link chassis (8), and head sensors zed_camera_link/intel_realsense/laser (14). 68 was chosen by FOAM's `--branch 8` parameter as a balance between coverage accuracy and FK cost per plan check. The earlier 82-sphere model was geometrically broken (all spheres stacked at link joint origins — see commit `9ef5bb6a` for the diagnosis and fix); the 68-sphere model from `vampTest/cricket/frida.hh` has proper FK distribution.

**Why `security_margin=0.02`?**
Small default — the OMPL fallback handles the cases where the sphere model under-covers what FCL would catch. Larger margins (0.05+) help VAMP avoid more obstacles directly but block close-approach picking of small objects. If you find yourself raising the margin, see if the corner case can be addressed by the fallback layer instead — that keeps the gripper precise.

---

## Safety architecture — debugging the layered fallback

The planner uses 4 layers of defense; understanding which layer fired tells you where to debug.

### Expected logs by scenario

**Happy path — VAMP succeeds, FCL agrees:**
```
[move_group] VAMP request: 0 spheres, 1 boxes
[vamp_server] SUCCESS | 56 waypoints | total=22.0 ms
[move_group] VAMP SUCCESS | 99.7 ms | 56 waypoints
[move_group] Motion plan was computed successfully
```
No fallback fired. VAMP's sphere model and FCL agree this plan is collision-free.

**Sphere model under-covers, fallback fires, OMPL succeeds first try:**
```
[move_group] VAMP SUCCESS | 109.6 ms | 56 waypoints
[move_group] VAMP plan failed FCL post-validation against current PlanningScene
[move_group] Falling back to OMPL (attempt 1/4)...
[move_group] OMPL fallback SUCCESS | attempt 1/4 | 258.8 ms total | 139 waypoints
[move_group] Motion plan was computed successfully
```
The sphere-vs-mesh disagreement was caught and recovered. Total wall-clock ≈ 155 ms.

**OMPL retry needed (TOTG densification grazed a sensor link):**
```
[move_group] Falling back to OMPL (attempt 1/4)...
[move_group] OMPL attempt 1: TOTG-densified trajectory has a colliding state (...); retrying
[move_group] Falling back to OMPL (attempt 2/4)...
[move_group] OMPL fallback SUCCESS | attempt 2/4 | 377.3 ms total | 113 waypoints
```
RRTConnect is stochastic so attempt 2 has different waypoints. The retry loop handles this transparently — the user sees a single Plan request that succeeds.

**Genuinely no valid plan (start or goal in real collision, or obstacle blocks all motion):**
```
[move_group] VAMP planning failed — trying OMPL fallback
[move_group] Falling back to OMPL (attempt N/4)...
[move_group] OMPL attempt N: solve() failed (code 99999)
... 4 attempts ...
[move_group] OMPL fallback FAILED after 4 attempts | ... ms total — caller will receive PLANNING_FAILED
[move_group] No motion plan found. No execution attempted.
```
The system correctly tells the caller no plan exists. No execution, no crash.

### Diagnosing sphere-model coverage

When `VAMP plan failed FCL post-validation` fires frequently for cases that VISUALLY look fine, the sphere model probably under-covers the link involved. Use `probe_cuboid.py` to inspect:

```bash
# Edit BOX_CENTER, BOX_HALF, START, GOAL at the top of the script to match
# the failing case (the move_group log prints these), then:
python3 /workspace/src/manipulation/packages/vamp_moveit_plugin/scripts/probe_cuboid.py
```

The "All sphere positions" section shows where the robot's 68 spheres land for a state on the failing trajectory. The "n_penetrating" column tells you how many spheres VAMP thinks intersect the obstacle. If FCL flags a state but n_penetrating is 0, the sphere model is missing coverage somewhere — usually a link with few spheres (link5/link6 have only a few each) or a sensor mounted at an awkward angle (zed_camera_link).

### Tunable knobs vs. fixing the model

| Symptom | Quick fix | Real fix |
|---|---|---|
| Fallback fires often on link3-6 sweeps | Raise `security_margin` to 0.04-0.05 | Re-FOAM body links with denser `--branch` |
| Fallback fires often on ZED graze | (none clean) | Verify ZED mount position in `frida_description/.../gripper.xacro`, fix URDF |
| OMPL fallback exhausts 4 retries often | Raise OMPL `state_validation_resolution` in `ompl_planning.yaml` | Same as ZED case — usually URDF geometry issue |
| VAMP takes >1s on every plan | Lower `max_iterations` | Tune `range` (RRT step) for your scene size |

---

## CI build note

The repo's `ros2-build.yml` exports `GITHUB_ACTIONS=false` before `colcon build` to work around a bug in `vamp/cmake/Python.cmake`: under `if($ENV{GITHUB_ACTIONS})` it sets `STUB_PREFIX=""` and reuses the same relative path for `nanobind_add_stub(OUTPUT ...)` and `install(FILES ...)`, but those resolve against the build dir and source dir respectively — the install step then fails with `file INSTALL cannot find __init__.pyi`. Forcing the env var to false makes vamp take the else branch which uses an absolute build-dir-prefixed path consistently. Verified locally against a `ros:humble` container reproducing the original CI failure.