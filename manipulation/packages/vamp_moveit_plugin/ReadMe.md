# vamp_moveit_plugin

MoveIt 2 (Jazzy) planner plugin that plans the xArm6 with [VAMP](https://github.com/KavrakiLab/vamp)
(Kingston et al., [arXiv:2309.14545](https://arxiv.org/abs/2309.14545)) and falls back to OMPL whenever
VAMP cannot deliver a path that MoveIt accepts. Callers see one pipeline, `vamp`; the default pipeline
on the robot is still `ompl`.

## How it works

```
move_group ── pipeline "vamp" ── VampPlannerManager
                                   └─ VampPlanningContext::solve()
                                        1. goal: sample a valid state from the goal constraints (IK for pose goals)
                                        2. VAMP: send start, goal and the PlanningScene to vamp_server.py
                                        3. check: time-parameterize (TOTG) and validate with FCL, as ValidateSolution does
                                        4. OMPL fallback when any step fails, retried if its timed path collides
```

- `src/scene_conversion.cpp` mirrors the PlanningScene for VAMP: spheres, oriented boxes, cylinders
  (capsules), meshes and cones as bounding spheres, the octomap at full voxel resolution, and attached
  bodies as spheres in the end effector frame.
- `scripts/vamp_server.py` builds the VAMP environment, runs RRT-Connect and VAMP's simplifier. It
  drops octomap points the robot overlaps at the start state (its own surface), never at the goal.
- The OMPL fallback reads its planner configs from the `vamp` namespace, so the `vamp` pipeline is the
  `ompl` pipeline plus `config/vamp_planning.yaml` (see `arm_pkg/vamp_pipeline.py`).
- On every request the server compares VAMP's end effector with MoveIt's and logs an error when they
  differ by more than `model_tolerance`: the collision model is out of date.

## Using it

The sim has a flag for it:

```bash
./run.sh simulation --build          # builds the plugin and VAMP's Python bindings
./run.sh simulation --manip --vamp   # vamp as default pipeline + vamp_server
```

Any other MoveIt launch needs `add_vamp_pipeline(moveit_config, default=True)` and
`vamp_moveit_plugin/launch/vamp_server.launch.py`. A request can also pick it with `pipeline_id: vamp`.

## Benchmark

Planning only: both pipelines get the same 6 joint goals in an open scene and with a wall in front of the
arm. It prints success, OMPL fallbacks, median/p95/max planning time and median joint path length per
scene and pipeline, and writes one CSV row per request (5 trials = 120 requests per pipeline, ~10 s):

```bash
./run.sh simulation --manip --vamp --headless --no-rviz
ros2 run vamp_moveit_plugin benchmark_vamp.py --ros-args -p trials:=5 -p csv:=/tmp/vamp_benchmark.csv
```

Full pick and place, 3 runs per planner; compare the placed/attempts report and the elapsed time:

```bash
for planner in "" --vamp; do
  for run in 1 2 3; do
    ./run.sh simulation --manip --headless --no-rviz $planner
    time ./run.sh simulation --tm
  done
done
```

## Parameters

Pipeline (`config/vamp_planning.yaml`, under `vamp.`): `service_name`, `end_effector_link`,
`request_timeout` (s to wait for the server before OMPL), `fallback_attempts`.

Server (`config/vamp_server.yaml`): `robot` (VAMP module), `collision_margin` (m added to every
obstacle), `model_tolerance` (m), `range` and `max_iterations` (RRT-Connect).

## Building VAMP

VAMP is the `manipulation/packages/vamp` submodule. Its Python bindings are not part of the colcon
build: `docker/manipulation/setup_vamp.sh` copies `resources/frida_real.hh` into it, registers the
robot and builds `_core_ext*.so` (the sim's `--build` runs it).

## Collision model

`resources/frida_real.hh` is generated code: [FOAM](https://github.com/CoMMALab/foam) spherizes the
collision meshes and [Cricket](https://github.com/CoMMALab/cricket) compiles the spheres into VAMP's
vectorized forward kinematics. Regenerate it whenever the robot geometry or the arm mount changes; the
server's model-check error says when.

```bash
./run.sh simulation --manip                                          # any running move_group works
bash manipulation/packages/vamp_moveit_plugin/model/generate_frida_real.sh  # host, needs Docker
./run.sh simulation --build                                          # reinstalls the header, rebuilds VAMP
./run.sh simulation --manip --vamp
ros2 run vamp_moveit_plugin check_collision_model.py                 # FK error and self-collision agreement
```

`export_robot_model.py` takes the URDF, SRDF and meshes from move_group itself, so the model always
matches what MoveIt checks; joints outside the `xarm6` group (the fingers) are frozen at their zero,
open position. FOAM and Cricket are pinned to the commits that match the VAMP submodule. The generated
`frida_real_spherized.urdf` and `frida_real.srdf` are kept next to the header.
