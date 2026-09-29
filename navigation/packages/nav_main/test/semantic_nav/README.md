# Semantic navigation tests

Two levels, both runnable on a laptop — no robot, no Nav2, no Jetson.

## 1. Core self-test (no ROS at all)

Everything numeric — surface typing, route ordering, scan bookkeeping, costmap
validation — against the real maps in `map_context/maps`:

```bash
python3 navigation/packages/nav_main/scripts/semantic_nav_selftest.py
python3 navigation/packages/nav_main/scripts/semantic_nav_selftest.py --map robocup_c1
# installed:
ros2 run nav_main semantic_nav_selftest.py
```

Exits non-zero on failure. `WARN` lines are map-data problems (a furniture pose
inside an obstacle, two pieces tagged at the same spot) that the code absorbs but
that should be fixed with `map_area_tagger.py`.

## 2. Integration test (live ROS 2 node)

Runs the real `semantic_nav_node.py` against a stand-in for nav_central:
`fake_nav.py` serves the arena's `areas_<MAP>.json` on `AREAS_SERVICE`, publishes
the saved `.pgm` as `/global_costmap/costmap`, and broadcasts `map -> base_link`
at a pose the tests move around.

Covered: every `PlanPatrol` mode, area filtering, `max_viewpoints`, RViz markers,
the staleness marking loop, the JSON snapshot, and a viewpoint deliberately
blocked in the costmap to prove it gets relocated instead of dropped.

```bash
docker run --rm --network none \
  -v "$PWD":/ws/src:ro -v /tmp/semnav_out:/out ros:jazzy-ros-base bash -lc '
    source /opt/ros/jazzy/setup.bash
    mkdir -p /out/ws/src && cp -r /ws/src/frida_interfaces /out/ws/src/ && cd /out/ws
    colcon build --packages-select frida_interfaces >/dev/null 2>&1
    source install/setup.bash
    export REPO_ROOT=/ws/src HOME=/out ROS_DOMAIN_ID=77
    export PYTHONPATH=/ws/src/frida_constants:/ws/src/navigation/packages/nav_main:$PYTHONPATH
    T=/ws/src/navigation/packages/nav_main/test/semantic_nav
    python3 $T/fake_nav.py > /out/fake_nav.log 2>&1 &
    sleep 3
    python3 /ws/src/navigation/packages/nav_main/scripts/semantic_nav_node.py --ros-args \
      --params-file /ws/src/navigation/packages/nav_main/config/semantic_nav.yaml \
      -r __node:=semantic_nav > /out/node.log 2>&1 &
    sleep 8
    python3 $T/test_plan_patrol.py && python3 $T/test_staleness_and_costmap.py'
```

On the robot the same tests run against the real stack — just skip `fake_nav.py`
and start navigation normally.

Last run (2026-09-28, `ros:jazzy-ros-base`, laptop): selftest 41/41 on every map
in `map_context`, integration 17/17 + 14/14.
