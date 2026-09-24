# Status Dashboard

Live terminal view of what is running on the robot: host DDS setup, containers,
expected ROS nodes per task, topic rates and integration logs.

## How it starts

`./run.sh --<task>` (`run_task` in `lib.sh`) opens one screen window per area in
session `frida` plus a `status` window running the dashboard with the same args:

```bash
./run.sh --hric          # then: screen -r frida  (Ctrl+A " → status)
```

Manual use (host needs ROS Jazzy, `python3-rich`, `python3-yaml`):

```bash
bash scripts/dashboard.sh --hric           # all areas
bash scripts/dashboard.sh vision --hric    # one area
bash scripts/status.sh --hric              # one-shot text version, no Python deps
```

Extra `run.sh` flags (`--build`, `--recreate`, ...) are ignored. Tasks:
`--hric --ppc --gpsr --dlc --restaurant --finals --safety`.

## Panels

| Panel         | Source                                   | Meaning                                                               |
| ------------- | ---------------------------------------- | --------------------------------------------------------------------- |
| Host DDS      | `/etc/cyclonedds.*`, sysctl, `home2-roudi` | Output of `scripts/setup_cyclonedds.sh`; RouDi only checked when SHM is expected (Jetson or `CYCLONE_SHM=1`). RMW is informational. |
| Logs          | `docker logs home2-integration`          | Last task manager lines, colored by level.                           |
| Live signals  | `configs/critical_topics.yaml`           | Hz of key topics (camera, `/tf`, `/scan`, `/cmd_vel`).               |
| Containers    | `configs/<area>_infra.cfg`               | `docker ps` state. Names are prefix-matched (`home2-display` → `home2-display-l4t`). |
| Nodes         | `configs/<area>_nodes.cfg`               | Expected vs running nodes for the task, with the missing ones listed. |
| Orphan topics | ROS graph                                | Topics with subscribers but no publishers (red) or the reverse (yellow). |
| Hints         | all of the above                         | Suggested fix, e.g. `./run.sh vision --recreate`.                    |

HRI containers run on the second Orin (`ORIN_SERVER_AREAS` in `lib.sh`), so they
show as `remote Orin`; their nodes are still checked over DDS.

## Files

```
scripts/dashboard.sh        sources ROS Jazzy, checks deps, runs python3 -m status.dashboard
scripts/status.sh           one-shot bash report (check_nodes.sh + check_infra.sh)
status/dashboard.py         rich layout and refresh loop (2 s)
status/infra_checks.py      host DDS + container checks
status/ros_introspection.py rclpy probe node: nodes, topics, services, orphans, Hz
status/area_config.py       reads the bash .cfg files (single source of truth)
status/configs/             <area>_nodes.cfg, <area>_infra.cfg, critical_topics.yaml
```

## Keeping it up to date

Node lists are hand-kept against the launch files each area runs per task:

| Area         | Where the task → launch mapping lives           |
| ------------ | ----------------------------------------------- |
| vision       | `docker/vision/run.sh` → `vision_general/launch/*_launch.py` |
| manipulation | `docker/manipulation/run.sh` → `manipulation_general/launch/*.launch.py` |
| navigation   | `docker/navigation/run.sh` → `nav_main/launch/task_launch/` |
| integration  | `docker/integration/run.sh` (one task manager node per task) |
| hri          | `speech/launch/hri_launch.py` (same core nodes for every task) |

When a launch file adds, removes or renames a node, update the matching
`<area>_nodes.cfg`. Use the full node name as shown by `ros2 node list`.
Optional nodes (moondream container, KWS, respeaker, embeddings) are left out on
purpose so they don't show as false failures.

## Known limits

- The probe runs on the host with the default RMW; it discovers the Cyclone
  containers over UDP (all containers use `network_mode: host`, domain 0).
- Sampling the camera topic for Hz pulls images over loopback for 0.8 s every
  refresh; remove it from `critical_topics.yaml` if it costs too much.
- `--finals` has no dedicated launch in vision/manipulation/navigation, so only
  the ZED and HRI nodes are expected.
