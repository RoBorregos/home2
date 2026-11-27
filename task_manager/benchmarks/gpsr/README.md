# GPSR experiment runs

Use the same Home2 commit, map, planner settings, and initial arena state for each pair. Keep a separate copy of the physical ground truth and record the run on video or with a ROS bag.

## Replay a parsed batch

Save a JSON array of exactly three nonempty action lists in the `CommandListLLM` schema. A `records` array from the paper benchmark can be converted with `[.records[].actions]` in `jq`. Reset the robot to the map's `start_location` and start the usual GPSR backends. Run one instance of the task manager per trial in the sourced integration environment.

```sh
ros2 run task_manager gpsr_task_manager.py --ros-args \
  -p parsed_batch_file:=/path/to/triple.json \
  -p batch_order:=merged -p recovery_policy:=skip-only \
  -p trial_log_file:=/path/to/merged.jsonl
```

Repeat with `batch_order:=issued` and a new log file. Both options use the same executor. For recovery, keep the batch and ordering fixed, and change `recovery_policy` among `sequential-fallback`, `skip-only`, and `skip-defer`. `replan_on_failure` defaults to false. `elapsed_before_batch_s` is the time already consumed at the start of a parsed replay and should be the same across the paired runs.

A missing or stale grasp reading is uncertain. The GPSR manipulation launch starts the existing grasp detector. Verify `/gripper/grasp_state` against the physical gripper before gathering data. The JSONL trace includes plan and phase events, source indices, action results, TF poses, grasp observations, and the return decision. Use external observation for goal success and physical stop times. The trial deadline gates dispatch and scoring. An outstanding synchronous call can outlive its deadline, including a call to the existing navigation service.

## Parked speech to plan

```sh
ros2 run task_manager gpsr_task_manager.py --ros-args \
  -p plan_only:=true -p trial_log_file:=/path/to/spoken.jsonl
```

The node listens for the first command and follows the usual confirmation and parsing path. Use the display button for subsequent commands. Once a batch has been planned and announced, the node exits before dispatching the skill tree. Use `parsed_batch_file` with `plan_only` to replay reference actions. Capture raw audio separately.

## Planner latency

Run from the Home2 checkout in a sourced environment. Supply the paper's `scenarios/benchmark_2027.json` and `sim/apartment.json`. These calls use fixed coordinates. Use separate instrumented traces for live map resolution and path lookups.

```sh
python3 task_manager/benchmarks/gpsr/runtime.py \
  --manifest ../gpsrPaper/scenarios/benchmark_2027.json \
  --layout ../gpsrPaper/sim/apartment.json --out /tmp/port-idle.json

git show 92800e8a2:task_manager/task_manager/gpsr/merger.py > /tmp/gpsr-historical.py
python3 task_manager/benchmarks/gpsr/runtime.py \
  --manifest ../gpsrPaper/scenarios/benchmark_2027.json \
  --layout ../gpsrPaper/sim/apartment.json --historical /tmp/gpsr-historical.py \
  --out /tmp/historical-idle.json
```

Repeat in separate processes under the normal Orin workload and record the software revisions and power mode. The output includes raw timings, nearest-rank p95, medians, segments, and fallback counts. `--settings` takes a JSON object of public `merge` keyword options, such as `holding_penalty`, `navigation_overhead`, `speed_m_s`, and `skill_durations_s`.
