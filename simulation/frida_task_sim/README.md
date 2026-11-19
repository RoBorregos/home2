# frida_task_sim

Task-level regression harness on top of `frida_gz_sim`. It runs the **real** task
manager headless in Gazebo against a seeded rulebook scenario, scores the run from
ground truth against the 2026 scoresheet and saves a replayable report.

```bash
./run.sh simulation --build                 # once, and after pulling
./run.sh simulation --task ppc --seed 3     # one run (add --gui for the Gazebo window)
./run.sh simulation --scenario docker/simulation/logs/runs/<run>/scenario.json   # replay
./run.sh simulation --suite --tasks ppc --seeds 1,2,3   # several seeds + summary.md
./run.sh simulation --task ppc --seed 3 --hold          # stack only, to drive it by hand
./run.sh simulation --task ppc --seed 3 --spec <spec.json>   # another PPC spec
./run.sh simulation --test                  # unit tests, no simulator
```

While it runs the terminal shows the task as a referee would see it:

```
[00:16] REACHED the dining table
          +15 pts  Navigate to the table   (score 15)
[00:32] FRIDA: "I see a sponge."
[00:32] RECOGNIZED  sponge at dinner_table
          +10 pts  Correctly recognize an object   (score 25)
```

## What a run does

1. `scenario.py` builds the scenario from `(task, seed, venue)`: which objects, where,
   which category is trash, what the cabinet shelves hold.
2. `world.py` writes the Gazebo world: the map walls, the venue furniture, the objects,
   an arena door and a top-view camera, all in map coordinates.
3. `run_scenario.py` starts Gazebo, manipulation, navigation and the harness nodes,
   then the task manager, presses the start button and stops everything at the time limit.
4. `referee.py` watches Gazebo poses, robot speech and task topics and emits events.
5. `scoring.py` turns events into scoresheet rows; `report.py` writes the report.

Harness nodes (`launch/task_sim.launch.py`):

| Node | Stands in for |
|---|---|
| `gt_object_detector.py` | The object detector: same `/vision/detection_handler` service and detections topic, fed from Gazebo poses with a field-of-view and occlusion check, plus seeded misses and mislabels |
| `fake_hri.py` | The speech stack: logs what the robot says and holds the call as long as a voice would |
| `referee.py` | The human referee, including opening the arena door after the start signal |
| `recorder.py` | Records the top view with clock, score, events and speech drawn on it |

Navigation, MoveIt, the pick and place pipeline and the task manager are the real ones.

## Run directory

`docker/simulation/logs/runs/<task>_seed<N>_<time>/`

| File | Content |
|---|---|
| `report.html`, `report.md` | Score table, timeline and video |
| `overview.mp4` | Top view with the live score |
| `score.json` | Scoresheet rows and totals |
| `events.jsonl`, `transcript.jsonl` | Referee events and speech, in task seconds |
| `scenario.json` | The scenario; pass it to `--scenario` to replay the same setup |
| `final.json` | Where every object ended |
| `tm.log`, `stack_logs/`, `bt/` | Task manager log, stack logs, behaviour tree leaf log and state snapshots |

Exit code 0 means the run was scored, 2 means the stack or the task manager failed.

## Scoring

`config/scoresheets/<task>.yaml` lists every row of the rulebook sheet. Rows marked
`simulated: false` (dishwasher door and rack, tab slot, pouring, opening the milk) are
shown in the report but never scored, and the report states the score reachable in sim.

To add a row: add it to the YAML and count it in the task's function in `scoring.py`.

## Adding things

- Object: add it to `OBJECTS` in `catalog.py` (it must exist in
  `frida_constants/data/objects.json` to have a category).
- Venue: add `config/venues/<map>.yaml` with the furniture in map coordinates, placed in
  front of the matching sublocations of `areas_<map>.json`.
- Task: a generator in `scenario.py`, a scoresheet, a counter in `scoring.py` and an
  entry in `TASK_MANAGERS` in `run_scenario.py`.

## CI

- `.github/workflows/sim-tests.yml`: the unit tests on every pull request (hosted runner).
- `.github/workflows/sim-tasks.yml`: the suite, nightly and on manual dispatch, on a
  self-hosted runner labelled `frida-sim` (GPU + the sim image). It never runs on pull
  requests because the repo is public. The score table lands in the job summary and the
  run directories are uploaded as an artifact.

Register the runner once on the machine that will run it (Settings > Actions > Runners >
New self-hosted runner), adding the label `frida-sim`.

## Known limits

- Grasp assist welds a grasp that ends within 6 cm of the object (`GRASP_MAX_GAP` in
  `run_scenario.py`; 3.5 cm in the plain sim): task runs judge task logic, not grasp quality.
- The cabinet is a guess at the real one; the shelf viewing poses do not see levels 2 and 3
  from the docked pose, so shelf scans come back empty there.

- Objects are coloured primitives; perception fidelity is not what this tests.
- The base has no collisions (see `frida_gz_sim`), so driving through furniture is not caught.
- Time is sim time: a slow machine does not shorten the 7 minutes.
