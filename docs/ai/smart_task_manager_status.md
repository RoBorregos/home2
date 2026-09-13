# Smart task manager — implementation status

Companion to `smart_task_manager_plan.md`. What exists in the repo today, what is
verified, and what is explicitly not done yet.

Target: RoboCup@Home 2027. Pilot: Pick & Place. Runtime LLM: Ollama on the Orin.

## What is built

| Milestone | Delivered |
|---|---|
| M1 | `@measured` + run log, `functools.wraps` fix, asynchronous `ActionLeaf`, `Deadline`, `Budget` |
| M2 | Skill registry, world model, `TaskRunner` base, MCP server |
| M3 | Brief schema + parser, capability manifest, score-aware selector, replay tool |
| M4 | `BriefRunner`, `SkillLeaf`/`build_objective_tree`, Pick & Place entry point |
| M5 | Laundry / HRIC / Restaurant briefs, event triggers |
| M6 | Finals brief, problem-category repetition scoring |
| M7 | LLM brief compiler, offline test runner, CI |

### Key files

```
task_manager/task_manager/
  skills/registry.py        36 declared skills: args, preconditions, effects, priors
  world/world_model.py      held object, location, objects, predicates, attempts
  planner/brief.py          YAML brief parser and step syntax
  planner/manifest.py       measured success rate / duration, blended with priors
  planner/selector.py       expected points per second under a deadline
  executor/objective_tree.py  registry-dispatching async behaviour tree leaves
  executor/brief_runner.py  select -> run -> update loop; replaces the FSMs
  briefs/*.yaml             pick_and_place, doing_laundry, hric, restaurant, finals
  gpsr/skill_runner.py      single worker so blocking skills never overlap
  gpsr/bt_decorators.py     Deadline + Budget
task_manager/scripts/
  fit_capabilities.py       run logs -> capabilities.yaml
  replay_selector.py        replay past runs, compare against the recorded FSM ordering
  compile_brief.py          rulebook section -> draft brief, parser as verifier
  frida_mcp_server.py       skills as MCP tools, read-only by default
  ppc_brief_task_manager.py / brief_task_manager.py
```

## Verified

`python3 task_manager/scripts/test/run_offline_tests.py` — 8 suites, ~4 s, no ROS:

- the behaviour-tree leaf returns RUNNING, so `Timeout` and `Deadline` actually fire
- a pick+place cycle is ranked against pouring by points per second
- a free fallback is taken exactly when doing it ourselves is worse
- the safe posture refuses a coin-flip pour; aggressive attempts it
- a full Pick & Place run fits in 420 s, picks nothing twice, and survives a dead gripper
- one unpickable object does not retire the objective for the others
- every shipped brief parses, uses only registered skills, and matches its rulebook budget
- Finals spreads across problem categories instead of farming trash
- an invented skill in a compiled brief is rejected and fed back for correction

Also green: ruff 0.8.4 check + format (the version pinned in `.pre-commit-config.yaml`).

## Not done — read this before trusting any of it

**Nothing here has run on the robot.** Every number in every brief and in the manifest
priors is an estimate. The plan's own gates are unmet:

1. **No real capability manifest.** `@measured` is wired into `pick_object`, `place`,
   `place_on_shelf`, `pour`, `move_to_location` and `detect_objects` only. Until a real
   run produces `~/frida_runs/*.jsonl`, the selector is deciding on registry priors —
   educated guesses, not measurements. Run one Pick & Place, then
   `python3 task_manager/scripts/fit_capabilities.py --by object`.
2. **The M4 gate is unmet**: beat 170 points in a timed rehearsal. Not attempted.
3. **A preempted skill cannot be cancelled.** `SkillRunner.abandon` leaves the call
   running on its worker and queues the next one behind it, so the tree moves on but the
   arm does not stop. Real cancellation needs the subtask managers to expose cancellable
   ROS action goals. This matters for the 30-second inactivity rule.
4. **No folding primitive.** `doing_laundry.yaml`'s `fold_shirt` is a placeholder that
   will not score. Flagged in the brief itself.
5. **Most triggers have no detector.** Only `doorbell` is wired
   (`hri.arm_door_detection`). `customer_calling`, `person_raised_hand`, `guest_seated`,
   `bag_offered` and `guest_at_door` must be fired manually via `fire_trigger()` until
   detectors exist. `brief_task_manager.py` warns about this at startup.
6. **Scoresheet values are transcribed from the 2026 rulebook.** They decide what the
   robot chases, so re-verify each one against the 2027 rulebook when it is published.
7. **The old task managers are untouched and still the fallback.** Nothing was deleted.
8. **Briefs bypass no-lookahead limits.** A zero-point enabler (asking for the dishwasher
   to be opened) is only selected once nothing else is admissible. That works, but it is
   a consequence of the ranking rather than reasoning about preconditions.

## Next step

Run `./run.sh --ppc` with the existing manager once to collect a real run log, fit the
manifest, then `python3 task_manager/scripts/replay_selector.py --verbose`. If the
selector's projected ordering does not beat the recorded one, fix that before putting
`ppc_brief_task_manager.py` on the robot.
