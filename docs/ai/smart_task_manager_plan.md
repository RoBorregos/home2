# Smart FRIDA — score-aware task execution

Target: RoboCup@Home 2027 (~July). Pilot task: Pick & Place. Runtime LLM: Ollama on the Orin.

## Context

**The problem is not that FRIDA lacks an LLM — it's that five of six tasks are hand-written FSMs that never look at the clock.**

Audited today:

- `scripts/` holds 6 task managers, ~4.5k lines. `pickandplace_task_manager.py` is 1759 lines: a nested
  `class TaskStates` of string constants (`:133-160`) dispatched by a flat 22-branch `if/elif` in `run()`
  (`:835-1730`), with implicit transitions (`self.current_state = ...` in 37 places). ~510 lines (29%) are
  state plumbing + `CLog` + `say()` boilerplate; ~180 are config.
- `main()` is **byte-for-byte identical** in all four managers (ppc `:1747`, laundry `:349`, restaurant `:662`,
  hric `:655`). `navigate_to`, `timeout()` spin-sleep, `_track_state_change`, and the start-button busy-wait are
  each duplicated 3-4× with silently divergent signatures and one buggy variant
  (`hric:166` uses `time.sleep`, blocking all callbacks).
- **No manager reads the clock.** `total_start_time` exists in ppc/hric but is consumed only by the post-hoc
  report in `END` (`ppc:1711-1720`). GPSR is the sole exception — `gpsr/timeouts.py:14` `GLOBAL_BUDGET_S = 300.0`,
  and it is the only manager built on a behaviour tree instead of an if/elif FSM.

The team already measured the consequence in `docs/task_manager/ppc/time_strategy_2026.md` (2026-06-30,
**written but explicitly NOT implemented**):

- P&P happy path takes **~14 min against a 420 s limit**. Aggregate navigation alone ≈465 s (15 legs × ~31 s).
  `place_on_shelf` costs 60-120 s per call, shelf-pick ~45 s.
- The robot therefore runs out of time *in cleanup*, so the **400-point pour cluster almost never executes**.
- Further, ~1000 pts of scoreable objectives are unreachable in the current code: plate pick (+100) is
  explicitly refused (`skip_names`, `ppc:998`), dishwasher-inside place (3×+70) is a `TODO` and unreachable by
  default config, and open-milk (400) and dishwasher-tab (+100/+160) are **not encoded at all**.
- Reality check from Incheon 2026: the **best P&P score was 170** (then 145/135/135/85/35/15, many 0s). Nobody
  approaches 3515. So the goal is not to attempt everything — it is to **reliably bank the cheap base and
  opportunistically reach the dense clusters.**

That doc is the right analysis, hand-specified for one task as hardcoded constants
(`CLEANUP_DEADLINE_S`/`BREAKFAST_TAIL_S`/`HARD_DEADLINE_S`). **This plan implements its Tier 0 as a computed
policy that generalizes to every task**, rather than as per-task magic numbers.

Intended outcome: one shared deadline-aware executor driven by per-task declarative briefs and a score-aware
selector; the LLM authors and repairs briefs offline instead of running the robot online.

---

## 1. Architecture

Five layers, deliberately at different clock speeds:

| Layer | Status | Home |
|---|---|---|
| **L0 Skills** — ~150 typed methods | exists, keep unchanged | `subtask_managers/{nav,vision,manipulation,hri}_tasks.py` |
| **L1 Skill registry** — schema, preconditions, effects | new | `task_manager/skills/registry.py` |
| **L2 World model + run log** | new | `task_manager/world/` |
| **L3 Selector** — score-aware scheduler | new | `task_manager/planner/selector.py` |
| **L4 Executor** — behaviour tree | exists, generalize | `gpsr/` → `task_manager/executor/` |
| **L5 LLM** — offline compiler, narrow runtime calls | partly exists | `hri/packages/nlp/scripts/llm_utils.py` |

**What already exists and must be reused rather than rebuilt:**

- `leaf_behaviours.py` and `bt_builder.py` are **already task-agnostic** — they know only "objects have an
  `.action` string", "handlers are a list", "handlers return `(Status, result)`". Only `merger.py` carries GPSR
  specifics (`_GRIPPER_ACQUIRES`/`_GRIPPER_RELEASES` at `:34-35`, the `"go_to"`/`"location_to_go"` literals).
- `nav_tasks.get_path_info(loc_b, subloc_b, loc_a, subloc_a)` (`nav_tasks.py:267`) returns **real Nav2 path
  distance in metres between named locations without moving the robot**. This is the selector's travel-cost
  oracle — no new service needed.
- `utils/exploration_planner.py` (207 lines) builds an all-pairs distance matrix over `areas.json` and a
  nearest-neighbour tour. **It has zero importers today** — it is the only location cost model in the repo.
  Revive it as the selector's distance cache instead of writing a new one.
- `utils/shelf_pick_logic.py` and `utils/grasp_confirmation.py` are already ROS-free, unit-tested, and shared
  across ppc + gpsr. Keep as the model for new pure-function modules.
- `utils/decorators.py` `@mockable` / `@service_check` already wrap every skill (see `docs/task_manager/decorators.md`).
  A third decorator slots in beside them with no call-site churn. ⚠️ Neither uses `functools.wraps` (only
  `wrapper.__name__` is copied), so signatures and docstrings are **already lost** — fix that first, since the
  registry wants to introspect skills.
- `gpsr/timeouts.py:16 ACTION_TIMEOUTS` is a **hand-authored per-action budget table** (`go_to`:45, `pick_object`:30,
  `find_person`:60, `follow_person_until`:160). It is exactly the skeleton the measured manifest replaces.
- `manipulation/packages/pick_and_place/pick_and_place/pick_benchmark.py:133` already writes
  `suite, object, trial, action_success, grasp_score, retained, duration_s` to CSV — **the only place in the repo
  emitting per-skill success + duration rows.** Copy its row shape for `@measured`.
- `hri/benchmarks/nlp/report.py:150-172` writes a per-model/per-task JSON report (`accuracy`, `cases`, `passed`,
  `avg_ttft_ms`) — the only existing artefact shaped like a capability manifest. Copy its file shape.
- `llm_utils.py` already exposes the LLM **as ROS services** (`generic_structured_output`, `llm_wrapper_service`,
  `command_interpreter`), defaulting to Ollama via `MODEL` in `frida_constants/hri_constants.py:61-70` (all `"qwen3"`).
  The planner's LLM calls go here; do not add a second LLM client.
- `SubtaskManager(node, task, mock_areas=[])` (`utils/subtask_manager.py`) already supports mocking every area —
  but **every manager passes `[]`**. This unused scaffolding is the offline test harness.

**The knowledge store is PostgreSQL + pgvector, not ChromaDB** (the `chromadb` line in `.gitignore:9` is stale).
`docker/hri/dockerfiles/Dockerfile.postgres` + a **persistent named volume** `postgres_data`; schema in
`docker/hri/init.sql`; three ROS services (`add_entry`, `query_entry`, `find_closest`) over
`postgres_adapter.py`. Tables: `items`, `actions`, `locations`, `command_history`, `knowledge`, `hand_location`.
Adding a collection is an additive change across 4 files — no new node, no new interface.

What that store does *not* give us, and why L2 is still needed:

- `items` and `locations` are **static build-time seeds**; `add_item`/`add_location` have **no production callers**.
  Nothing writes observed object→location facts at runtime.
- `command_history` is **GPSR-only** — no other manager writes a single row. It has **no duration column**, its
  `created_at` is never selected back, its `context` parameter is silently dropped (no such column), `status` is
  stored as the literal string `"Status.EXECUTION_SUCCESS"`, and there is **no run/episode grouping key**.
- **"What the robot is holding" is not tracked anywhere persistent** — only per-task booleans
  (`hric:91 carrying_bag`, `ppc:295 carrying`) lost on restart. The GPSR merger's gripper reasoning
  (`merger.py:149-187`) is hypothetical scheduling state over a candidate plan, not runtime truth.
- **Object → default-location knowledge is missing from shared constants.** The only such map is hardcoded in a
  one-off script: `scripts/misc/egsr_cut.py:105 CATEGORY_TO_LOCATION`. Promote it to `frida_constants/data/`
  alongside `objects.json` — the P&P brief needs it to resolve `obj.dest`.

---

## 2. The blocking-leaf fix (prerequisite for everything)

`ActionLeaf.update()` is fully synchronous and **never returns `RUNNING`** (`leaf_behaviours.py:107-123`;
acknowledged in its own docstring `:8-12` and in `timeouts.py:9-11`). Consequences:

1. The per-action `Timeout` decorators in `bt_builder.py:48-52` **can never fire** — py_trees converts a child
   to FAILURE only if it is `RUNNING` at the next tick. Real timeouts today live only inside subtask managers.
2. A whole healthy plan executes inside **one `tree.tick()`** (`gpsr_task_manager.py:350-355`), so nothing can
   be preempted and no deadline guard can interrupt work in flight.
3. This is safety-relevant, not just architectural: `place_on_shelf` can leave the arm planning for >30 s, and
   the **30-second inactivity rule removes the robot from the arena** (flagged in `time_strategy_2026.md §6`).

**Fix:** make `ActionLeaf` asynchronous. `initialise()` submits `self._method(action)` to a single-worker
`ThreadPoolExecutor`; `update()` returns `RUNNING` until the future completes, then maps `(Status, result)` as
today. ~50 lines in `leaf_behaviours.py`. This makes the existing `Timeout`/`Retry` decorators live, lets the
tick loop spin ROS while a skill runs, and is what a global deadline guard hangs off.

Add a `Deadline` decorator (absolute wall-clock, not duration) at the tree root, plus an `inactivity_watchdog`
that fails the running leaf if no skill progress is reported for 25 s.

---

## 3. Task brief schema

One YAML per task in `task_manager/briefs/`. Replaces the FSM, not the skills.

```yaml
task: pick_and_place
budget_s: 420
risk_posture: safe          # safe | aggressive  (see §5)

on_start: [wait_for_door, go_to(dining_table)]
on_deadline: [go_to_named_position(nav_pose)]     # always leave the arm safe

objectives:
  - id: perceive_table
    points: 135                                    # 15 navigate + ~120 recognize
    template: [detect_objects(dining_table), announce_objects]
    once: true

  - id: pick_place
    repeat_for: table_objects                      # bound at runtime from the world model
    points: 90                                     # 50 pick + 40 place
    requires: [object_detected, arm_free]
    template: [go_to(obj.surface), pick(obj), go_to(obj.dest), place(obj)]
    guard: not_fragile                             # plate excluded until flat-grasp is reliable

  - id: pour
    repeat_for: [cereal, milk]
    points: 200
    requires: [bowl_placed, holding(item)]
    template: [pour(item, bowl)]
    penalty_risk: -100                             # spill

  - id: open_dishwasher
    points: 400
    skill: open_dishwasher_door
    fallback: ask_referee_open                     # scoresheet: -0, free
```

Notes on schema decisions:

- **`repeat_for` binds at runtime** from the world model, so "12 objects" is never hardcoded.
- **Prerequisite chains live inside `template`**, not in a backward-chaining planner. Deliberate v1 simplification:
  deterministic and sufficient for all six tasks. Upgrade path is a real precondition solver if Finals needs it.
- **`fallback` is first-class** because the P&P scoresheet makes dishwasher-door and milk-opening assistance
  cost **−0** — asking is free, and the selector must know that.
- **`penalty_risk`** feeds the risk posture; objectives that can score negative are gated differently.

Event-driven tasks (HRIC, Restaurant) add `triggers:` alongside `objectives:`:

```yaml
triggers:
  - on: doorbell_detected
    when: at(start_position)
    objective: receive_guest
```

---

## 4. Capability manifest

`task_manager/config/capabilities.yaml`, **generated from logs, never hand-written**:

```yaml
pick:
  default:    {p: 0.60, p50: 35, n: 0}
  contexts:
    - {match: {object_class: cutlery},           p: 0.45, p50: 42, n: 18}
    - {match: {object_class: box, surface: shelf}, p: 0.88, p50: 45, n: 31}
place_on_shelf:
  default:    {p: 0.75, p50: 90, n: 22}          # measured 60-120s
go_to:
  default:    {p: 0.95, p50: 31, n: 140}         # measured ~31s/leg
```

Data source — a third decorator beside the existing two, so no call sites change:

```python
@measured("pick")                    # new
@mockable(return_value=..., delay=2)
@service_check("pick_client", ...)
def pick_object(self, ...): ...
```

`@measured` appends one JSONL line per skill call (`skill`, context keys, `Status`, duration, task, **run id**) to
`~/frida_runs/<run_id>.jsonl`. `scripts/fit_capabilities.py` folds those into the YAML, keeping `n` so the
selector can fall back to a prior for thin contexts.

**Why a file and not the Postgres store:** the DB is persistent and already wired, but writing to it on every
skill call adds a service round-trip and a failure mode inside the 420 s budget, and `command_history` would need
schema changes anyway (no duration, no run id, `status` stored as a display string). A JSONL append is crash-safe,
costs nothing, and needs no running DB to replay offline. Revisit a `skill_runs` table only if cross-machine
aggregation becomes the bottleneck.

**A run id is the missing primitive.** There is currently *no* episode concept anywhere in the repo — no run id,
no start/end marker, no grouping key, and no file-based structured log emitted during a real run. `CLog`/`Logger`
are stdout-only; `_track_state_change` computes `state_times` and throws them away at `END`. Introducing a run id
at door-open (which is also where `time_strategy_2026.md §0.1` says to re-anchor the clock) is the single change
that makes every later phase measurable.

---

## 5. Selector

`task_manager/planner/selector.py` — a **pure function**, no ROS, unit-testable offline:

```python
def choose(brief, world, manifest, elapsed) -> Objective | None
```

Loop, re-run after every completed action:

1. `candidates` = objectives whose `requires` hold in `world` and that are not exhausted.
2. For each, `ev = p × points / (expected_duration + travel_time)` where `travel_time` comes from the
   `get_path_info` distance cache and `p`/`expected_duration` from the manifest.
3. Drop any candidate whose `p50` exceeds remaining time unless it offers partial credit.
4. If the objective has a free `fallback`, attempt autonomously only when `p × points > ev_of_best_alternative ×
   expected_duration`; otherwise take the fallback immediately. *(Worked example: P&P dishwasher door — a
   pick+place cycle is 90 pts / 45 s ≈ 2 pts/s, so 65 s of door attempt must return >130 pts, i.e. p > 33%. At
   the measured p=0.30, ask the referee and spend the 65 s on another object.)*
5. `risk_posture` scales the bar: `safe` requires a higher `p` and refuses objectives with `penalty_risk`;
   `aggressive` lowers it. This exists because **each task runs on 3 competition days and only the best score
   counts** (`time_strategy_2026.md §0.bis`) — bank a safe run one day, reach on another. It is a CLI flag, not
   a code change.
6. Return the winner; the runner expands its `template` into `PlanAction`s and hands them to the existing
   `build_tree`.

Grouping falls out of the formula: including travel time naturally batches objects sharing a surface, which is
what kills the measured 465 s of navigation.

---

## 6. Skill registry and MCP server

`task_manager/skills/registry.py` — one declaration per skill: name, typed args, preconditions, effects,
which `SubtaskManager` attribute implements it. Single source of truth feeding four consumers: BT leaf dispatch,
the selector's precondition checks, the BAML action types, and the MCP tool list.

This closes a real hazard: dispatch today is bare `getattr(handler, action_name)` over an ordered handler list
(`leaf_behaviours.py:23-28`), so **every public method is reachable by name** — `deus_pick`, `timeout`,
`test_function` included — with no arity or signature validation.

`frida_mcp` — an MCP server over that registry, running in a ROS-enabled container. Exposes skills as tools plus
read-only `get_world_state` / `get_capability_manifest` / `get_score_projection`. Primary value is **development**:
drive the real robot conversationally from Claude Code instead of writing another one-off script in
`scripts/test/`. Manipulation tools gated behind an explicit opt-in flag; dry-run mode by default.

---

## 7. Rollout

**M1 — Sep/Oct: measure and unblock.** `functools.wraps` on the two existing decorators; `@measured` + run id at
door-open + JSONL run logger; `fit_capabilities.py`; first manifest from real P&P/GPSR runs. Async `ActionLeaf`
+ `Deadline` decorator + inactivity watchdog (§2). *Nothing else in this plan can be calibrated until the
manifest exists — every threshold here is currently a guess.*

**M2 — Oct/Nov: registry + MCP.** `skills/registry.py`; `frida_mcp`; migrate `leaf_behaviours` dispatch from bare
`getattr` to the registry. Extract the duplicated `main()`/`navigate_to`/`timeout`/`_track_state_change`
boilerplate into a shared `TaskRunner` base — which also fixes step reporting, currently published by only 3 of 8
managers and by ppc on `/pickandplace/display/task_step`, a topic with no constant in `hri_constants.py`. Promote
`CATEGORY_TO_LOCATION` out of `scripts/misc/egsr_cut.py` into `frida_constants/data/`.

**M3 — Nov/Dec: brief + selector, offline.** World-model module; `briefs/pick_and_place.yaml` verified line by
line against the §5.2 scoresheet; selector as a pure function. **Test by replaying M1 logs — no robot.** Gate:
selector's chosen ordering beats the recorded FSM ordering on those same runs.

**M4 — Jan/Feb: P&P on the new stack.** Generalize `build_tree` to accept brief-derived plans (synthesize
`source_cmd=0`, pass `fallback=[]` — already handled at `bt_builder.py:134`). Run new vs. old behind the `Task`
enum. Gate: beat 170 pts in a timed rehearsal, and finish inside 420 s.

**M5 — Mar/Apr: laundry, then triggers.** Migrate `doing_laundry` (simplest, 361 lines). Add the trigger model
and migrate HRIC + Restaurant.

**M6 — May/Jun: Finals.** Greenfield — `run.sh:54` already advertises `--finals` but **no finals task manager
exists**. This is where the LLM genuinely earns its place: problem identification from an unmodeled scene.

**M7 — Jun/Jul: harden.** Offline/local-model validation, LLM-as-compiler loop (rulebook section + scoresheet →
draft brief → human review), competition dry runs.

---

## 8. Verification

- **Unit** — selector, brief parser, and manifest fitting are ROS-free pure functions; follow the existing
  `test_grasp_confirmation.py` pattern. Run in CI (`.github/workflows/pre-commit.yml`, `ros2-build.yml`).
- **Replay** — `scripts/test/replay_selector.py` feeds M1 JSONL logs through the selector and reports projected
  score vs. the score the FSM actually achieved on that run. This is the primary gate for M3.
- **Mocked end-to-end** — `SubtaskManager(..., mock_areas=["vision","navigation","manipulation","hri"])` runs a
  full brief with no hardware, asserting the brief reaches `END` under budget.
- **On robot** — `./run.sh --ppc` with the new runner; compare the `END` timing report and projected score
  against the old manager on the same arena setup.
- **MCP** — drive `pick`/`go_to`/`detect_objects` from Claude Code against the robot and confirm the world
  model updates.

---

## 9. Risks

- **VRAM on the Orin.** A local qwen3 shares the GPU with the vision stack, which `docs/ai/vision_optimization_plan.md`
  already flags as tight. That plan's §2.1 pose-model consolidation (5 engines → 1, ~1-2 GB) is effectively a
  **dependency** of the runtime-LLM path. Keep runtime LLM calls to a handful per run regardless.
- **Manifest overfitting.** Early `n` values will be small. Always keep a prior, never let the selector refuse
  an objective on `n < 5`, and hand-pin values the team is confident about.
- **Negative scoring.** Penalties are real (−40 thrown, −30 miscategorized, −100 spill, −100 per handover assist).
  The `guard` predicate and `risk_posture` are load-bearing, not decoration.
- **Scoresheet transcription.** The brief becomes what the robot optimizes, so a typo is a strategy bug. Verified
  the P&P sheet against the PDF (p.44) during planning; re-verify against the 2027 rulebook when it drops, since
  the objectives themselves may change.
- **Migration risk.** Old managers stay runnable behind the `Task` enum until each new brief beats them in a
  timed rehearsal. Never delete a working manager before its replacement has won once.

---

## 10. First commit

Copy this plan to `docs/ai/smart_task_manager_plan.md` (matching `vision_optimization_plan.md`'s role), then
start M1 — the `@measured` decorator and run logger, since every threshold in this plan is currently a guess.
