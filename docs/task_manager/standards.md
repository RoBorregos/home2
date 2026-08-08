# Task Manager Standards

Conventions for writing and maintaining the competition task managers in
`task_manager/scripts/`.

These rules exist because our task managers grow fast during a competition
season and then become unreadable. The worst case today is
`scripts/pickandplace_task_manager.py`: **1758 lines, of which `run()` alone is
897** — a single method holding 22 inlined state bodies. That file is not bad
because it uses a state machine; it is bad because the state bodies were never
given names. Everything below is aimed at preventing exactly that.

## Table of Contents

- [Architecture: three layers](#architecture-three-layers)
- [File layout](#file-layout)
- [The FSM contract](#the-fsm-contract)
- [State method rules](#state-method-rules)
- [Managing task state](#managing-task-state)
- [Logging](#logging)
- [Retries and fallbacks](#retries-and-fallbacks)
- [Blocking calls and timeouts](#blocking-calls-and-timeouts)
- [When to use a Behavior Tree instead](#when-to-use-a-behavior-tree-instead)
- [Lint and formatting](#lint-and-formatting)
- [Migration guide: refactoring an existing manager](#migration-guide-refactoring-an-existing-manager)
- [Review checklist](#review-checklist)

## Architecture: three layers

Keep these layers strictly separated. Most of our mess comes from layer 1 doing
layer 3's job.

```
┌──────────────────────────────────────────────────────────────┐
│  Layer 1 — Task Manager        scripts/<task>_task_manager.py │
│  Mission flow only: which state comes next, retry/fallback     │
│  policy, task-specific decisions. NO ROS service plumbing.     │
└───────────────────────────┬──────────────────────────────────┘
                            │ calls
┌───────────────────────────▼──────────────────────────────────┐
│  Layer 2 — Subtask Managers    task_manager/subtask_managers/ │
│  nav_tasks / vision_tasks / manipulation_tasks / hri_tasks     │
│  One method per capability. Owns clients, timeouts, retries    │
│  of the ROS call itself. Returns Status (or (Status, result)). │
└───────────────────────────┬──────────────────────────────────┘
                            │ ROS 2 services / actions / topics
┌───────────────────────────▼──────────────────────────────────┐
│  Layer 3 — Area nodes          vision/ nav/ manipulation/ hri/ │
└──────────────────────────────────────────────────────────────┘
```

**Rules:**

- A task manager **never** creates a service or action client directly. If a
  capability is missing, add a method to the relevant subtask manager.
- Access everything through `self.subtask_manager` (see
  `task_manager/utils/subtask_manager.py`), which exposes `.nav`, `.vision`,
  `.manipulation`, `.hri`.
- Subtask manager methods must not know which task is calling them. Task-specific
  logic (e.g. "in PPC, cutlery goes to the dishwasher") lives in the task manager.
- Every subtask method returns `Status` or `(Status, result)` — see
  `task_manager/utils/status.py`. Never return bare booleans or `None`.

## File layout

Task manager files follow this section order. Keep the section banners — they
make a 700-line file navigable.

```python
#!/usr/bin/env python3
"""
Task Manager for <Task> of RoboCup @Home <year>

<2-4 line summary of the mission flow, including any non-obvious
constraint — e.g. "all scan results are stored in the MAP frame at scan
time because the robot moves between scan and use.">
"""

# 1. Imports — stdlib, then third-party, then frida/task_manager
# 2. Module constants — grouped with `# ── section ──` comments and units
# 3. Enums and dataclasses (ObjectCategory, Location, ...)
# 4. class <Task>TM(Node):
#      4a. class TaskStates
#      4b. __init__
#      4c. # ── helpers ──          (small, pure-ish, testable)
#      4d. # ── state methods ──    (one per TaskStates entry)
#      4e. run()                    (dispatch only — see below)
# 5. main()
```

Constants get units in a comment. This is already done well in
`scripts/restaurant_task_manager.py:32-52` — copy that style:

```python
CUSTOMER_STANDOFF = 2.0       # m from the caller for the table scan
PAN_SETTLE_TIME = 1.0         # s to let the camera settle after a pan
MAX_BASE_ROTATIONS = 2        # initial heading + 2 rotations ≈ full circle
```

## The FSM contract

**A finite state machine is the correct model for our tasks.** Do not replace it
with anything else (see
[When to use a Behavior Tree instead](#when-to-use-a-behavior-tree-instead)).
What is *not* correct is the `if/elif` chain.

### Required: dict dispatch, one method per state

```python
class TaskStates:
    WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
    START = "START"
    PERCEIVE_TABLE = "PERCEIVE_TABLE"
    ...
    END = "END"


def __init__(self):
    ...
    self.current_state = TaskStates.WAIT_FOR_BUTTON
    self._states = {
        TaskStates.WAIT_FOR_BUTTON: self._state_wait_for_button,
        TaskStates.START:           self._state_start,
        TaskStates.PERCEIVE_TABLE:  self._state_perceive_table,
        ...
        TaskStates.END:             self._state_end,
    }


def run(self):
    """Dispatch one state per call. No logic lives here."""
    handler = self._states.get(self.current_state)
    if handler is None:
        Logger.error(self, f"Unknown state: {self.current_state}")
        self.current_state = TaskStates.END
        return
    self._track_state_change(self.current_state)
    handler()
```

### Why this is mandatory

1. **`run()` stays ~8 lines forever.** It cannot grow.
2. **Exactly one state runs per call.** Today `scripts/restaurant_task_manager.py`
   uses `if` (states fall through, several run per tick) while
   `scripts/hric_task_manager.py` uses `elif` (one per tick). Identical-looking
   code, different semantics — a guaranteed source of confusing bugs. Dict
   dispatch removes the ambiguity by construction.
3. **Cross-cutting concerns happen in one place.** State timing, attempt-counter
   reset, display publishing and the `old → new` log line all belong in
   `_track_state_change`. `scripts/pickandplace_task_manager.py:346` already
   implements this well — it just has to be pasted into all 22 blocks today.
4. **States become unit-testable.** You can call
   `tm._state_determine_placement()` with a fake `grasped_object` and assert the
   resulting `current_state`. You cannot test a branch of a 900-line method.
5. **A missing state is caught, not silently skipped.** The `if/elif` chain falls
   through to nothing when a state name is typo'd.

### State method naming

- One method per state, named `_state_<lowercased_state_name>`.
- The mapping must be mechanical: `TaskStates.PERCEIVE_TABLE` →
  `_state_perceive_table`. No exceptions, so you can always jump from a log line
  to the code.

## State method rules

**Hard limit: a state method is at most ~60 lines.** If it is longer, extract
sub-steps as helpers. For reference, the current offenders:

| State | Lines today | Action |
| --- | --- | --- |
| `PLACE_OBJECT` (PPC) | 153 | Split by placement location |
| `SCAN_CABINET_SHELVES` (PPC) | 111 | Split scan / parse / store |
| `PICK_OBJECT` (PPC) | 76 | Extract confirmation logic |
| `PICK_BREAKFAST_ITEM` (PPC) | 71 | Extract into shared pick helper |

A long state method is almost always a dispatcher in disguise. Make it explicit:

```python
def _state_place_object(self):
    loc = self.grasped_object.placement_location
    if   loc == Location.DISHWASHER: status = self._place_in_dishwasher()
    elif loc == Location.TRASH_BIN:  status = self._place_in_trash()
    elif loc == Location.CABINET:    status = self._place_on_shelf()
    else:                            status = self._place_generic()
    self._advance_after_place(status)
```

**A state method must end by setting `self.current_state`** (or explicitly
documenting that it re-enters itself, e.g. a retry state). Make the transition
the last statement so it is easy to find:

```python
def _state_perceive_table(self):
    status, detections = self.subtask_manager.vision.detect_objects()
    if status != Status.EXECUTION_SUCCESS or not detections:
        self.current_state = TaskStates.START_BREAKFAST_PREP
        return
    self.detected_objects = [ObjectInfo(d) for d in detections]
    self.current_state = TaskStates.ANNOUNCE_OBJECTS
```

**Do not put blocking `while` loops inside a state** unless it is a genuine
wait-for-external-event (start button, door). Loops over collections belong in a
loop *state* that advances an index, so the FSM stays interruptible and each
iteration gets logged. `CLEANUP_LOOP` in
`scripts/pickandplace_task_manager.py:1045` is the right pattern.

## Managing task state

`scripts/pickandplace_task_manager.py.__init__` declares **37 instance
attributes**. That is the second-biggest readability problem after `run()`.

**Group related mutable state into dataclasses.** Scratch state that belongs to
"one object", "one cabinet visit" or "one customer cycle" should live together:

```python
@dataclass
class CabinetVisit:
    """Per-visit cabinet scratch state. Reset with `self.cabinet = CabinetVisit()`."""
    scan_fresh: bool = False
    fallback_heights: list[float] = field(default_factory=list)
    fallback_idx: int = 0
```

**Why this matters more than it looks:** our most common logic bug is a
*forgotten reset*. `scripts/restaurant_task_manager.py:114` has a `_reset_cycle()`
clearing 8 separate fields, called from 3 places — miss one and the next customer
inherits the previous customer's `approach_attempts`. With a dataclass, the reset
is one assignment and cannot be partial.

**Rules:**

- Anything reset together must be grouped together.
- Every group gets exactly one reset site: `self.<group> = <Group>()`.
- Long-lived task state (`bar_pose`, `detected_objects`) stays as plain
  attributes — grouping only applies to per-cycle scratch state.
- Prefer `Enum` over string constants for domain values (`Location`,
  `ObjectCategory` in PPC are the model to follow).

## Logging

Two utilities, with distinct jobs. Do not invent a third.

**`CLog`** (`task_manager/utils/colored_logger.py`) — subsystem-tagged, for
tracing mission flow. Use this in task managers.

```python
from task_manager.utils.colored_logger import CLog

CLog.fsm(self,    "STATE",  f"{old} → {new}")
CLog.nav(self,    "MOVE",   "Moving to kitchen")
CLog.manip(self,  "PICK",   f"Failed to pick {name}", level="error")
CLog.vision(self, "DETECT", f"{len(detections)} objects found")
CLog.hri(self,    "SAY",    "Announcing order")
```

Subsystems: `fsm`, `nav`, `manip`, `vision`, `hri`. Levels: `info`, `success`,
`warn`, `error`.

**`Logger`** (`task_manager/utils/logger.py`) — generic level logging, used
inside subtask managers and for `Logger.run_test` in `scripts/test/`.

**Rules:**

- Every state transition is logged exactly once, by `_track_state_change` — never
  log transitions manually inside a state method.
- Log the *decision*, not just the action: `"cutlery → dishwasher"` beats
  `"placing object"`. When reviewing a failed run you need to know why the robot
  chose what it chose.
- Failure paths always log at `warn` or `error` with the reason. A silent
  `return` is a bug.
- Do not log inside tight loops — you will drown the console during a run.

## Retries and fallbacks

Retries are policy and belong to the task manager; the subtask manager handles
only the timeout of a single call.

**Standard shape**, using a module-level `ATTEMPT_LIMIT`:

```python
ATTEMPT_LIMIT = 3

for attempt in range(ATTEMPT_LIMIT):
    status = self.subtask_manager.manipulation.pick_object(label)
    if status == Status.EXECUTION_SUCCESS:
        return Status.EXECUTION_SUCCESS
    CLog.manip(self, "PICK", f"attempt {attempt + 1}/{ATTEMPT_LIMIT} failed", level="warn")
return self._deus_pick(label)
```

**Always provide a human fallback ("deus ex machina") for scoring actions.** In
RoboCup, asking a human for help costs some points; failing the action costs all
of them. The reference implementation is
`scripts/restaurant_task_manager.py:257` (`deus_pick`) — ask, confirm with
`hri.confirm(..., use_keyword=True, retries=3)`, then continue.

**Extract shared fallbacks.** `pick_with_fallback` and `place_with_fallback` are
currently duplicated in near-identical form across restaurant, PPC and
clean_table. New duplicates should not be added — factor the shared version into
`task_manager/utils/` and call it from all three.

## Blocking calls and timeouts

All subtask methods are **synchronous** — they call
`rclpy.spin_until_future_complete(...)` internally (e.g.
`subtask_managers/nav_tasks.py:222`, `manipulation_tasks.py:371`) with timeouts
up to 60 s. Consequences you must design around:

- **A state method blocks the whole node** while a subtask call is in flight.
  Nothing can preempt it. Do not write code that assumes a timer or subscriber
  will fire mid-call.
- **Never `time.sleep()` in a task manager.** Use the node's `timeout()` helper
  so callbacks keep running:

  ```python
  def timeout(self, duration: float = 2.0):
      start = time.time()
      while (time.time() - start) < duration:
          rclpy.spin_once(self, timeout_sec=0.1)
  ```

- Timeout enforcement belongs in the subtask manager (`timeout_sec=` on the
  spin), not in the task manager.
- Decorate subtask methods with `@mockable` and `@service_check` so a missing
  service returns an error `Status` instead of hanging forever. See
  [decorators.md](decorators.md).

## When to use a Behavior Tree instead

We use `py_trees` (pinned in `task_manager/requirements/requirements.txt`) in
**exactly one place**: GPSR, in `task_manager/task_manager/gpsr/`. That is
deliberate.

**Use a BT only when the plan is generated at runtime.** GPSR qualifies: the
`merger.py` interleaves multiple parsed commands into a plan that is not known
until the robot hears them, and `bt_builder.py` assembles a
`Selector(interleaved, sequential_fallback)` around it. There is no clean FSM
equivalent for that.

**Use an FSM for everything else.** All other tasks are fixed scripts written
before the run. Converting them to BTs would:

- move the same long state bodies into equally long `Behaviour.update()` methods
  — the code does not get shorter, only relocated;
- replace typed instance attributes with stringly-typed blackboard keys;
- buy no reactivity, because our leaves block (this is documented in our own
  `gpsr/leaf_behaviours.py:8-12`) so `Timeout` and `Retry` decorators can only
  act *between* ticks, never preempt a running action.

**Do not introduce BT XML.** `py_trees` has no supported XML loader, and Groot2
targets BehaviorTree.CPP, not `py_trees` — the tooling payoff does not exist for
our stack. Build trees with a Python builder function, as `gpsr/bt_builder.py`
already does.

Reusable BT subtrees are fine to share, since a tickable object can be driven
from either a BT or an FSM state.

## Lint and formatting

- `task_manager/` uses its own `ruff.toml`: `line-length = 100`, `E501` ignored.
- `ruff` and `ruff-format` run via pre-commit (`.pre-commit-config.yaml`). Run
  `pre-commit run --all-files` before opening a PR.
- Type-hint helper signatures and dataclass fields. Full annotation of state
  methods is not required (they all return `None`).
- Every state method and helper gets a one-line docstring. Reserve multi-line
  docstrings for non-obvious constraints — the TF-frame note in
  `restaurant_task_manager.py:125` is a good example of a comment that has
  prevented real bugs.

## Migration guide: refactoring an existing manager

Target: `scripts/pickandplace_task_manager.py`, but the recipe applies to any of
them. **This is a behavior-preserving refactor.** Do not fix logic and restructure
in the same commit — if a run regresses you need to know which change caused it.

### Step 0 — Baseline

Confirm the task still runs (mocked is fine) and record the current shape:

```bash
./run.sh --ppc
git rev-parse HEAD    # note the known-good commit
```

Work on a branch. Commit after each step so any step can be reverted alone.

### Step 1 — Split `run()` into state methods

The mechanical core of the refactor, and ~80% of the readability win.

1. For each `elif self.current_state == TaskStates.X:` block, create
   `def _state_x(self):` in the `# ── state methods ──` section.
2. **Cut and paste the body verbatim**, then de-indent by two levels. Change
   nothing else — not a variable name, not a log string.
3. Delete the now-duplicated `self._track_state_change(...)` call from the top of
   each body; the dispatcher does it.
4. Replace any bare `return` used to exit the state — it still works, since the
   state body is now its own function.
5. Build the `self._states` dict in `__init__`, and replace `run()` with the
   8-line dispatcher above.

Verify with:

```bash
git diff --stat        # only pickandplace_task_manager.py should change
git diff -w            # -w ignores whitespace: the remaining diff should be
                       # method boundaries and the new dispatcher, nothing else
```

That `git diff -w` check is the whole safety argument for this step — if it shows
logic changes, you did too much.

### Step 2 — Break up the oversized state methods

Only now, with each state visible in isolation, split the four states listed in
[State method rules](#state-method-rules). Extract sub-steps as private helpers
(`_place_on_shelf`, `_place_in_dishwasher`, ...). Bodies move unchanged; only the
enclosing `def` is new.

### Step 3 — Group instance state into dataclasses

Take the 37 attributes in `__init__` and cluster them by "what resets together".
Introduce one dataclass per cluster, then replace each multi-line reset block with
a single assignment. Grep for every old attribute name afterwards to be sure none
survive.

### Step 4 — Deduplicate across managers

With PPC readable, the shared helpers become obvious. Move `pick_with_fallback`,
`place_with_fallback` and `approach_and_look_at` into `task_manager/utils/`, and
update restaurant, PPC and clean_table to import them. This is the first step that
touches more than one file — do it last and in its own PR.

### Step 5 — Roll out to the remaining managers

Apply Step 1 only (dispatcher + state methods) to `restaurant`, `hric`,
`doing_laundry`, `clean_table`, `hand`. Each is a few hours.

`scripts/storing_groceries_manager.py` is a special case: it has **no
`TaskStates` class at all** — just a 1086-line file with the flow inlined. It
needs states identified and named first, so treat it as a separate task, not part
of this rollout.

⚠️ **`restaurant_task_manager.py` needs a decision before converting.** It
currently uses `if` (not `elif`), so several states run per `run()` call. Confirm
whether that fall-through is intentional before moving to one-state-per-tick
dispatch — if it is, the states need to be merged or explicitly chained.

### Effort

| Step | Scope | Estimate |
| --- | --- | --- |
| 1 | PPC dispatcher + 22 state methods | ~half a day |
| 2 | Split 4 oversized states | ~half a day |
| 3 | Dataclass grouping | ~half a day |
| 4 | Shared helpers across 3 managers | ~half a day |
| 5 | Remaining 5 managers, Step 1 only | ~2 days |

## Review checklist

Use this on every task manager PR.

- [ ] `run()` is dispatch only, no logic
- [ ] One method per state, named `_state_<state_name>`
- [ ] No state method exceeds ~60 lines
- [ ] Every state method ends by setting `self.current_state`
- [ ] No new service/action clients created in the task manager
- [ ] Subtask calls go through `self.subtask_manager.<area>`
- [ ] All subtask return values checked against `Status`, never truthiness
- [ ] Per-cycle scratch state grouped in a dataclass with one reset site
- [ ] State transitions logged only by `_track_state_change`
- [ ] Failure paths log at `warn`/`error` with a reason
- [ ] Scoring actions have a human fallback
- [ ] No `time.sleep()` — uses `self.timeout()`
- [ ] Constants at module level with units in a comment
- [ ] No copy-pasted helper that already exists in `task_manager/utils/`
- [ ] `pre-commit run --all-files` passes
