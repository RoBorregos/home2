# frida_interfaces

## MoveJoints.action definition

**Request:**
- `float64[] joint_positions` — Target joint positions in radians.
- `string[] joint_names` — Names of the joints.
- `float32 velocity` — Velocity scaling factor.
- `float32 acceleration` — Acceleration scaling factor.
- `string planner_id` — Identifier for the motion planner.
- `bool apply_constraint` — Whether to apply path constraints.

**Result:**
- `int32 success` — `1` if the trajectory was accepted, `0` otherwise.

**Feedback:**
- `string execution_state` — Current state of trajectory execution.

---

## Other Messages

### SomeMessage.msg

Description of `SomeMessage`...

### SomeService.srv

Description of `SomeService`...
