# FRIDA MCP server

Exposes the skill registry (`task_manager/task_manager/skills/registry.py`) as MCP tools so
an agent can drive the robot conversationally during development, instead of writing another
one-off script under `task_manager/scripts/test/`.

It is a **development tool**. Competition runs use the behaviour-tree executor, not this.

## Safety model

| Mode | What is exposed |
|---|---|
| default | read-only skills only — perception, speech, queries, map lookups |
| `--allow-actuation` | everything, including navigation and manipulation |
| `--dry-run` | reports the call it *would* make, executes nothing |
| `--mock vision,navigation` | runs against `SubtaskManager` mocks, no hardware |

"Read-only" comes from `Skill.mutates_world` in the registry, so adding a skill
automatically lands it in the right bucket.

## Running it

Inside the ROS container:

```bash
python3 /workspace/src/task_manager/scripts/frida_mcp_server.py --allow-actuation
```

`.mcp.json` at the repo root wires it into Claude Code via `docker exec` against
`home2-hri-ros-${ENV_TYPE}`. Start the HRI stack first (`./run.sh hri`), then restart
Claude Code so it picks the server up.

To try it with no robot at all:

```bash
python3 task_manager/scripts/frida_mcp_server.py --mock vision,navigation,manipulation,hri
```

## Tools

Every registry entry becomes a tool with a generated JSON schema, plus two extras:

- `get_world_state` — held object, location, known objects, predicates, objectives done
- `get_run_id` — id of the run log currently being written

## Adding a skill

Add a `Skill(...)` to `registry.py`. That single entry gives you the MCP tool, the
behaviour-tree dispatch entry, the selector's precondition check, and the duration prior.
`task_manager/scripts/test/test_skills_registry.py` will fail if the declaration is
inconsistent (duplicate args, effects that both set and clear a predicate, missing defaults).
