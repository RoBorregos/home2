#!/usr/bin/env python3
"""
MCP server exposing FRIDA's skills as tools.

Purpose is development, not competition: it lets an agent drive the real robot
conversationally ("go to the kitchen, what do you see?") instead of writing another
one-off script under scripts/test/. The tool list is generated from the skill
registry, so it is the same contract the behaviour tree and selector use.

Safety: read-only skills (perception, speech, queries) are exposed by default.
Anything that moves the robot requires --allow-actuation, and --dry-run reports what
would be called without calling it.

Run inside a ROS-enabled container:
    python3 frida_mcp_server.py --allow-actuation
"""

import argparse
import json
import sys
import threading
from typing import Any

import rclpy
from rclpy.node import Node

from task_manager.skills.registry import REGISTRY, call_skill, describe
from task_manager.utils.run_log import RunLog
from task_manager.utils.subtask_manager import SubtaskManager, Task
from task_manager.world.world_model import WorldModel

TYPE_MAP = {
    "str": "string",
    "float": "number",
    "int": "integer",
    "bool": "boolean",
    "point": "object",
}


class FridaMCP(Node):
    """Holds the subtask managers and serves skill calls from one background spin."""

    def __init__(self, allow_actuation: bool, dry_run: bool, mock_areas: list):
        super().__init__("frida_mcp_server")
        self.allow_actuation = allow_actuation
        self.dry_run = dry_run
        self.subtask_manager = SubtaskManager(self, task=Task.DEBUG, mock_areas=mock_areas)
        self.world = WorldModel()
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self) -> None:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self)
        executor.spin()

    def available(self) -> list[str]:
        return sorted(
            name
            for name, skill in REGISTRY.items()
            if self.allow_actuation or not skill.mutates_world
        )

    def call(self, name: str, arguments: dict) -> dict:
        if name not in REGISTRY:
            return {"error": f"unknown skill '{name}'"}
        skill = REGISTRY[name]
        if skill.mutates_world and not self.allow_actuation:
            return {"error": f"'{name}' moves the robot; restart with --allow-actuation"}
        if self.dry_run:
            return {"dry_run": True, "skill": name, "arguments": arguments}
        try:
            outcome = call_skill(self.subtask_manager, name, **arguments)
        except Exception as error:  # noqa: BLE001 — report, never kill the server
            return {"error": f"{type(error).__name__}: {error}"}
        status, result = _split(outcome)
        self.world.apply(name, status)
        return {"status": status, "result": _stringify(result)}


def _split(outcome: Any) -> tuple[str, Any]:
    if isinstance(outcome, tuple) and outcome:
        status, result = outcome[0], outcome[1] if len(outcome) > 1 else None
    else:
        status, result = outcome, None
    return getattr(status, "name", str(status)), result


def _stringify(value: Any, limit: int = 4000) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    text = str(value)
    return text if len(text) <= limit else text[:limit] + "…"


def tool_schema(name: str) -> dict:
    """Registry entry -> MCP tool definition."""
    info = describe(name)
    properties, required = {}, []
    for arg in info["args"]:
        properties[arg["name"]] = {
            "type": TYPE_MAP.get(arg["type"], "string"),
            "description": arg["description"] or arg["name"],
        }
        if arg["required"]:
            required.append(arg["name"])
    note = "" if info["mutates_world"] else " (read-only)"
    return {
        "name": name,
        "description": info["summary"] + note,
        "inputSchema": {"type": "object", "properties": properties, "required": required},
    }


class StdioServer:
    """Minimal JSON-RPC over stdio — avoids depending on an MCP SDK inside the robot image."""

    def __init__(self, robot: FridaMCP):
        self.robot = robot

    def serve(self) -> None:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
            except json.JSONDecodeError:
                continue
            response = self.handle(request)
            if response is not None:
                sys.stdout.write(json.dumps(response) + "\n")
                sys.stdout.flush()

    def handle(self, request: dict) -> dict | None:
        method = request.get("method")
        request_id = request.get("id")
        if method == "initialize":
            return _ok(
                request_id,
                {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": "frida", "version": "0.1.0"},
                },
            )
        if method == "tools/list":
            tools = [tool_schema(name) for name in self.robot.available()]
            tools.extend([_world_tool(), _manifest_tool()])
            return _ok(request_id, {"tools": tools})
        if method == "tools/call":
            params = request.get("params", {})
            name = params.get("name", "")
            arguments = params.get("arguments", {}) or {}
            if name == "get_world_state":
                payload = self.robot.world.snapshot()
            elif name == "get_run_id":
                payload = {"run_id": RunLog.run_id()}
            else:
                payload = self.robot.call(name, arguments)
            return _ok(
                request_id,
                {"content": [{"type": "text", "text": json.dumps(payload, default=str)}]},
            )
        if method and method.startswith("notifications/"):
            return None
        return _ok(request_id, {})


def _ok(request_id: Any, result: dict) -> dict:
    return {"jsonrpc": "2.0", "id": request_id, "result": result}


def _world_tool() -> dict:
    return {
        "name": "get_world_state",
        "description": "Current world model: held object, location, known objects, elapsed time.",
        "inputSchema": {"type": "object", "properties": {}, "required": []},
    }


def _manifest_tool() -> dict:
    return {
        "name": "get_run_id",
        "description": "Id of the run log currently being written, if a run is open.",
        "inputSchema": {"type": "object", "properties": {}, "required": []},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allow-actuation", action="store_true", help="expose moving skills")
    parser.add_argument("--dry-run", action="store_true", help="report calls without making them")
    parser.add_argument("--mock", default="", help="comma-separated areas to mock")
    args = parser.parse_args()

    rclpy.init()
    robot = FridaMCP(
        allow_actuation=args.allow_actuation,
        dry_run=args.dry_run,
        mock_areas=[area for area in args.mock.split(",") if area],
    )
    try:
        StdioServer(robot).serve()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
