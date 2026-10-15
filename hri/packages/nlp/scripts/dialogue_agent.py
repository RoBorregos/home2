#!/usr/bin/env python3
"""
Dialogue agent: multi-turn conversation where the LLM decides which schedule
lookups to make (OpenAI-style tool calling on the local llama.cpp server).

Service CONVESATION_SERVICE (frida_interfaces/AgentTurn): one call per user turn.
The task manager does the hearing and speaking; this node keeps the history,
runs the tools (Candidates Dashboard Robot API, GET only, with offline backup)
and returns what FRIDA should say.
"""

import json
import time
from datetime import datetime

import pytz
import rclpy
from frida_constants.hri_constants import LLM_ALIAS
from frida_constants.robot_api import ApiError, make_schedule_source
from frida_interfaces.srv import AgentTurn
from nlp.assets.dialogs import NO_THINKING, strip_thinking
from openai import OpenAI
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

TIMEZONE = "America/Monterrey"

SYSTEM_PROMPT = """You are FRIDA, a friendly receptionist robot built by RoBorregos (Tec de Monterrey).
You are talking out loud with a participant of the RoBorregos candidates program.
You can look up their interview and competition rounds with your tools.

Rules:
- Your words are spoken by a text-to-speech voice: answer in English, in one or two short sentences, no lists, no markdown, no emojis.
- To look someone up you need their full name, and ideally their team. Ask for whatever is missing.
- Always use the tools for schedule information. Never invent times, names or rooms.
- Read times exactly as the 'local' field says. For "my next round" use next_challenge (null means there is no upcoming round). If an interview is null it has not been scheduled yet; if rounds is empty they have not been published yet; if location is null you don't know the room yet.
- If several people match, ask which one they are, mentioning their teams, and wait for the answer before giving anyone's schedule.
- If the speech transcription seems wrong, ask them to repeat or spell their name.
- You can only help with the candidates' schedule and small talk about FRIDA and RoBorregos. Politely decline anything else.
- When the person says goodbye or has no more questions, call end_conversation and say a short goodbye.
Right now it is {now} in Monterrey."""

TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "search_candidate",
            "description": "Find registered candidates by their spoken name (tolerates misspellings). "
            "Pass the team if you know it to narrow the search.",
            "parameters": {
                "type": "object",
                "properties": {
                    "name": {
                        "type": "string",
                        "description": "Name as the person said it",
                    },
                    "team": {"type": "string", "description": "Team name, if known"},
                },
                "required": ["name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "search_team",
            "description": "Find a team by its spoken name.",
            "parameters": {
                "type": "object",
                "properties": {"name": {"type": "string"}},
                "required": ["name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_candidate",
            "description": "Interview and round schedule of one candidate (use an id from search_candidate).",
            "parameters": {
                "type": "object",
                "properties": {"candidate_id": {"type": "string"}},
                "required": ["candidate_id"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_team",
            "description": "Members (with their interviews) and rounds of a team (use an id from search_team or a candidate's teamId).",
            "parameters": {
                "type": "object",
                "properties": {"team_id": {"type": "string"}},
                "required": ["team_id"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "end_conversation",
            "description": "Call when the person says goodbye or has no more questions.",
            "parameters": {
                "type": "object",
                "properties": {"reason": {"type": "string"}},
                "required": [],
            },
        },
    },
]


def _slot(slot: dict | None, keys=("local", "location")) -> dict | None:
    """Keep only what the model should read (no UTC timestamps)."""
    if not slot:
        return None
    return {k: slot.get(k) for k in keys}


def _next_challenge(rounds: list) -> dict | None:
    """Earliest challenge that has not started yet (computed here, not by the LLM)."""
    now = datetime.now(pytz.utc).strftime("%Y-%m-%dT%H:%M:%S")
    upcoming = sorted(
        (
            (c.get("at", ""), r.get("round"), c)
            for r in rounds or []
            for c in r.get("challenges", [])
            if c.get("at", "") >= now
        ),
        key=lambda item: item[0],
    )
    if not upcoming:
        return None
    _, number, challenge = upcoming[0]
    return {"round": number, "track": challenge.get("track"), **_slot(challenge)}


def _label(value: str | None) -> str | None:
    return value.capitalize() if isinstance(value, str) and value.isupper() else value


def _rounds(rounds: list) -> list:
    return [
        {
            "round": r.get("round"),
            "challenges": [
                {"track": c.get("track"), **_slot(c)} for c in r.get("challenges", [])
            ],
        }
        for r in rounds or []
    ]


class DialogueAgent(Node):
    def __init__(self) -> None:
        super().__init__("dialogue_agent")
        self.declare_parameter("base_url", "http://localhost:11434/v1")
        self.declare_parameter("temperature", 0.2)
        self.declare_parameter("max_tool_calls", 4)
        self.declare_parameter("max_history_messages", 30)
        self.declare_parameter("request_timeout", 60.0)
        self.declare_parameter("CONVESATION_SERVICE", "/hri/nlp/conversation")

        param = self.get_parameter
        self.temperature = param("temperature").value
        self.max_tool_calls = param("max_tool_calls").value
        self.max_history = param("max_history_messages").value
        self.client = OpenAI(
            api_key="ollama",
            base_url=param("base_url").value,
            timeout=param("request_timeout").value,
        )
        self.sessions: dict[str, list] = {}
        self.sources = {
            False: make_schedule_source(offline=False),
            True: make_schedule_source(offline=True),
        }

        service = param("CONVESATION_SERVICE").value
        self.create_service(AgentTurn, service, self.turn)
        self.get_logger().info(f"DialogueAgent ready on {service}")

    # ----------------------------------------------------------------- tools

    def _run_tool(self, name: str, args: dict, offline: bool) -> dict:
        schedule = self.sources[offline]
        if name == "search_candidate":
            result = schedule.search_candidates(
                args.get("name", ""), team=args.get("team")
            )
        elif name == "search_team":
            result = schedule.search_teams(args.get("name", ""))
        elif name == "get_candidate":
            data = schedule.candidate(args.get("candidate_id", ""))
            interview = data.get("interview")
            result = {
                "name": data.get("name"),
                "team": data.get("team"),
                "teamId": data.get("teamId"),
                "track": _label(data.get("track")),
                "challenge": data.get("challenge"),
                "next_challenge": _next_challenge(data.get("rounds")),
                "interview": (
                    {
                        **_slot(interview),
                        "area": interview.get("areaLabel") or interview.get("area"),
                        "interviewer": interview.get("interviewer"),
                    }
                    if interview
                    else None
                ),
                "rounds": _rounds(data.get("rounds")),
            }
        elif name == "get_team":
            data = schedule.team(args.get("team_id", ""))
            result = {
                "name": data.get("name"),
                "track": _label(data.get("track")),
                "next_challenge": _next_challenge(data.get("rounds")),
                "members": [
                    {"name": m.get("name"), "interview": _slot(m.get("interview"))}
                    for m in data.get("members", [])
                ],
                "rounds": _rounds(data.get("rounds")),
            }
        elif name == "end_conversation":
            return {"ok": True}
        else:
            return {"error": f"Unknown tool {name}"}
        result["_source"] = schedule.last_source
        return result

    # --------------------------------------------------------------- history

    def _history(self, session_id: str, reset: bool) -> list:
        if reset or session_id not in self.sessions:
            now = datetime.now(pytz.timezone(TIMEZONE)).strftime("%A, %B %d, %I:%M %p")
            self.sessions[session_id] = [
                {"role": "system", "content": SYSTEM_PROMPT.format(now=now)}
            ]
        return self.sessions[session_id]

    def _trim(self, messages: list) -> None:
        """Drop the oldest turns (whole user turns, so tool calls stay paired)."""
        while len(messages) > self.max_history:
            next_user = next(
                (i for i, m in enumerate(messages[2:], start=2) if m["role"] == "user"),
                None,
            )
            if next_user is None:
                break
            del messages[1:next_user]

    def _complete(self, messages: list, allow_tools: bool):
        kwargs = {
            "model": LLM_ALIAS,
            "messages": messages,
            "temperature": self.temperature,
            "extra_body": NO_THINKING,
        }
        if allow_tools:
            kwargs["tools"] = TOOLS
        return self.client.chat.completions.create(**kwargs).choices[0].message

    # ------------------------------------------------------------------ turn

    def turn(self, request, response):
        messages = self._history(request.session_id, request.reset)
        if request.user_text.strip():
            messages.append({"role": "user", "content": request.user_text.strip()})
        self._trim(messages)

        calls, end, reply = [], False, ""
        try:
            for step in range(self.max_tool_calls + 1):
                message = self._complete(
                    messages, allow_tools=step < self.max_tool_calls
                )
                if not message.tool_calls:
                    reply = strip_thinking(message.content or "").strip()
                    break
                messages.append(
                    {
                        "role": "assistant",
                        "content": message.content or "",
                        "tool_calls": [tc.model_dump() for tc in message.tool_calls],
                    }
                )
                for tool_call in message.tool_calls:
                    name = tool_call.function.name
                    start = time.perf_counter()
                    try:
                        args = json.loads(tool_call.function.arguments or "{}")
                        result = self._run_tool(name, args, request.offline)
                    except (ApiError, json.JSONDecodeError, ValueError) as e:
                        args = tool_call.function.arguments
                        result = {"error": str(e)}
                    end = end or name == "end_conversation"
                    calls.append(
                        {
                            "name": name,
                            "args": args,
                            "ok": "error" not in result,
                            "source": result.get("_source"),
                            "latency_s": round(time.perf_counter() - start, 3),
                        }
                    )
                    messages.append(
                        {
                            "role": "tool",
                            "tool_call_id": tool_call.id,
                            "content": json.dumps(result, ensure_ascii=False),
                        }
                    )
        except Exception as e:  # LLM server down or malformed output
            self.get_logger().error(f"Agent turn failed: {e!r}")
            reply = (
                "Sorry, I had a problem thinking about that. Could you say it again?"
            )

        if not reply:
            reply = "Goodbye, good luck!" if end else "Sorry, could you repeat that?"
        messages.append({"role": "assistant", "content": reply})

        self.get_logger().info(
            f"[{request.session_id}] tools={[c['name'] for c in calls]} -> {reply}"
        )
        response.reply = reply
        response.end_conversation = end
        response.tool_calls_json = json.dumps(calls, ensure_ascii=False)
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(DialogueAgent())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
