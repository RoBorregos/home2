#!/usr/bin/env python3

"""
Experiment 3 (agent): FRIDA as a conversational agent. HRI's dialogue agent
(/hri/nlp/conversation) decides what to say and which schedule lookups to make
(LLM tool calling); this task manager only listens, forwards and speaks.
Ends with a touch survey focused on satisfaction.

ros2 run task_manager exp3_agent.py [--scripted-user] [--no-speaker] [--offline]
Needs the LLM with a larger context: LLAMA_CTX_SIZE=8192 ./run.sh hri --hric
"""

import json
import time
import uuid

import rclpy
from frida_constants.hri_constants import CONVESATION_SERVICE
from frida_interfaces.srv import AgentTurn
from task_manager.benchmarks.experiment_base import ExperimentTM, run_experiment
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status

MAX_TURNS = 10
MAX_SILENT_TURNS = 2
AGENT_TIMEOUT = 90.0
OPENING = "Hi! I'm Frida. I can help you check your interview and your competition rounds. How can I help you?"
REPROMPT = "Are you still there? You can ask me about your interview or your rounds."


class AgentDialogueTM(ExperimentTM):
    """hear -> agent turn -> say, until the agent ends the conversation"""

    EXPERIMENT = "exp3_agent"
    NEEDS_SCHEDULE = True  # only for hotwords; the agent does the lookups
    SCHEDULE_ONLINE = False

    def __init__(self, args):
        super().__init__(args)
        self.agent_client = self.create_client(AgentTurn, CONVESATION_SERVICE)
        if not self.agent_client.wait_for_service(timeout_sec=10.0):
            Logger.warn(self, f"{CONVESATION_SERVICE} not available yet")

    def _agent_turn(self, session_id: str, text: str, reset: bool = False):
        request = AgentTurn.Request(
            session_id=session_id,
            user_text=text,
            reset=reset,
            offline=self.args.offline,
        )
        future = self.agent_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=AGENT_TIMEOUT)
        return future.result()

    def dialogue(self) -> str:
        session_id = uuid.uuid4().hex[:8]
        self.metrics.note(agent_session=session_id)
        self.say(OPENING)

        silent = 0
        reset = True
        used_schedule = False
        for turn in range(1, MAX_TURNS + 1):
            with self.metrics.phase("hear"):
                status, text, _ = self.hri.hear(
                    hotwords=self.hotwords(),
                    initial_prompt="A candidate talks to a receptionist robot about their interview and rounds",
                    max_audio_length=15.0,
                )
            if status != Status.EXECUTION_SUCCESS or not text.strip():
                silent += 1
                if silent >= MAX_SILENT_TURNS:
                    self.say("I'll be here if you need me. Goodbye!")
                    self.metrics.note(turns=turn - 1)
                    return "no_input" if turn <= MAX_SILENT_TURNS else "user_left"
                self.say(REPROMPT)
                continue
            silent = 0

            start = time.perf_counter()
            with self.metrics.phase("agent"):
                result = self._agent_turn(session_id, text, reset=reset)
            reset = False
            latency = round(time.perf_counter() - start, 3)

            if result is None:
                Logger.error(self, "Agent did not answer")
                self.say("Sorry, I'm having trouble right now. Please ask the staff.")
                self.metrics.note(turns=turn)
                return "agent_error"

            calls = json.loads(result.tool_calls_json or "[]")
            for call in calls:
                self.metrics.count_source(call.get("source"))
            used_schedule |= any(
                c["name"] in ("get_candidate", "get_team") and c["ok"] for c in calls
            )
            self.metrics.append(
                "turns_log",
                {
                    "turn": turn,
                    "user": text,
                    "reply": result.reply,
                    "tool_calls": calls,
                    "agent_latency_s": latency,
                },
            )
            with self.metrics.phase("say_reply"):
                self.say(result.reply)

            if result.end_conversation:
                self.metrics.note(turns=turn, ended_by="agent")
                return "success" if used_schedule else "ended_without_lookup"

        self.say("I have to help other people now. Good luck in the competition!")
        self.metrics.note(turns=MAX_TURNS, ended_by="max_turns")
        return "max_turns"


def main(args=None):
    run_experiment(AgentDialogueTM, args)


if __name__ == "__main__":
    main()
