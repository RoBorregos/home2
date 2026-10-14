#!/usr/bin/env python3
"""Unit tests for the agentic executor/supervisors (no ROS, no LLM server).

Run: python -m pytest task_manager/scripts/test/test_agentic_executor.py
"""

import json
import os
import sys
import urllib.error

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from task_manager.agentic.events import SkillResult  # noqa: E402
from task_manager.agentic.executor import SimClock, TaskExecutor  # noqa: E402
from task_manager.agentic.sim_skills import make_sim_skills, sample_world  # noqa: E402
from task_manager.agentic.spec import load_spec, spec_from_dict  # noqa: E402
from task_manager.agentic.supervisor import (
    ErrorAwareRulesSupervisor,
    LLMSupervisor,
    RulesSupervisor,
)  # noqa: E402

SPEC_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "config", "agentic")


def flaky(fail_times, error="boom"):
    calls = {"n": 0}

    def skill(**_):
        calls["n"] += 1
        if calls["n"] <= fail_times:
            return SkillResult.fail(error)
        return SkillResult.ok()

    skill.calls = calls
    return skill


def run(spec_dict, skills, supervisor):
    clock = SimClock()
    return TaskExecutor(
        spec_from_dict(spec_dict), skills, supervisor, clock=clock, log=lambda _: None
    ).run()


ONE_STEP = {
    "name": "t",
    "steps": [
        {"id": "a", "skill": "a", "points": 10, "max_attempts": 3, "fallbacks": ["help"]},
        {"id": "b", "skill": "b", "points": 5},
    ],
}


def test_rules_retry_then_success():
    a = flaky(2)
    res = run(ONE_STEP, {"a": a, "b": flaky(0), "help": flaky(0)}, RulesSupervisor())
    assert a.calls["n"] == 3
    assert res.outcomes == {"a": "success", "b": "success"}
    assert res.points == 15


def test_rules_fallback_after_max_attempts():
    res = run(ONE_STEP, {"a": flaky(99), "b": flaky(0), "help": flaky(0)}, RulesSupervisor())
    assert res.outcomes["a"] == "fallback"
    assert res.points == 5 + 5  # half of a + b
    assert res.fallbacks_run == 1


def test_rules_skip_when_everything_fails():
    res = run(ONE_STEP, {"a": flaky(99), "b": flaky(0), "help": flaky(99)}, RulesSupervisor())
    assert res.outcomes["a"] == "skipped"
    assert res.outcomes["b"] == "success"


def test_critical_skip_aborts():
    spec = {
        "name": "t",
        "steps": [
            {"id": "a", "skill": "a", "critical": True, "max_attempts": 1},
            {"id": "b", "skill": "b"},
        ],
    }
    res = run(spec, {"a": flaky(99), "b": flaky(0)}, RulesSupervisor())
    assert res.aborted and "b" not in res.outcomes


def test_skill_exception_is_a_failed_attempt():
    def boom(**_):
        raise RuntimeError("service down")

    res = run(ONE_STEP, {"a": boom, "b": flaky(0), "help": flaky(0)}, RulesSupervisor())
    assert res.outcomes["a"] == "fallback"
    assert "exception: service down" in res.events[0]["error"]


def test_foreach_uses_blackboard_from_previous_step():
    spec = {
        "name": "t",
        "steps": [
            {"id": "order", "skill": "order"},
            {
                "foreach": "orders",
                "as": "item",
                "steps": [
                    {"id": "pick", "skill": "pick", "args": {"object": "{item}"}, "points": 1}
                ],
            },
        ],
    }
    picked = []

    def pick(object):
        picked.append(object)
        return SkillResult.ok()

    res = run(
        spec,
        {"order": lambda: SkillResult.ok(orders=["coke", "chips"]), "pick": pick},
        RulesSupervisor(),
    )
    assert picked == ["coke", "chips"]
    assert res.max_points == 2 and res.points == 2
    assert "orders[1].pick" in res.outcomes


def test_time_limit_stops_task():
    clock = SimClock()

    def slow(**_):
        clock.advance(100)
        return SkillResult.fail("slow")

    spec = spec_from_dict(
        {
            "name": "t",
            "time_limit_s": 150,
            "steps": [{"id": "a", "skill": "a", "max_attempts": 10}, {"id": "b", "skill": "b"}],
        }
    )
    res = TaskExecutor(
        spec, {"a": slow, "b": flaky(0)}, RulesSupervisor(), clock=clock, log=lambda _: None
    ).run()
    assert res.timed_out and "b" not in res.outcomes


def test_rules_plus_skips_retries_on_permanent_error():
    a = flaky(99, "object out of arm workspace")
    res = run(
        ONE_STEP,
        {"a": a, "b": flaky(0), "help": flaky(0)},
        ErrorAwareRulesSupervisor(["out of arm workspace"]),
    )
    assert a.calls["n"] == 1 and res.outcomes["a"] == "fallback"


# ── LLM supervisor (transport mocked) ──


def llm_with(answer=None, exc=None):
    sup = LLMSupervisor(base_url="http://fake", model="m")

    def chat(_messages):
        if exc:
            raise exc
        return answer

    sup._chat = chat
    return sup


def test_llm_valid_decision_and_speech():
    sup = llm_with(
        json.dumps({"action": "fallback", "say": "Could you help me?", "reason": "perm"})
    )
    res = run(
        ONE_STEP,
        {
            "a": flaky(99, "out of arm workspace"),
            "b": flaky(0),
            "help": flaky(0),
            "say": lambda text: SkillResult.ok(),
        },
        sup,
    )
    assert res.outcomes["a"] == "fallback"
    assert res.fallbacks_run == 1 and res.attempts == 3  # 1 try + help + b
    assert res.utterances == ["Could you help me?"]
    assert res.invalid_decisions == 0


def test_llm_thinking_block_is_stripped():
    sup = llm_with('<think>hmm</think>{"action": "skip", "say": "", "reason": "x"}')
    res = run(ONE_STEP, {"a": flaky(99), "b": flaky(0), "help": flaky(0)}, sup)
    assert res.outcomes["a"] == "skipped"


@pytest.mark.parametrize(
    "answer,exc",
    [
        ("not json", None),
        ('{"action": "dance", "say": "", "reason": ""}', None),
        (None, urllib.error.URLError("refused")),
        (None, TimeoutError("timed out")),
    ],
)
def test_llm_invalid_or_unreachable_falls_back_to_rules(answer, exc):
    a = flaky(2)
    res = run(ONE_STEP, {"a": a, "b": flaky(0), "help": flaky(0)}, llm_with(answer, exc))
    assert a.calls["n"] == 3  # rules: retry, retry, success
    assert res.outcomes["a"] == "success"
    assert res.invalid_decisions == 2
    assert all(d.source == "rules(llm-fallback)" for d in res.decisions)


def test_llm_cannot_fallback_without_fallbacks():
    spec = {"name": "t", "steps": [{"id": "a", "skill": "a", "max_attempts": 1}]}
    sup = llm_with('{"action": "fallback", "say": "", "reason": ""}')
    res = run(spec, {"a": flaky(99)}, sup)
    assert res.outcomes["a"] == "skipped" and res.invalid_decisions == 1


@pytest.mark.parametrize("factor,expected_calls", [(1.0, 2), (2.0, 4)])
def test_llm_retry_cap(factor, expected_calls):
    spec = {"name": "t", "steps": [{"id": "a", "skill": "a", "max_attempts": 2}]}
    a = flaky(99)
    sup = llm_with('{"action": "retry", "say": "", "reason": ""}')
    sup.retry_cap_factor = factor
    res = run(spec, {"a": a}, sup)
    assert a.calls["n"] == expected_calls
    assert res.outcomes["a"] == "skipped"


def test_llm_cannot_skip_unskippable_step():
    spec = {"name": "t", "steps": [{"id": "a", "skill": "a", "skippable": False}]}
    a = flaky(1)
    res = run(spec, {"a": a}, llm_with('{"action": "skip", "say": "", "reason": ""}'))
    assert res.outcomes["a"] == "success"  # rules took over and retried
    assert res.invalid_decisions == 1


def test_specs_load_and_sim_runs():
    for name in ("restaurant_lite.yaml", "safety.yaml"):
        load_spec(os.path.join(SPEC_DIR, name))
    spec = load_spec(os.path.join(SPEC_DIR, "restaurant_lite.yaml"))
    clock = SimClock()
    res = TaskExecutor(
        spec,
        make_sim_skills(sample_world(1), clock),
        RulesSupervisor(),
        clock=clock,
        log=lambda _: None,
    ).run()
    assert res.points > 0 and res.max_points >= res.points
