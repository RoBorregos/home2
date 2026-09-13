#!/usr/bin/env python3
"""
Offline tests for the LLM brief compiler.

No LLM is called: the completion function is injected, so these tests pin down the part
that actually matters — that the strict parser rejects bad drafts and that the rejection
is fed back for another attempt.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from compile_brief import (  # noqa: E402
    build_prompt,
    compile_brief,
    skill_catalogue,
    strip_fences,
)

from task_manager.planner.brief import BriefError  # noqa: E402
from task_manager.skills.registry import REGISTRY  # noqa: E402

SECTION = "Doing Laundry. Time limit 7:00. Folding a piece of clothing: 800 points."

GOOD = """
task: doing_laundry
budget_s: 420
risk_posture: safe
on_deadline:
  - move_arm_to(nav_pose)
objectives:
  - id: fold
    points: 800
    template:
      - go_to(kitchen, laundry_table)
      - place()
"""

INVENTED_SKILL = """
task: doing_laundry
budget_s: 420
on_deadline:
  - move_arm_to(nav_pose)
objectives:
  - id: fold
    points: 800
    template:
      - fold_laundry_neatly(shirt)
"""

NOT_YAML = "Here is your brief! It folds the laundry nicely."


def test_catalogue_lists_every_skill() -> None:
    catalogue = skill_catalogue()
    for name in REGISTRY:
        assert f"  {name}(" in catalogue, f"{name} missing from the prompt catalogue"
    print(f"✓ prompt catalogue lists all {len(REGISTRY)} skills")


def test_prompt_includes_section_and_rules() -> None:
    messages = build_prompt(SECTION, "doing_laundry")
    assert messages[0]["role"] == "system"
    joined = messages[1]["content"]
    assert SECTION.strip() in joined
    assert "Available skills:" in joined
    assert "doing_laundry" in joined
    print("✓ prompt carries the rulebook section and the skill catalogue")


def test_error_feedback_is_included_on_retry() -> None:
    messages = build_prompt(SECTION, "doing_laundry", previous_error="unknown skill 'teleport'")
    assert "rejected by the brief validator" in messages[1]["content"]
    assert "teleport" in messages[1]["content"]
    print("✓ a rejection is fed back into the next attempt")


def test_strip_fences() -> None:
    assert strip_fences("```yaml\ntask: x\n```").strip() == "task: x"
    assert strip_fences("task: x").strip() == "task: x"
    assert strip_fences("<think>hmm</think>\ntask: x").strip() == "task: x"
    print("✓ code fences and reasoning preambles are stripped")


def test_valid_draft_is_accepted_first_try() -> None:
    calls = []

    def complete(messages):
        calls.append(messages)
        return GOOD

    text, brief = compile_brief(SECTION, "doing_laundry", complete)
    assert brief.task == "doing_laundry"
    assert brief.objectives[0].points == 800
    assert len(calls) == 1, "a valid draft should not be retried"
    assert "fold" in text
    print("✓ a valid draft is accepted on the first attempt")


def test_invented_skill_is_rejected_then_corrected() -> None:
    """The failure mode that matters: the model makes up a primitive."""
    responses = [INVENTED_SKILL, GOOD]
    seen_errors = []

    def complete(messages):
        content = messages[1]["content"]
        if "rejected by the brief validator" in content:
            seen_errors.append(content)
        return responses.pop(0)

    _, brief = compile_brief(SECTION, "doing_laundry", complete)
    assert brief.task == "doing_laundry"
    assert len(seen_errors) == 1, "the retry should have carried the validator's error"
    assert "fold_laundry_neatly" in seen_errors[0], seen_errors[0]
    print("✓ an invented skill is caught and the error drives the retry")


def test_gives_up_after_max_attempts() -> None:
    def complete(messages):
        return INVENTED_SKILL

    try:
        compile_brief(SECTION, "doing_laundry", complete, attempts=2)
        raise AssertionError("should have given up")
    except BriefError as error:
        assert "no valid brief after 2 attempts" in str(error), error
    print("✓ the compiler gives up rather than emitting an invalid brief")


def test_non_yaml_is_rejected() -> None:
    def complete(messages):
        return NOT_YAML

    try:
        compile_brief(SECTION, "doing_laundry", complete, attempts=1)
        raise AssertionError("prose should not be accepted as a brief")
    except BriefError:
        pass
    print("✓ prose instead of YAML is rejected")


if __name__ == "__main__":
    test_catalogue_lists_every_skill()
    test_prompt_includes_section_and_rules()
    test_error_feedback_is_included_on_retry()
    test_strip_fences()
    test_valid_draft_is_accepted_first_try()
    test_invented_skill_is_rejected_then_corrected()
    test_gives_up_after_max_attempts()
    test_non_yaml_is_rejected()
    print("\nAll brief compiler tests passed.")
