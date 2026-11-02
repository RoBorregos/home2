#!/usr/bin/env python3

"""Mock test of the HRITasks.ask_and_confirm decision flow (#1384).

ROS packages that are not installed (rclpy, frida_interfaces, baml...) are replaced by MagicMock
modules, so it also runs outside the containers:

    python -m pytest task_manager/test/test_ask_and_confirm_flow.py
"""

import importlib.abc
import importlib.machinery
import json
import os
import sys
from unittest.mock import MagicMock

import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path[:0] = [os.path.join(REPO, "task_manager"), os.path.join(REPO, "frida_constants")]

STUBBED = (
    "rclpy",
    "ament_index_python",
    "frida_interfaces",
    "std_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "baml_py",
    "task_manager.utils.baml_client",
)


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, name, path, target=None):
        if not any(name == s or name.startswith(s + ".") for s in STUBBED):
            return None
        if importlib.machinery.PathFinder.find_spec(name.split(".")[0]) and "." not in name:
            return None  # real package installed
        return importlib.machinery.ModuleSpec(name, self, is_package=True)

    def create_module(self, spec):
        module = MagicMock(name=spec.name)
        module.__path__ = []
        module.__spec__ = spec
        return module

    def exec_module(self, module):
        pass


sys.meta_path.insert(0, _StubFinder())

from task_manager.subtask_managers import hri_tasks  # noqa: E402
from task_manager.utils import confidence_policy as cp  # noqa: E402
from task_manager.utils.status import Status  # noqa: E402


class _Time:
    def __sub__(self, other):
        return MagicMock(nanoseconds=10**12)


def make_hri(tmp_path, utterances, extracted, confirm_answers=("yes",) * 10):
    """utterances: list of (transcription, words, confidences); extracted: values returned by extract_data."""
    hri = hri_tasks.HRITasks.__new__(hri_tasks.HRITasks)
    hri.node = MagicMock()
    hri.node.get_clock.return_value.now.return_value = _Time()
    hri.mock_data = False
    hri.task_name = "hric"
    hri.decision_logger = cp.DecisionLogger("hric", root=tmp_path)
    hri.confidence_model = cp.CalibratedModel(
        {"type": "isotonic", "feature": "conf_min", "x": [0.0, 1.0], "y": [0.0, 1.0]}
    )
    hri.last_stt_words, hri.last_stt_confidences, hri.last_stt_meta = [], [], {}
    hri.said = []
    hri.confirms = 0
    utterances, extracted, answers = list(utterances), list(extracted), list(confirm_answers)

    def hear(**kwargs):
        transcription, words, confidences = utterances.pop(0)
        hri.last_stt_words, hri.last_stt_confidences = words, confidences
        return Status.EXECUTION_SUCCESS, transcription, dict(zip(words, confidences))

    def confirm(*args, **kwargs):
        hri.confirms += 1
        return Status.EXECUTION_SUCCESS, answers.pop(0)

    hri.say = lambda text, *a, **k: hri.said.append(text)
    hri.hear = hear
    hri.extract_data = lambda *a, **k: (Status.EXECUTION_SUCCESS, extracted.pop(0))
    hri.confirm = confirm
    hri.find_closest = MagicMock()
    return hri


def records(hri):
    return [json.loads(line) for line in hri.decision_logger.path.read_text().splitlines()]


COKE = ("I want a coke", ["I", "want", "a", "coke"], [0.95, 0.95, 0.95, 0.95])
COKE_BAD = ("I want a coke", ["I", "want", "a", "coke"], [0.95, 0.95, 0.95, 0.1])


@pytest.fixture
def policy(monkeypatch):
    def set_policy(name, fix=True):
        monkeypatch.setattr(hri_tasks, "CONFIDENCE_POLICY", name)
        monkeypatch.setattr(hri_tasks, "CONFIDENCE_FIX_MATCHING", fix)

    return set_policy


def ask(hri, **kwargs):
    return hri.ask_and_confirm(
        "What is your favorite drink?", "drink", min_wait_between_retries=0, **kwargs
    )


def test_normalized_value_was_always_confirmed(tmp_path, policy):
    policy("legacy", fix=False)
    hri = make_hri(tmp_path, [COKE], ["Coca-Cola"])
    assert ask(hri) == (Status.EXECUTION_SUCCESS, "Coca-Cola")
    assert hri.confirms == 1  # old behavior: "coke" -> "Coca-Cola" has confidence 0


def test_normalized_value_accepted_after_fix(tmp_path, policy):
    policy("legacy")
    hri = make_hri(tmp_path, [COKE], ["Coca-Cola"])
    assert ask(hri) == (Status.EXECUTION_SUCCESS, "Coca-Cola")
    assert hri.confirms == 0
    (rec,) = records(hri)
    assert rec["decision"] == cp.ACCEPT
    assert rec["features"]["match_type"] == "whole"
    assert rec["features"]["legacy_conf"] == 0


def test_always_confirm_overrides_policy(tmp_path, policy):
    policy("legacy")
    hri = make_hri(tmp_path, [COKE], ["Coca-Cola"])
    ask(hri, always_confirm=True)
    assert hri.confirms == 1
    assert records(hri)[0]["confirm_answer"] == "yes"


def test_three_way_repeats_then_accepts(tmp_path, policy):
    policy("three_way")
    hri = make_hri(tmp_path, [COKE_BAD, COKE], ["coke", "coke"])
    assert ask(hri) == (Status.EXECUTION_SUCCESS, "coke")
    # The question is not asked again after "could you repeat?"
    assert hri.said == ["What is your favorite drink?", hri_tasks.ASK_TO_REPEAT_TEXT]
    assert hri.confirms == 0
    assert [r["decision"] for r in records(hri)] == [cp.REPEAT, cp.ACCEPT]
    assert [r["attempt_idx"] for r in records(hri)] == [1, 2]


def test_three_way_confirms_on_last_attempt(tmp_path, policy):
    policy("three_way")
    hri = make_hri(tmp_path, [COKE_BAD], ["coke"])
    assert ask(hri, retries=1) == (Status.EXECUTION_SUCCESS, "coke")
    assert hri.confirms == 1


def test_three_way_confirms_in_the_middle(tmp_path, policy):
    policy("three_way")
    mid = ("I want a coke", ["I", "want", "a", "coke"], [0.95, 0.95, 0.95, 0.6])
    hri = make_hri(tmp_path, [mid], ["coke"])
    ask(hri)
    assert hri.confirms == 1
    assert records(hri)[0]["p_correct"] == pytest.approx(0.6)


def test_options_exact_match_accepts_and_closest_low_similarity_confirms(tmp_path, policy):
    policy("legacy")
    hri = make_hri(
        tmp_path,
        [("the kitchen", ["the", "kitchen"], [0.9, 0.9]), ("hmm", ["hmm"], [0.95])],
        [],
    )
    hri.find_closest.return_value = (
        Status.EXECUTION_SUCCESS,
        MagicMock(results=["bedroom"], similarities=[0.4]),
    )
    assert ask(hri, options=["kitchen", "bedroom"], skip_extract_data=True)[1] == "kitchen"
    assert hri.confirms == 0
    assert ask(hri, options=["kitchen", "bedroom"], skip_extract_data=True)[1] == "bedroom"
    assert hri.confirms == 1
    assert [r["features"]["exact_match"] for r in records(hri)] == [True, False]


def test_exact_match_with_always_confirm_asks_on_every_attempt(tmp_path, policy):
    policy("legacy")
    kitchen = ("the kitchen", ["the", "kitchen"], [0.9, 0.9])
    hri = make_hri(tmp_path, [kitchen, kitchen], [], confirm_answers=["no", "yes"])
    assert (
        ask(hri, options=["kitchen"], skip_extract_data=True, always_confirm=True)[1] == "kitchen"
    )
    assert hri.confirms == 2


def test_rejected_confirmation_retries(tmp_path, policy):
    policy("legacy")
    hri = make_hri(tmp_path, [COKE_BAD, COKE_BAD], ["coke", "coke"], confirm_answers=["no", "yes"])
    assert ask(hri) == (Status.EXECUTION_SUCCESS, "coke")
    assert [r["confirm_answer"] for r in records(hri)] == ["no", "yes"]
