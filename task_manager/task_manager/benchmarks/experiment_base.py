"""
Shared base for the HRI experiments (task_manager/scripts/benchmarks/exp*.py).

Every experiment follows the first part of HRIC:
    wait for "Frida" -> detect person + follow face -> dialogue (subclass) -> survey
and saves one JSON line per participant in task_manager/benchmarks/results/<experiment>/.
"""

import argparse
import difflib
import json
import os

import rclpy
from frida_constants.robot_api import make_schedule_source, normalize_name
from frida_constants.vision_constants import FOLLOW_TOPIC
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from task_manager.benchmarks.dialogue_metrics import (
    HRI_METHODS,
    MANIPULATION_METHODS,
    VISION_METHODS,
    DialogueMetrics,
)
from task_manager.benchmarks.survey import MODES as SURVEY_MODES
from task_manager.benchmarks.survey import SurveyRunner
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

BENCHMARK_DIR = os.getenv("BENCHMARK_DIR", "/workspace/src/task_manager/benchmarks")

WAKE_WORD = "frida"
WAKE_TIMEOUT = 60.0
DETECT_PERSON_TIMEOUT = 5.0


def experiment_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock-hri", action="store_true", help="Mock all HRI services")
    parser.add_argument(
        "--scripted-user",
        action="store_true",
        help="Real HRI services but the participant's words come from scripted/<exp>.json",
    )
    parser.add_argument(
        "--no-speaker",
        action="store_true",
        help="Skip the TTS service (it blocks without an audio output); lines are logged",
    )
    parser.add_argument(
        "--real-vision",
        action="store_true",
        help="Real vision + arm for detect person / follow face (moves the arm)",
    )
    parser.add_argument("--offline", action="store_true", help="Use only the schedule backup")
    parser.add_argument(
        "--script", default=None, help="Scripted sessions file (default scripted/<exp>.json)"
    )
    parser.add_argument("--sessions", type=int, default=0, help="0 = run until stopped")
    parser.add_argument("--label", default="run", help="Name of this experiment run")
    parser.add_argument("--survey", choices=SURVEY_MODES, default=None)
    parser.add_argument("--survey-timeout", type=float, default=180.0)
    parser.add_argument("--no-button", action="store_true", help="Do not wait for the start button")
    args = parser.parse_args(argv if argv is not None else remove_ros_args()[1:])
    args.scripted = args.mock_hri or args.scripted_user
    if args.survey is None:
        args.survey = "auto" if args.scripted else "display"
    return args


def best_match(text: str, options: list[str]) -> tuple[int, float]:
    """Index and similarity of the option closest to text (accent/sound-alike insensitive)."""
    target = " ".join(normalize_name(text))
    best_index, best_ratio = -1, 0.0
    for i, option in enumerate(options):
        candidate = " ".join(normalize_name(option))
        ratio = difflib.SequenceMatcher(None, target, candidate).ratio()
        if candidate and candidate in target:
            ratio = 1.0
        if ratio > best_ratio:
            best_index, best_ratio = i, ratio
    return best_index, best_ratio


ORDINALS = {
    "first": 0, "1": 0,
    "second": 1, "two": 1, "2": 1,
    "third": 2, "three": 2, "3": 2,
    "fourth": 3, "four": 3, "4": 3,
    "fifth": 4, "five": 4, "5": 4,
}  # fmt: skip


def pick_option(text: str, options: list[str], threshold: float = 0.5) -> int | None:
    """Index chosen by (partial) name, team, or number/ordinal ("the second one", "2")."""
    if not text or not options:
        return None
    words = set(normalize_name(text))
    option_words = [set(normalize_name(o)) - {"from"} for o in options]
    shared = set.intersection(*option_words) if len(options) > 1 else set()
    hits = [len((ow - shared) & words) for ow in option_words]
    best = max(hits)
    if best > 0 and hits.count(best) == 1:
        return hits.index(best)

    tokens = text.lower().replace(".", " ").replace(",", " ").split()
    for token in tokens:
        position = ORDINALS.get(token.strip("#"))
        if position is not None and position < len(options):
            return position
    if " ".join(tokens) in ("one", "number one", "option one", "the one"):
        return 0

    index, ratio = best_match(text, options)
    return index if ratio >= threshold else None


class ScriptedUser:
    """Replaces what the participant says: hear() and non-wake keyword spotting
    (which is what confirm() uses for yes/no) pop the next scripted utterance."""

    def __init__(self, hri, sessions: list[dict], mock_hri: bool):
        self.sessions = sessions
        self.queue = []
        self.survey_answers = None
        self.expect = None

        def interpret_keyword(keywords, *args, **kwargs):
            if WAKE_WORD in keywords:
                return Status.EXECUTION_SUCCESS, WAKE_WORD
            text = self._next() or ""
            for keyword in keywords:
                if keyword in text.lower().split() or text.lower().startswith(keyword):
                    return Status.EXECUTION_SUCCESS, keyword
            return Status.TARGET_NOT_FOUND, ""

        def hear(*args, **kwargs):
            text = self._next()
            if text is None:
                return Status.TARGET_NOT_FOUND, "", {}
            return Status.EXECUTION_SUCCESS, text, {w: 0.9 for w in text.split()}

        hri.interpret_keyword = interpret_keyword
        hri.hear = hear

        if mock_hri:
            # Mocked HRI never calls hear(), so the composite calls consume the script

            def confirm(question, *args, **kwargs):
                hri.say(question)
                status, keyword = interpret_keyword(["yes", "no"])
                return (status, keyword) if keyword else (Status.TIMEOUT, "")

            def ask_and_confirm(question, query, *args, options=None, **kwargs):
                hri.say(question)
                text = self._next()
                if text is None:
                    return Status.TIMEOUT, None
                if options:
                    index, _ = best_match(text, options)
                    return Status.EXECUTION_SUCCESS, options[index]
                return Status.EXECUTION_SUCCESS, text

            hri.confirm = confirm
            hri.ask_and_confirm = ask_and_confirm
            hri.extract_data = lambda query, text, *a, **k: (
                Status.EXECUTION_SUCCESS,
                text,
            )

    def _next(self):
        return self.queue.pop(0) if self.queue else None

    def load(self, index: int):
        session = self.sessions[index % len(self.sessions)] if self.sessions else {}
        self.queue = list(session.get("utterances", []))
        self.survey_answers = session.get("survey")
        self.expect = session.get("expect")


class ExperimentTM(Node):
    """Wake -> engage (follow face) -> dialogue() -> survey -> save. Subclasses
    implement dialogue() and return the session outcome."""

    EXPERIMENT = "experiment"
    NEEDS_SCHEDULE = False  # load names/teams (and the API) through ScheduleSource
    SCHEDULE_ONLINE = True  # False: only the local backup (e.g. just for hotwords)

    class States:
        WAITING_FOR_BUTTON = 0
        WAIT_WAKE = 1
        DIALOGUE = 2
        SURVEY = 3
        CLOSE = 4

    def __init__(self, args: argparse.Namespace):
        super().__init__(f"{self.EXPERIMENT}_task_manager")
        self.args = args

        mock_areas = ["navigation"]
        mock_areas += [] if args.real_vision else ["vision", "manipulation"]
        mock_areas += ["hri"] if args.mock_hri else []
        self.subtask_manager = SubtaskManager(self, task=Task.DEMO, mock_areas=mock_areas)
        self.hri = self.subtask_manager.hri

        self.schedule = (
            make_schedule_source(offline=args.offline or not self.SCHEDULE_ONLINE)
            if self.NEEDS_SCHEDULE
            else None
        )

        if args.no_speaker:
            self._install_silent_say()

        self.scripted_user = None
        if args.scripted:
            script_path = args.script or os.path.join(
                BENCHMARK_DIR, "scripted", f"{self.EXPERIMENT}.json"
            )
            with open(script_path, "r") as file:
                sessions = json.load(file)["sessions"]
            self.scripted_user = ScriptedUser(self.hri, sessions, args.mock_hri)
            if args.sessions == 0:
                args.sessions = len(sessions)

        self.metrics = DialogueMetrics(
            os.path.join(BENCHMARK_DIR, "results", self.EXPERIMENT),
            self.EXPERIMENT,
            args.label,
        )
        self.metrics.instrument(self.hri, HRI_METHODS)
        self.metrics.instrument(self.subtask_manager.vision, VISION_METHODS)
        self.metrics.instrument(self.subtask_manager.manipulation, MANIPULATION_METHODS)
        self.create_subscription(Point, FOLLOW_TOPIC, lambda _: self.metrics.face_seen(), 10)

        self.survey = SurveyRunner(
            self,
            os.path.join(BENCHMARK_DIR, "surveys", f"{self.EXPERIMENT}.json"),
            args.survey,
            args.survey_timeout,
        )

        self.current_state = (
            self.States.WAIT_WAKE
            if args.no_button or args.scripted
            else self.States.WAITING_FOR_BUTTON
        )
        self.running_task = True
        self.sessions_done = 0
        self.announce_wake = True
        self.vision_ready = False
        self.outcome = None

        Logger.info(self, f"{self.EXPERIMENT} started. Results -> {self.metrics.out_path}")
        if self.schedule is not None:
            online = "online + backup" if self.schedule.online else "backup only"
            Logger.info(self, f"Schedule source: {online}")

    # ----------------------------------------------------------------- helpers

    def _install_silent_say(self):
        def say(text, wait=True, speed=1.15):
            Logger.info(self, f"[no-speaker] ROBOT: {text}")
            return Status.EXECUTION_SUCCESS

        self.hri.say = say

    def say(self, text: str, wait: bool = True):
        self.metrics.transcript("robot", text)
        self.hri.say(text, wait=wait)

    def schedule_call(self, method: str, *args, **kwargs):
        """Call the schedule source and record whether online or backup answered."""
        try:
            return getattr(self.schedule, method)(*args, **kwargs)
        finally:
            self.metrics.count_source(self.schedule.last_source)

    def hotwords(self, extra: list[str] = ()) -> str:
        """Names and teams from the schedule backup, to help the transcription."""
        words = list(extra)
        if self.schedule is not None:
            words += self.schedule.team_names() + self.schedule.candidate_names()
        return ", ".join(dict.fromkeys(w for w in words if w))

    def engage_person(self):
        """First part of HRIC: detect the person, then follow their face with the arm."""
        vision = self.subtask_manager.vision
        with self.metrics.phase("vision_engage"):
            status = vision.detect_person(timeout=DETECT_PERSON_TIMEOUT)
            self.metrics.note(person_detected=status == Status.EXECUTION_SUCCESS)
            vision.activate_face_recognition()
            vision.follow_by_name("area")
            self.subtask_manager.manipulation.follow_face(True)
        self.metrics.start_face_window()

    def disengage_person(self):
        with self.metrics.phase("vision_disengage"):
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.vision.deactivate_face_recognition()

    def dialogue(self) -> str:
        """Run the experiment's conversation. Returns the session outcome."""
        raise NotImplementedError

    # ------------------------------------------------------------ state machine

    def run(self):
        s = self.States

        if self.current_state == s.WAITING_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.hri.reset_task_status()
            self.hri.say("Press start on my screen to begin the experiment.")
            while not self.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.current_state = s.WAIT_WAKE

        elif self.current_state == s.WAIT_WAKE:
            if not self.vision_ready:
                self.subtask_manager.vision.deactivate_face_recognition()
                self.subtask_manager.manipulation.move_to_position("front_stare")
                self.vision_ready = True
            if self.announce_wake:
                Logger.state(self, "Waiting for wake word")
                self.hri.say("Say Frida whenever you want to talk to me.")
                self.announce_wake = False
            status, _ = self.hri.interpret_keyword([WAKE_WORD], timeout=WAKE_TIMEOUT)
            if status != Status.EXECUTION_SUCCESS:
                return
            if self.scripted_user:
                self.scripted_user.load(self.sessions_done)
            self.metrics.start_session(scripted=self.scripted_user is not None)
            if self.scripted_user and self.scripted_user.expect:
                self.metrics.note(expect=self.scripted_user.expect)
            self.engage_person()
            self.current_state = s.DIALOGUE

        elif self.current_state == s.DIALOGUE:
            Logger.state(self, "Dialogue")
            with self.metrics.phase("dialogue"):
                try:
                    self.outcome = self.dialogue()
                except Exception as e:  # keep the session data even if the dialogue fails
                    Logger.error(self, f"Dialogue failed: {e!r}")
                    self.metrics.note(error=repr(e))
                    self.outcome = "error"
            self.current_state = s.SURVEY

        elif self.current_state == s.SURVEY:
            Logger.state(self, "Survey")
            if self.args.survey != "skip":
                self.say(self.survey.intro, wait=False)
            with self.metrics.phase("survey"):
                scripted = self.scripted_user.survey_answers if self.scripted_user else None
                self.metrics.note(survey=self.survey.run(scripted))
            if self.args.survey != "skip":
                self.say("Thank you for participating!")
            self.current_state = s.CLOSE

        elif self.current_state == s.CLOSE:
            self.disengage_person()
            session = self.metrics.end_session(self.outcome)
            Logger.success(
                self,
                f"Session {session.get('session')} -> {self.outcome} "
                f"in {session.get('duration_s')}s",
            )
            self.sessions_done += 1
            self.announce_wake = True
            self.current_state = s.WAIT_WAKE
            if self.args.sessions and self.sessions_done >= self.args.sessions:
                self.running_task = False


def run_experiment(experiment_cls, args=None):
    """Standard main loop shared by the experiment scripts."""
    rclpy.init(args=args)
    node = experiment_cls(experiment_args())
    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node.args.real_vision:
            node.subtask_manager.manipulation.follow_face(False)
        if node.metrics.active:
            node.metrics.end_session("aborted")
        Logger.info(node, f"Results saved in {node.metrics.out_path}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
