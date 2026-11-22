#!/usr/bin/env python3
"""Finals routine: patrol the house, report what is out of place, guide people to objects.

The robot stops at every tagged piece of furniture and records what it sees there
(task_manager/utils/scene_memory.py), then says what is misplaced and answers spoken
commands about what it has seen. It never grasps anything.

A failure anywhere degrades: a viewpoint it cannot reach is skipped, and running out
of time goes straight to the report with whatever it has seen.
"""

import json
import os
import time
from dataclasses import replace
from enum import Enum

import rclpy
from ament_index_python.packages import get_package_share_directory
from frida_constants.hri_constants import FINALS_HOTWORDS
from frida_constants.vision_constants import DETECTIONS_IMAGE_TOPIC
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from task_manager.utils.logger import Logger
from task_manager.utils.scene_memory import (
    Query,
    SceneMemory,
    SeenObject,
    fuse,
    parse_prompt,
    parse_query,
)
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

ARM_TRAVEL_POSE = "nav_pose"
# Words that end the command phase before the time limit
STOP_WORDS = ("stop", "finish", "that's all", "that is all", "terminar")
# Keys of an area in areas.json that are not furniture
NOT_FURNITURE = {"polygon", "safe_place", "description"}
# Minimum HRI embedding similarity to replace a spoken place with a map name
SNAP_SIMILARITY = 0.5


class ExecutionStates(Enum):
    WAIT_START = 0
    PATROL = 10
    REPORT = 20
    COMMANDS = 30
    END = 40


class FinalsTaskManager(Node):
    def __init__(self):
        super().__init__("finals_task_manager")
        self.patrol_time_limit_s = self._param("patrol_time_limit_s", 300.0)
        self.total_time_limit_s = self._param("total_time_limit_s", 600.0)
        self.patrol_mode = self._param("patrol_mode", "full")
        self.nav_retries = self._param("nav_retries", 2)
        self.approach_standoff_m = self._param("approach_standoff_m", 0.65)
        # Each round asks up to listen_retries times, waiting listen_start_s for speech
        self.max_silent_rounds = self._param("max_silent_rounds", 2)
        self.listen_retries = self._param("listen_retries", 3)
        self.listen_start_s = self._param("listen_start_s", 8.0)
        # Detections an object needs, sampled every sample_period_s while the robot dwells
        self.confirm_min_observations = self._param("confirm_min_observations", 3)
        self.sample_period_s = self._param("sample_period_s", 0.5)
        # JSON {"category": ["area", "sublocation"]} for this arena; "" uses objects.json
        default_location_json = self._param("default_location_json", "")
        mock_areas = self._param("mock_areas", [""])

        self.subtask_manager = SubtaskManager(
            self, task=Task.FINALS, mock_areas=[area for area in mock_areas if area]
        )
        objects_file = os.path.join(
            get_package_share_directory("frida_constants"), "data/objects.json"
        )
        with open(objects_file) as file:
            objects = json.load(file)
        self.object_to_category = objects["object_to_category"]
        self.memory = SceneMemory(
            json.loads(default_location_json)
            if default_location_json
            else objects["default_location"]
        )
        self.areas = {}

        self.state = ExecutionStates.WAIT_START
        self.started_at = 0.0
        Logger.info(self, f"Finals task manager ready, mocking: {mock_areas}")

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    # ------------------------------------------------------------------ helpers

    def _elapsed(self) -> float:
        return time.monotonic() - self.started_at

    def _wait(self, seconds: float) -> None:
        """Wait without blocking the node's callbacks."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    @staticmethod
    def _speakable(name: str) -> str:
        return name.replace("_", " ")

    def _step(self, text: str) -> None:
        """Show what the robot is doing on the display; the run goes on without it.

        Sent as a display answer: the default view, which Finals uses, lists answers
        in its message panel but has no task-step bar.
        """
        try:
            self.subtask_manager.hri.send_display_answer(text)
        except Exception as error:
            Logger.warn(self, f"Display step not shown: {error}")

    def _capture(self, label: str, data: dict) -> None:
        """Save the camera's current view in the display's evidence gallery."""
        try:
            self.subtask_manager.hri.publish_display_capture(label, DETECTIONS_IMAGE_TOPIC, data)
        except Exception as error:
            Logger.warn(self, f"Display capture not saved: {error}")

    @staticmethod
    def _where(obj) -> str:
        """Short place for the display: sublocation (area)."""
        return f"{obj.sublocation or '?'} ({obj.area or 'unknown room'})"

    def _place(self, area: str, sublocation: str) -> str:
        """How to say where something is. Empty when the map does not know."""
        if area and sublocation:
            return f"the {self._speakable(sublocation)} in the {self._speakable(area)}"
        return f"the {self._speakable(area)}" if area else ""

    # ------------------------------------------------------------------- patrol

    def patrol(self, mode: str, time_limit_s: float, max_viewpoints: int = 0) -> int:
        """Stand at each viewpoint long enough for vision to see it.

        Nav only plans the route; this drives it. Returns how many it reached.
        """
        status, route = self.subtask_manager.nav.plan_patrol(
            mode=mode, max_viewpoints=max_viewpoints
        )
        if status != Status.EXECUTION_SUCCESS or not route:
            Logger.error(self, f"No patrol route (mode '{mode}')")
            return 0

        # Live camera detections on the display while the robot looks around
        try:
            self.subtask_manager.hri.publish_display_topic(DETECTIONS_IMAGE_TOPIC)
        except Exception as error:
            Logger.warn(self, f"Display feed not switched: {error}")

        started = time.monotonic()
        reached = 0
        arm_pose = ""
        for index, step in enumerate(route, start=1):
            self._step(f"Patrolling {index}/{len(route)}: {step['area']}/{step['sublocation']}")
            if time.monotonic() - started > time_limit_s:
                Logger.warn(self, f"Patrol out of time after {reached}/{len(route)} viewpoints")
                break
            # Say it too: the display alone leaves the robot silent while it drives.
            place = self._place(step["area"], step["sublocation"])
            self.subtask_manager.hri.say(f"Now I am going to check {place}.", wait=False)
            # Never drive with the arm out in a looking pose.
            if arm_pose not in ("", ARM_TRAVEL_POSE):
                self.subtask_manager.manipulation.move_to_position(ARM_TRAVEL_POSE)
            if not self._drive_to(step):
                arm_pose = ARM_TRAVEL_POSE
                continue
            self.subtask_manager.manipulation.move_to_position(step["arm_pose"])
            arm_pose = step["arm_pose"]
            self._look(step)
            reached += 1
        self.subtask_manager.manipulation.move_to_position(ARM_TRAVEL_POSE)
        Logger.info(self, f"Patrol done: {reached}/{len(route)} viewpoints")
        self._step(f"Patrol done: {reached}/{len(route)} places checked")
        return reached

    def _drive_to(self, step: dict) -> bool:
        name = f"{step['area']}/{step['sublocation']}"
        for attempt in range(1 + self.nav_retries):
            status, error = self.subtask_manager.nav.move_to_pose(step["pose"])
            if status == Status.EXECUTION_SUCCESS:
                return True
            Logger.warn(self, f"Could not reach {name} ({error}), attempt {attempt + 1}")
        Logger.error(self, f"Skipping {name}")
        place = self._place(step["area"], step["sublocation"])
        self.subtask_manager.hri.say(f"I cannot get to {place}. I will skip it.", wait=False)
        return False

    def _look(self, step: dict) -> None:
        """Watch a viewpoint for its dwell and record the objects detected often enough."""
        samples, last_stamp, looked = [], None, False
        deadline = time.monotonic() + step["dwell_s"]
        while time.monotonic() < deadline:
            status, boxes = self.subtask_manager.vision.detect_objects()
            if status in (Status.EXECUTION_SUCCESS, Status.TARGET_NOT_FOUND):
                looked = True
                stamp = boxes[0].point3d.header.stamp if boxes else None
                if stamp is None or stamp != last_stamp:
                    samples.append(self._to_map(boxes))
                    last_stamp = stamp
            self._wait(self.sample_period_s)
        if not looked:
            return
        seen = [self._locate(obj) for obj in fuse(samples, self.confirm_min_observations)]
        self.memory.record(f"{step['area']}/{step['sublocation']}", [o for o in seen if o])

    def _to_map(self, boxes: list) -> list[SeenObject]:
        """Detections with depth as map-frame objects, with crop vectors for the newest frame."""
        boxes = [box for box in boxes if box.distance > 0.0]
        # Gallery detections come from older frames, which embed_crops cannot crop
        stamp = boxes[0].point3d.header.stamp if boxes else None
        current = [box for box in boxes if box.point3d.header.stamp == stamp]
        _, vectors = self.subtask_manager.vision.embed_crops(current)
        crops = dict(zip(map(id, current), vectors)) if len(vectors) == len(current) else {}
        objects = []
        for box in boxes:
            point = self.subtask_manager.nav.to_map_point(box.point3d)
            if point is not None:
                p = point.point
                embedding = crops.get(id(box))
                objects.append(SeenObject(box.classname, p.x, p.y, p.z, embedding=embedding))
        return objects

    def _locate(self, obj: SeenObject) -> SeenObject | None:
        """The object with its category and the room and furniture it is on."""
        point = PointStamped()
        point.header.frame_id = "map"
        point.point.x, point.point.y, point.point.z = obj.x, obj.y, obj.z
        status, place = self.subtask_manager.nav.get_area_for_point(point)
        if status != Status.EXECUTION_SUCCESS:
            return None
        return replace(
            obj,
            category=self.object_to_category.get(obj.label.strip().lower(), ""),
            area=place["area"],
            sublocation=(place["sublocations"] or [""])[0],
        )

    def _has_furniture(self, area: str, sublocation: str) -> bool:
        return sublocation in self.areas.get(area, {})

    # ------------------------------------------------------------------- report

    def report_misplaced(self) -> int:
        """Say every object that is not where it belongs. Returns how many."""
        self._step("Checking for objects out of place")
        misplaced = self.memory.misplaced(self._has_furniture)
        if not misplaced:
            self._step("Everything is in its place")
            self.subtask_manager.hri.say("Everything I saw is in its place.")
            return 0
        self.subtask_manager.hri.say(f"I found {len(misplaced)} things out of place.")
        for obj, expected_area, expected_sublocation in misplaced:
            self._step(
                f"Out of place: {obj.label} -> {self._where(obj)}, "
                f"should be: {expected_sublocation} ({expected_area})"
            )
            label = self._speakable(obj.label)
            found = self._place(obj.area, obj.sublocation)
            expected = self._place(expected_area, expected_sublocation)
            self.subtask_manager.hri.say(f"The {label} is on {found}. It belongs on {expected}.")
        return len(misplaced)

    # ----------------------------------------------------------------- commands

    def handle_command(self, text: str) -> None:
        """Answer one spoken command, and guide to the object when there is one."""
        self._step(f"Looking for: {text}")
        query = self._parse(text)
        matches = self._select(query)
        if not matches and query.target:
            self._step(f"Not seen yet: {text}. Looking again")
            self.subtask_manager.hri.say("I have not seen that. I will take another look.")
            self.patrol("revisit", self.patrol_time_limit_s, max_viewpoints=3)
            matches = self._select(query)
        if not query.target:
            self._say_list(matches)
            return
        if not matches:
            self._step(f"Not found: {text}")
            self.subtask_manager.hri.say("Sorry, I could not find it anywhere.")
            return
        self._guide_to(matches[0])

    def _parse(self, text: str) -> Query:
        """Split a command with the LLM; the whole text is the target if that fails.

        The LLM gets this map's rooms and furniture, and the names it answers are
        snapped to the real ones with HRI embeddings (as GPSR does), so a made-up
        place like "the sofa" still lands on the map's "couch".
        """
        rooms = [
            n for n, data in self.areas.items() if isinstance(data, dict) and data.get("polygon")
        ]
        furniture = sorted(
            {
                name
                for data in self.areas.values()
                if isinstance(data, dict)
                for name in data
                if name not in NOT_FURNITURE
            }
        )
        status, reply = self.subtask_manager.hri.answer_with_context(
            parse_prompt(text, rooms, furniture), ""
        )
        if status != Status.EXECUTION_SUCCESS:
            return Query(target=text)
        query = parse_query(text, reply, rooms, furniture, snap=self._snap)
        Logger.info(self, f"Command '{text}' -> {query}")
        return query

    def _snap(self, name: str, choices: list[str]) -> str:
        """The map name closest to `name` by HRI embeddings, or "" if none is close."""
        if name in choices:
            return name
        spoken = [self._speakable(choice) for choice in choices]
        status, closest = self.subtask_manager.hri.find_closest(
            spoken, self._speakable(name), threshold=SNAP_SIMILARITY
        )
        if status != Status.EXECUTION_SUCCESS or not closest.results:
            return ""
        return choices[spoken.index(closest.results[0])]

    def _select(self, query: Query) -> list[SeenObject]:
        text_embedding = None
        if query.target:
            _, text_embedding = self.subtask_manager.vision.embed_text(query.target)
        return self.memory.select(query, text_embedding)

    def _say_list(self, objects: list) -> None:
        if not objects:
            self._step("Nothing seen there")
            self.subtask_manager.hri.say("I did not see anything there.")
            return
        self._step("Seen: " + ", ".join(f"{obj.label} -> {self._where(obj)}" for obj in objects))
        names = ", ".join(self._speakable(obj.label) for obj in objects)
        self.subtask_manager.hri.say(f"I saw: {names}.")

    def _guide_to(self, obj) -> None:
        label = self._speakable(obj.label)
        place = self._place(obj.area, obj.sublocation)
        where = f"on {place}" if place else "nearby"
        self._step(f"Found: {obj.label} -> {self._where(obj)}. Guiding there")
        self.subtask_manager.hri.say(f"The {label} is {where}. Please follow me.")
        status, error = self.subtask_manager.nav.approach_point(
            (obj.x, obj.y), standoff=self.approach_standoff_m
        )
        if status == Status.EXECUTION_SUCCESS:
            self._step(f"Here is the {obj.label}")
            self._capture(f"found {obj.label}", {"area": obj.area, "sublocation": obj.sublocation})
            self.subtask_manager.hri.say(f"Here is the {label}.")
        else:
            self._step(f"Could not reach the {obj.label}")
            Logger.error(self, f"Could not reach the {label}: {error}")
            self.subtask_manager.hri.say(f"Sorry, I cannot get to the {label} from here.")

    def _is_yes_no(self, text: str) -> bool:
        """A bare yes/no reply (frida_constants positive/negative words), not a request."""
        hri = self.subtask_manager.hri
        yes_no = hri.affirmation_words | hri.negation_words
        words = "".join(c if c.isalnum() or c.isspace() else " " for c in text.lower()).split()
        return bool(words) and all(word in yes_no for word in words)

    @staticmethod
    def _is_stop(text: str) -> bool:
        return any(word in text.lower() for word in STOP_WORDS)

    def _confirm_question(self, heard: str, target: str) -> str:
        if self._is_stop(target):
            return "Do you want me to finish? Yes or no?"
        return f"Are you looking for: {target}? Yes or no?"

    def _ask(self, question: str) -> str:
        """Ask, hear and confirm, like GPSR's ask_and_confirm; "" when nothing was understood.

        Its own loop rather than hri.ask_and_confirm: that one always waits the default
        4 s for speech to start, and people need longer right after a question.
        """
        hri = self.subtask_manager.hri
        for attempt in range(1, self.listen_retries + 1):
            self._step(f"Listening ({attempt}/{self.listen_retries}): {question}")
            hri.say(question)
            status, text, _ = hri.hear(
                hotwords=FINALS_HOTWORDS, start_silence_time=self.listen_start_s
            )
            text = (text or "").strip()
            if status != Status.EXECUTION_SUCCESS or not text:
                self._step("No answer heard")
                hri.say("Sorry, I could not hear you.")
                continue
            self._step(f"Heard: {text}")
            if self._is_yes_no(text):
                self._step(f"Not a request: {text}")
                hri.say("Please tell me the name of the object.")
                continue
            status, answer = hri.confirm(
                self._confirm_question(text, text),
                use_keyword=True,
                retries=2,
                wait_between_retries=2.0,
            )
            if status == Status.EXECUTION_SUCCESS and answer == "yes":
                return text
            self._step(f"Not confirmed: {text}")
        return ""

    def command_loop(self) -> None:
        """Listen until told to stop, the time runs out, or nobody answers for a while."""
        question = "What are you looking for? Say stop when you are done."
        silent_rounds = 0
        while self._elapsed() < self.total_time_limit_s and silent_rounds < self.max_silent_rounds:
            text = self._ask(question)
            if not text:
                silent_rounds += 1
                continue
            silent_rounds = 0
            if self._is_stop(text):
                return
            self.handle_command(text)
            question = "What else are you looking for? Say stop when you are done."

    # ---------------------------------------------------------------------- run

    def run(self) -> None:
        hri = self.subtask_manager.hri

        self.state = ExecutionStates.WAIT_START
        hri.reset_task_status()
        self._step("Press start to begin")
        hri.say("Press the start button when you are ready.", wait=False)
        while not hri.start_button_clicked and not self.subtask_manager.get_mocked_areas():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.started_at = time.monotonic()
        self.memory.clear()
        _, areas = self.subtask_manager.nav.retrieve_areas()
        self.areas = areas or {}

        self.state = ExecutionStates.PATROL
        hri.say("I will look around the house.", wait=False)
        self.patrol(self.patrol_mode, self.patrol_time_limit_s)

        self.state = ExecutionStates.REPORT
        self.report_misplaced()

        self.state = ExecutionStates.COMMANDS
        self.command_loop()

        self.state = ExecutionStates.END
        self._step("Done")
        hri.say("I am done. Thank you.")


def main(args=None):
    rclpy.init(args=args)
    node = FinalsTaskManager()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
