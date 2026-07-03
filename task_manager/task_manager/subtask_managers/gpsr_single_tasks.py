import os
import time

import rclpy.time
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point  # noqa: F401 (registers transform type)

from frida_constants.hri_constants import GPSR_COMMANDS
from frida_constants.vision_constants import DETECTIONS_IMAGE_TOPIC, IMAGE_ORIENTED_TOPIC
from task_manager.utils.baml_client.types import (
    AnswerQuestion,
    GetVisualInfo,
    GoTo,
    PickObject,
    PlaceObject,
    SayWithContext,
)
from task_manager.utils.grasp_confirmation import count_by_class, picked_ok
from task_manager.utils.shelf_pick_logic import (
    find_target_on_level,
    height_matches_level,
    levels_from_sorted_heights,
)
from task_manager.utils.status import Status

from task_manager.subtask_managers.generic_tasks import GenericTask

RETRIES = 3

# The arm cannot pick or place below this base_link Z
MIN_REACHABLE_SHELF_Z = 0.20
# A pick/place location whose name contains one of these is treated as a shelf:
SHELF_FURNITURE_HINTS = ("shelf", "cabinet", "rack", "pantry", "bookcase")
SHELF_SETTLE_S = 3.0  # let the octomap build at a level's viewing pose


def search_command(command, objects: list[object]):
    for object in objects:
        if hasattr(object, command):
            method = getattr(object, command)
            if callable(method):
                return method
    return None


class GPSRSingleTask(GenericTask):
    """Class to manage the GPSR task"""

    def __init__(self, subtask_manager):
        """Initialize the class"""
        super().__init__(subtask_manager)

        node = self.subtask_manager.manipulation.node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node, spin_thread=True)
        self.last_location = ""  # furniture the robot last navigated to
        self._shelf_level_cache: dict[str, float] = {}
        try:
            from xarm_utils.shelf_levels import get_shelf_levels

            arena = int(os.environ.get("FRIDA_ARENA", "1"))
            self.shelf_level_heights = levels_from_sorted_heights(get_shelf_levels(arena))
        except Exception:
            self.shelf_level_heights = {1: 0.095, 2: 0.39, 3: 0.68, 4: 1.05}

    ## Nav
    def go_to(self, command: GoTo):
        """
        Navigate to a given location.

        Args:
            complement (str): The location to go to.
            characteristic (str, optional): Always empty.

        Purpose:
            - Go to a location.

        Preconditions:
            - The robot knows the location

        Behavior:
            - Move towards the specified location, while avoiding obstacles.

        Postconditions:
            - The robot is in the specified location
        """

        if isinstance(command, dict):
            command = GoTo(**command)

        self.subtask_manager.manipulation.move_to_position("nav_pose")
        location = self.subtask_manager.hri.query_location(command.location_to_go)[0]

        target = location.subarea if location.subarea else location.area
        pretty_target = target.replace("_", " ")
        self.subtask_manager.hri.say(f"Now I will go to the {pretty_target}.", wait=False)

        result, error = self.subtask_manager.nav.move_to_location(location.area, location.subarea)
        self.last_location = target
        return result, "arrived to:" + command.location_to_go

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        self.subtask_manager.manipulation.move_to_position("nav_pose")

        if say:
            target = sublocation if sublocation else location
            pretty_target = target.replace("_", " ")
            self.subtask_manager.hri.say(f"Now I will go to the {pretty_target}.", wait=False)

        result, error = self.subtask_manager.nav.move_to_location(location, sublocation)
        self.last_location = sublocation if sublocation else location
        return result

    ## Manipulation
    def _table_counts(self) -> dict:
        """Detect at the current pose and count detections per class."""
        _, dets = self.subtask_manager.vision.detect_objects()
        return count_by_class([d.classname for d in (dets or [])])

    def _confirm_pick_by_vision(self, target_label: str, before_counts: dict):
        """Re-look at the table; fail only if the picked object is clearly still there.

        Same feedback as the PPC task manager: the pick motion can report success
        while holding air, so only trust it if the target class count dropped.
        Returns (ok, after_counts) so a retry can reuse the fresh counts.
        """
        target = (target_label or "").lower()
        if not before_counts or before_counts.get(target, 0) == 0:
            # Nothing to compare against: stay lenient, trust the motion result.
            return True, before_counts
        self.subtask_manager.manipulation.move_to_position("table_stare")
        after = self._table_counts()
        if picked_ok(before_counts, after, target):
            return True, after
        return False, after

    def _at_shelf_furniture(self) -> bool:
        """True when the last navigated furniture is shelf-like (levels, not a table)."""
        loc = (self.last_location or "").lower()
        return any(hint in loc for hint in SHELF_FURNITURE_HINTS)

    def _detection_height(self, detection):
        """base_link Z of a detection's 3D point, or None on TF failure."""
        try:
            p = PointStamped()
            p.header.frame_id = "zed_left_camera_optical_frame"
            p.header.stamp = rclpy.time.Time().to_msg()  # latest available
            p.point.x = float(detection.px)
            p.point.y = float(detection.py)
            p.point.z = float(detection.pz)
            transformed = self.tf_buffer.transform(p, "base_link", timeout=Duration(seconds=1.0))
            return transformed.point.z
        except TransformException:
            return None

    def _shelf_counts(self, detections, level: float) -> dict:
        """Count detections whose height matches a shelf level, per class."""
        names = []
        for det in detections or []:
            h = self._detection_height(det)
            if det.classname and h is not None and height_matches_level(h, level):
                names.append(det.classname)
        return count_by_class(names)

    def _visit_shelf_level(self, height: float, object_name: str):
        """Move to a shelf level, settle, detect, and learn what sits there.

        Returns (found, before_counts). Only detections whose height matches this
        level are cached (classname -> height): the camera also frames adjacent
        levels and caching those would send later picks to the wrong level.
        """
        self.subtask_manager.manipulation.get_optimal_position_for_plane(
            height, tolerance=0.1, table_or_shelf=False, approach_plane=True
        )
        self.timeout(SHELF_SETTLE_S)
        s, detections = self.subtask_manager.vision.detect_objects()
        retry = 0
        while s != Status.EXECUTION_SUCCESS and retry < RETRIES:
            self.timeout(1)
            s, detections = self.subtask_manager.vision.detect_objects()
            retry += 1
        if s != Status.EXECUTION_SUCCESS or not detections:
            return (False, None)
        candidates = [
            (det.classname, h)
            for det in detections
            if (h := self._detection_height(det)) is not None
        ]
        for name, h in candidates:
            if name and height_matches_level(h, height):
                self._shelf_level_cache[name.lower()] = height
        if find_target_on_level(candidates, object_name, height) is not None:
            return (True, self._shelf_counts(detections, height))
        return (False, None)

    def _confirm_pick_shelf(self, target_label: str, before_counts: dict, level: float) -> bool:
        """Re-look at the shelf level; fail only if the object is clearly still there."""
        target = (target_label or "").lower()
        if not before_counts or before_counts.get(target, 0) == 0:
            return True
        self.subtask_manager.manipulation.get_optimal_position_for_plane(
            level, tolerance=0.1, table_or_shelf=False, approach_plane=True
        )
        self.timeout(2)
        _, dets = self.subtask_manager.vision.detect_objects()
        return picked_ok(before_counts, self._shelf_counts(dets, level), target)

    def _pick_from_shelf(self, object_name: str):
        """Find the target by detecting at each reachable shelf level, then pick there.

        Fast path: a level already cached for this object is confirmed with one
        detect; a miss drops the entry and falls back to the full sweep. Levels
        below MIN_REACHABLE_SHELF_Z are skipped (view-only). The pick keeps the
        found level's viewing pose (in_configuration) — table_stare frames only
        the lower levels (see docs/ai/shelf_pick_plan.md).
        """
        found_level = None
        before_counts = None

        cached = self._shelf_level_cache.get(object_name.lower())
        if cached is not None:
            found, before_counts = self._visit_shelf_level(cached, object_name)
            if found:
                found_level = cached
            else:
                self._shelf_level_cache.pop(object_name.lower(), None)

        if found_level is None:
            for height in sorted(self.shelf_level_heights.values()):
                if height < MIN_REACHABLE_SHELF_Z:
                    continue  # real level 1: too low to grasp from
                found, before_counts = self._visit_shelf_level(height, object_name)
                if found:
                    found_level = height
                    break

        if found_level is None:
            return Status.TARGET_NOT_FOUND

        s = self.subtask_manager.manipulation.pick_object(
            object_name, in_configuration=True, scan_environment=True
        )
        if s == Status.EXECUTION_SUCCESS and not self._confirm_pick_shelf(
            object_name, before_counts, found_level
        ):
            return Status.EXECUTION_ERROR
        return s

    def pick_object(self, command: PickObject):
        """
        Picks an object from a designated picking spot.

        Args:
            object_to_pick (str): Description of the object to pick.

        Purpose:
            To enable the robot to pick up an object.

        Preconditions:
            - The robot must be at the picking spot.
            - The object must be present on the picking spot.

        Behavior:
            - Executes the necessary manipulation to pick the object from the picking spot.

        Postconditions:
            - The robot holds the object.

        Pseudocode:
            - pick_object(complement)
        """
        if isinstance(command, dict):
            command = PickObject(**command)
        self.subtask_manager.nav.dock_table()
        self.subtask_manager.hri.publish_display_topic(DETECTIONS_IMAGE_TOPIC)

        if self._at_shelf_furniture():
            self.subtask_manager.hri.say(f"I will pick the {command.object_to_pick}.", wait=False)
            current_try = 0
            while current_try < RETRIES:
                s = self._pick_from_shelf(command.object_to_pick)
                if s == Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say(
                        f"I have picked the {command.object_to_pick}.", wait=True
                    )
                    return s, f"picked {command.object_to_pick}"
                self.subtask_manager.hri.say("My picking plan failed. I will try again", wait=True)
                current_try += 1
            return self.deus_pick(command)

        self.subtask_manager.manipulation.move_to_position("table_stare")
        self.subtask_manager.hri.say(f"I will pick the {command.object_to_pick}.", wait=False)

        current_try = 0

        while True:
            s, detections = self.subtask_manager.vision.detect_objects()
            current_try += 1

            if current_try >= RETRIES:
                self.subtask_manager.hri.say(
                    f"I could not see the object. Please hand me the {command.object_to_pick}."
                )
                return self.deus_pick(command)

            if len(detections) == 0:
                self.subtask_manager.hri.say("I didn't find any object. I will try again.")
                continue
            if s == Status.EXECUTION_SUCCESS:
                break

        labels = self.subtask_manager.vision.get_labels(detections)
        s, closest = self.subtask_manager.hri.find_closest(labels, command.object_to_pick)
        object_to_pick = closest.results[0]
        before_counts = count_by_class(labels)
        current_try = 0
        while True:
            if current_try >= RETRIES:
                return self.deus_pick(command)
            s = self.subtask_manager.manipulation.pick_object(object_to_pick)
            if s == Status.EXECUTION_SUCCESS:
                # PPC-style feedback: verify with vision that the object actually
                # left the table before claiming (and delivering) the pick.
                confirmed, after_counts = self._confirm_pick_by_vision(
                    object_to_pick, before_counts
                )
                if confirmed:
                    self.subtask_manager.hri.say(
                        f"I have picked the {command.object_to_pick}.", wait=True
                    )
                    return s, f"picked {command.object_to_pick}"
                self.subtask_manager.hri.say(
                    f"The {command.object_to_pick} is still there. I will try again.",
                    wait=True,
                )
                before_counts = after_counts  # fresh scene is the new baseline
            else:
                self.subtask_manager.hri.say("My picking plan failed. I will try again", wait=True)
            current_try += 1

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def deus_pick(self, command: PickObject):
        deus_machina_retries = 0
        self.subtask_manager.hri.say(
            f"I couldn't make a pick on my own. I will require some help picking the {command.object_to_pick}!"
        )
        while True:
            if deus_machina_retries >= RETRIES:
                self.subtask_manager.hri.say(
                    "I couldn't hear your confirmation, I will abort picking the object."
                )
                return Status.TARGET_NOT_FOUND, ""
            s, res = self.subtask_manager.hri.confirm(
                f"Have you placed the {command.object_to_pick} on my gripper?"
            )
            if res == "yes":
                self.subtask_manager.hri.say("Thank you. I will close my gripper")
                return self.subtask_manager.manipulation.close_gripper(), ""
            elif res == "no":
                return Status.TARGET_NOT_FOUND, ""

            else:
                deus_machina_retries += 1

    ## Manipulation
    def place_object(self, command: PlaceObject):
        """
        Places an object in the available location.

        Args:
            None

        Purpose:
            - To place an object

        Preconditions:
            - The robot must have an object.
            - The robot must be in front of the placing location.

        Behaviour:
            - Attempts to place the object in the specified location if possible.

        Postconditions:
            - The robot is in front of the placing location without the object.

        Pseudocode:
            place()
        """

        place_retries = 0
        self.subtask_manager.hri.say("I will place the object.", wait=False)

        if self._at_shelf_furniture():
            reachable = sorted(
                h for h in self.shelf_level_heights.values() if h >= MIN_REACHABLE_SHELF_Z
            )
            for height in reachable[:RETRIES]:
                self.subtask_manager.manipulation.get_optimal_position_for_plane(
                    height, tolerance=0.1, table_or_shelf=False, approach_plane=True
                )
                self.timeout(SHELF_SETTLE_S)
                self.subtask_manager.manipulation.move_to_position("front_stare")
                self.subtask_manager.manipulation.get_optimal_position_for_plane(
                    height, tolerance=0.1, table_or_shelf=False, approach_plane=False
                )
                s = self.subtask_manager.manipulation.place_on_shelf(
                    plane_height=height, tolerance=0.1
                )
                if s == Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say("I have placed the object.")
                    return Status.EXECUTION_SUCCESS, "placed"
            # All levels failed: fall through to the human-help fallback below.
        else:
            while place_retries < RETRIES:
                s = self.subtask_manager.manipulation.place()
                if s == Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say("I have placed the object.")
                    return Status.EXECUTION_SUCCESS, "placed"
                place_retries += 1

        self.subtask_manager.hri.say(
            "I couldn't place the object. Please help me place the object. When you confirm you have grabbed the object, I will open my gripper, so that you can place it."
        )

        deus_place_retries = 0
        while True:
            if deus_place_retries >= RETRIES:
                self.subtask_manager.hri.say(
                    "I couldn't hear your confirmation, I will abort placing the object."
                )
                return Status.EXECUTION_ERROR, ""
            s, res = self.subtask_manager.hri.confirm("Have you picked the object in my gripper?")
            if res == "yes":
                self.subtask_manager.hri.say("Thank you. I will open my gripper")
                return (
                    self.subtask_manager.manipulation.open_gripper(),
                    "placed with deus ex machina",
                )
            else:
                deus_place_retries += 1

    ## HRI
    def say_with_context(self, command: SayWithContext):
        """
        Say something grounded on the information known to the robot, which can include the results of
        previous executions, robot information, and general knowledge information.

        Args:
            complement (str): The original command.
            characteristic (str): the information to fetch, which can be a previous command or information to answer.

        Purpose:
            - To generate and vocalize a response based on the robot's available information, which may
            include execution results, robot-specific data, or general knowledge.

        Preconditions:
            - The necessary information must be available in the robot's database.

        Behavior:
            - If a command name is specified, fetch the execution results of the last command.
            - Use the provided complement to elaborate a response.
            - Generate the response using a language model and the fetched information.
            - Say the generated response.

        Postconditions:
            None.

        Pseudocode:
            say(llm_response(complement, fetch_info(characteristic)))
        """

        if isinstance(command, dict):
            command = SayWithContext(**command)

        context = command.previous_command_info[0]

        if context == "introduction":
            self.subtask_manager.hri.say("Hello, I am Frida. Nice to meet you.")
            return Status.EXECUTION_SUCCESS, "success"

        if context in GPSR_COMMANDS:
            s, history = self.subtask_manager.hri.query_command_history(
                query=command.user_instruction,
                action=command.previous_command_info[0],
            )
            last_command = history[0]
            s, answer = self.subtask_manager.hri.answer_with_context(
                command.user_instruction,
                f"Result of executing {context} function on the context provided: {last_command.result}.",
            )
            self.subtask_manager.hri.say(answer, wait=True)
            return Status.EXECUTION_SUCCESS, "success"
        else:
            s, response, score = self.subtask_manager.hri.answer_question(
                command.previous_command_info[0]
            )

            self.subtask_manager.hri.say(response, wait=True)
            return Status.EXECUTION_SUCCESS, "success"

    ## HRI
    def answer_question(self, command: AnswerQuestion):
        """
        Answers a user's question by asking, confirming, and responding.

        Args:
            complement (str, optional): Always empty.
            characteristic (str, optional): Always empty.

        Purpose:
            To answer the user's question by following a structured process.

        Preconditions:
            The robot must be in front of the user.

        Behavior:
            - Ask a question.
            - Confirm the question.
            - Provide an answer to the question.

        Postconditions:
            None.

        Pseudocode:
            while get_question():
                if confirm_question():
                    contextual_say('Please answer my question', question)
        """

        def confirm_question(interpreted_text, target_info):
            return f"Is your question: {target_info}?"

        status, question = self.subtask_manager.hri.ask_and_confirm(
            "Please ask your question",
            "LLM_question",
            context="The user was asked to say a question. We want to infer his question from the response",
            confirm_question=confirm_question,
            retries=3,
            min_wait_between_retries=5.0,
            skip_extract_data=True,
            always_confirm=True,
        )

        if status != Status.EXECUTION_SUCCESS:
            self.subtask_manager.hri.say("I am sorry, I could not understand your question.")
            return Status.TARGET_NOT_FOUND, ""

        question = SayWithContext(
            action="say_with_context",
            user_instruction=f"Please answer my question: {question}",
            previous_command_info=[question],
        )
        return self.say_with_context(question)

    ## Vision
    def get_visual_info(self, command: GetVisualInfo):
        """
        Retrieves visual information about an object based on the specified complement
        and optional characteristic.

        Args:
            complement (str): A descriptor indicating the type of object to analyze.
                Expected values include "biggest object", "largest object",
                "smallest object", "heaviest object", "lightest object",
                or "thinnest object".
            characteristic (str, optional): A filter specifying the object category
                to consider (e.g., "object category"). If empty, there is no filter.

        Purpose:
            To obtain specific data about an object in an image.

        Preconditions:
            The camera must be positioned appropriately to capture the required frame.

        Behavior:
            Analyzes the image to extract the requested data, applying the
            characteristic filter if provided.

        Postconditions:
            The robot saves the specified information for further use.
        """
        if isinstance(command, dict):
            command = GetVisualInfo(**command)

        # Moondream answers about the live image — show the camera feed
        self.subtask_manager.hri.publish_display_topic(IMAGE_ORIENTED_TOPIC)

        status, result = self.subtask_manager.vision.visual_info(
            command.measure, command.object_category
        )

        self.subtask_manager.hri.say(f"{result}", wait=False)

        return status, result
