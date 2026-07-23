import time

from geometry_msgs.msg import PointStamped
from frida_constants.vision_enums import DetectBy, Gestures, Poses, is_value_in_enum
from frida_constants.vision_constants import (
    FACE_RECOGNITION_IMAGE,
    DETECTIONS_IMAGE_TOPIC,
    IMAGE_ORIENTED_TOPIC,
    IMAGE_TOPIC,
    TRACKER_IMAGE_TOPIC,
)
from task_manager.utils.baml_client.types import (
    Count,
    FindPersonByName,
    FollowPersonUntil,
    GetPersonInfo,
    GiveObject,
    GuidePersonTo,
)
from task_manager.utils.status import Status

from task_manager.subtask_managers.generic_tasks import GenericTask

# Physical person-following is disabled for GPSR; set True to restore the
# tracker-based follow, which uses the constants below.
FOLLOW_PERSON_ENABLED = False

FOLLOW_STOP_KEYWORDS = ["stop", "stop following", "halt", "you can stop"]
FOLLOW_LISTEN_TIMEOUT = 2.5
FOLLOW_MAX_DURATION = 120.0
FOLLOW_ARRIVED_DISTANCE = 1.5
FOLLOW_TRACK_ATTEMPTS = 5
FOLLOW_LOST_TIMEOUT = 3.0
FOLLOW_RELOCK_ATTEMPTS = 3
FIND_PERSON_MAX_ROUNDS = 4
# Scan points closer than this (map frame, meters) count as the same person across rounds.
FIND_PERSON_VISITED_RADIUS = 0.75


class GPSRTask(GenericTask):
    """Class to manage the GPSR task"""

    def __init__(self, subtask_manager):
        """Initialize the class"""
        super().__init__(subtask_manager)
        # Angles are relative to current position
        self.pan_angles = [-35, 70]
        self.color_list = ["blue", "yellow", "black", "white", "red", "orange", "gray", "green"]
        self.clothe_list = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket", "jeans"]

    def navigate_to(self, location, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        self.subtask_manager.manipulation.move_to_position("nav_pose")

        if say:
            target = sublocation if sublocation else location
            pretty_target = target.replace("_", " ")
            self.subtask_manager.hri.say(f"Now I will go to the {pretty_target}.", wait=False)

        result, error = self.subtask_manager.nav.move_to_location(location, sublocation)
        return result

    def get_path_info(
        self, location, sublocation: str = "", from_location: str = "", from_sublocation: str = ""
    ):
        """Query the real path distance to a location without moving the
        robot, so HRI can use it to make decisions.

        Args:
            location: Destination area name from areas.json.
            sublocation: Destination sublocation (defaults to safe_place).
            from_location: Origin area; empty means the robot's current pose.
            from_sublocation: Origin sublocation (defaults to safe_place).

        Returns:
            (Status, info): info is {"distance": meters} on success, or an
            error string on failure.
        """
        return self.subtask_manager.nav.get_path_info(
            location, sublocation, from_location, from_sublocation
        )

    ## HRI, Manipulation
    def give_object(self, command: GiveObject):
        """
        Handles the action of giving an object to a person.

        Args:
            complement (str, optional): Always empty.
            characteristic (str, optional): Always empty.

        Purpose:
            - Give an object to a person.

        Preconditions:
            - The robot must have possession of an object.
            - The robot must be positioned in front of a person.

        Behavior:
            - Moves the robot's arm to the 'giving' position.
            - Waits for confirmation that the person has grabbed the object.
            - Releases the object by opening the gripper.
            - Moves the robot's arm back to the 'standard' position.

        Postconditions:
            - The robot is in front of a person without holding an object.
        """
        self.subtask_manager.hri.say(
            "I will give you the object. Once you have picked the object, I will open my gripper",
            wait=False,
        )
        self.subtask_manager.manipulation.move_to_position(named_position="receive_object")

        while True:
            s, res = self.subtask_manager.hri.confirm("Have you grabbed the object?")
            if res == "yes":
                break
            else:
                self.subtask_manager.hri.say(
                    "I will give you the object. Once you have picked the object, I will open my gripper.",
                    wait=True,
                )

        self.subtask_manager.hri.say("I will now release the object.", wait=True)
        self.subtask_manager.manipulation.open_gripper()

        return Status.EXECUTION_SUCCESS, "object given"

    def _teardown_follow(self):
        """Restore arm pose after a follow (camera flip stays cleared —
        CARRY_POSE never rotates the camera)."""
        self.subtask_manager.vision.camera_upside_down(False)
        self.subtask_manager.manipulation.move_to_position("nav_pose")

    def _recover_lost_person(self) -> bool:
        """Try to re-lock the tracker after the person has been lost.

        Pauses base+arm follow (person_goal_smoother has already frozen the
        goal on its own timeout), re-centers the camera, asks the person to
        come back and re-locks with track_person(True). The tracker's ReID
        re-acquisition covers brief occlusions on its own — this handles the
        long losses it couldn't recover from.

        Returns True when tracking (and follow) resumed, False to give up.
        """
        self.subtask_manager.nav.follow_person(False)
        self.subtask_manager.manipulation.follow_person(False)
        # Re-center the camera: the arm may be panned far off after the chase.
        self.subtask_manager.manipulation.move_to_position("carry_pose")
        self.subtask_manager.hri.say(
            "I lost you. Please come back and stand in front of me.", wait=True
        )
        for _ in range(FOLLOW_RELOCK_ATTEMPTS):
            if self.subtask_manager.vision.track_person(True) == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say("I found you again, let's continue.", wait=False)
                self.subtask_manager.nav.follow_person(True)
                self.subtask_manager.manipulation.follow_person(True)
                return True
            time.sleep(1.0)
        return False

    ## HRI, Nav, Vision, Manipulation
    def follow_person_until(self, command: FollowPersonUntil):
        """
        GPSR follow command with physical following disabled
        (FOLLOW_PERSON_ENABLED = False): the robot tells the person it cannot
        follow them, asks for the location they want to go (or reuses the
        commanded destination), and navigates there with the person walking
        along — a guide in place of a follow.

        Args:
            destination (str): Where the person wanted to be followed to.
                'cancelled' (follow until told to stop) carries no
                destination, so the robot asks the person for one.

        Postconditions:
            - The robot (and, walking with it, the person) is at the
              destination.
        """
        if isinstance(command, dict):
            command = FollowPersonUntil(**command)

        if FOLLOW_PERSON_ENABLED:
            return self._follow_person_tracking(command)

        self.subtask_manager.hri.publish_display_topic(IMAGE_ORIENTED_TOPIC)

        destination = (command.destination or "").strip().lower()
        until_cancelled = destination in ("", "cancelled", "canceled")

        loc_text = command.destination
        if until_cancelled:
            self.subtask_manager.hri.say("I am sorry, I cannot follow you right now.", wait=True)
            status, loc_text = self.subtask_manager.hri.ask_and_confirm(
                question="Please tell me the location you want to go, and I will take you there.",
                query="location",
                context="The user was asked for the location they want to go to. "
                "We want to infer the location from the response",
            )
            if status != Status.EXECUTION_SUCCESS or not loc_text:
                self.subtask_manager.hri.say(
                    "Sorry, I could not understand the location.", wait=False
                )
                return Status.TARGET_NOT_FOUND, "no destination to guide to"
        else:
            self.subtask_manager.hri.say(
                "I am sorry, I cannot follow you right now, "
                f"but I can take you to the {command.destination}.",
                wait=True,
            )

        location = self.subtask_manager.hri.query_location(loc_text)[0]
        target = location.subarea if location.subarea else location.area
        pretty_target = target.replace("_", " ")
        self.subtask_manager.hri.say(f"Please follow me to the {pretty_target}.", wait=True)
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        result, _ = self.subtask_manager.nav.move_to_location(location.area, location.subarea)
        if result == Status.EXECUTION_SUCCESS:
            self.subtask_manager.hri.say(f"We have arrived to the {pretty_target}.", wait=False)
        return result, f"guided person to {pretty_target} instead of following"

    def _follow_person_tracking(self, command: FollowPersonUntil):
        """
        Follow a person until a stop condition is met (HRIC FOLLOW_PERSON port).
        Inactive while FOLLOW_PERSON_ENABLED is False — kept intact so the
        tracker-based follow can be restored with the flag.

        Args:
            destination (str): Where to stop following. A location name means
                the robot follows the person until it arrives there;
                'cancelled' means it follows until the person says a stop
                keyword (FOLLOW_STOP_KEYWORDS).

        Preconditions:
            - The robot is in front of the person it will follow (the
              interpreter guarantees a find_person action precedes this one).

        Behavior:
            - Moves the arm to CARRY_POSE (camera upright, tilted slightly
              higher than nav_pose) and clears the camera-flip flag, so follow
              starts from the same camera configuration no matter how a
              previous action left the camera.
            - Locks the person tracker (track_person), then starts base-follow
              (nav) and arm-follow (manipulation, keeps the person in frame).
            - Follows until a stop keyword is heard, the destination is reached
              (path distance below FOLLOW_ARRIVED_DISTANCE), or the safety cap
              FOLLOW_MAX_DURATION expires.
            - If the tracker never locks, degrades to plain navigation to the
              destination (asking for one when following until cancelled).

        Postconditions:
            - Follow and tracking are off; the arm is back in nav_pose. The
              robot is where the person stopped it or at the target location.
        """
        if isinstance(command, dict):
            command = FollowPersonUntil(**command)

        # Show the tracker's annotated feed (locked-person bbox) while following
        self.subtask_manager.hri.publish_display_topic(TRACKER_IMAGE_TOPIC)
        self.subtask_manager.vision.deactivate_face_recognition()

        # Resolve the stop condition up front. BAML emits 'cancelled' (see
        # robot_commands.baml FollowPersonUntil), but tolerate the US spelling.
        destination = (command.destination or "").strip().lower()
        until_cancelled = destination in ("", "cancelled", "canceled")
        location = None
        if not until_cancelled:
            try:
                location = self.subtask_manager.hri.query_location(command.destination)[0]
            except Exception:
                location = None

        # Follow always runs in CARRY_POSE (xarm_configurations): like nav_pose
        # but with the camera tilted a bit higher, and the camera NOT rotated.
        # Explicitly clear the flip flag so the tracker treats frames as
        # upright no matter what orientation a previous action left announced.
        self.subtask_manager.manipulation.move_to_position("carry_pose")
        self.subtask_manager.vision.camera_upside_down(False)
        time.sleep(1.0)  # let the pose settle before locking the tracker

        # Lock the tracker on the person. track_person(True) is the start/stop
        # command — get_track_person() is only a status query and never starts
        # tracking. The tracker publishes the person's 3D point that
        # person_goal_smoother turns into the moving Nav2 goal.
        self.subtask_manager.hri.say(
            "Please stand in front of me so I can start following you.", wait=True
        )
        tracking = False
        for attempt in range(FOLLOW_TRACK_ATTEMPTS):
            if self.subtask_manager.vision.track_person(True) == Status.EXECUTION_SUCCESS:
                tracking = True
                break
            if attempt < FOLLOW_TRACK_ATTEMPTS - 1:
                self.subtask_manager.hri.say(
                    "I cannot see you yet. Please stand right in front of me."
                )

        if not tracking:
            # No locked target -> there is no goal to follow; degrade to plain
            # navigation instead of chasing the smoother's dummy pose.
            self.subtask_manager.vision.track_person(False)
            self._teardown_follow()
            loc_text = command.destination
            if until_cancelled:
                self.subtask_manager.hri.say("I could not lock onto you, so I cannot follow you.")
                status, loc_text = self.subtask_manager.hri.ask_and_confirm(
                    question="Please tell me where to go instead.",
                    query="location",
                    context="The user was asked to say the location. We want to infer the location from the response",
                )
                if status != Status.EXECUTION_SUCCESS or not loc_text:
                    return Status.TARGET_NOT_FOUND, "could not lock on person to follow"
            else:
                self.subtask_manager.hri.say(
                    f"I could not lock onto you, but I will go to the {command.destination}.",
                )
            location = self.subtask_manager.hri.query_location(loc_text)[0]
            result, _ = self.subtask_manager.nav.move_to_location(location.area, location.subarea)
            return result, "could not follow, navigated to " + str(loc_text)

        self.subtask_manager.hri.say(
            "I will start following you now, you can start walking. "
            "Say stop whenever you want me to stop.",
            wait=True,
        )

        # Base + arm follow. Arm-follow (joint1 pan keeping the person centered)
        # keeps the person in frame as the base maneuvers, reducing losses.
        self.subtask_manager.nav.follow_person(True)
        self.subtask_manager.manipulation.follow_person(True)

        follow_start = time.time()
        lost_since = None
        stop_reason = "safety timeout"
        while True:
            status, _ = self.subtask_manager.hri.interpret_keyword(
                FOLLOW_STOP_KEYWORDS, timeout=FOLLOW_LISTEN_TIMEOUT, play_chime=False
            )
            if status == Status.EXECUTION_SUCCESS:
                stop_reason = "stop requested"
                break

            # Lost-person fallback: get_track_person reports whether the tracker
            # currently has the target in frame (ReID re-acquisition included).
            # Only after FOLLOW_LOST_TIMEOUT of continuous loss do we interrupt
            # the follow and renegotiate — brief occlusions self-heal.
            if self.subtask_manager.vision.get_track_person() == Status.EXECUTION_SUCCESS:
                lost_since = None
            else:
                lost_since = lost_since or time.time()
                if time.time() - lost_since > FOLLOW_LOST_TIMEOUT:
                    if self._recover_lost_person():
                        lost_since = None
                    else:
                        stop_reason = "person lost"
                        break

            if location is not None:
                s, info = self.subtask_manager.nav.get_path_info(location.area, location.subarea)
                if (
                    s == Status.EXECUTION_SUCCESS
                    and isinstance(info, dict)
                    and info.get("distance", float("inf")) <= FOLLOW_ARRIVED_DISTANCE
                ):
                    stop_reason = f"arrived at {command.destination}"
                    break
            if time.time() - follow_start > FOLLOW_MAX_DURATION:
                self.subtask_manager.hri.node.get_logger().warning(
                    "follow_person_until: timed out without a stop condition"
                )
                break

        # Stop nav-follow, arm-follow AND the tracker, then restore camera/pose.
        self.subtask_manager.nav.follow_person(False)
        self.subtask_manager.manipulation.follow_person(False)
        self.subtask_manager.vision.track_person(False)
        self._teardown_follow()

        if stop_reason == "person lost":
            if location is not None:
                # We know where the follow was headed — meet the person there.
                self.subtask_manager.hri.say(
                    f"I could not find you again, so I will meet you at the {command.destination}.",
                    wait=False,
                )
                result, _ = self.subtask_manager.nav.move_to_location(
                    location.area, location.subarea
                )
                return result, "lost person, navigated to " + str(command.destination)
            self.subtask_manager.hri.say("I lost you and could not find you again.", wait=False)
            # FAILURE lets the behaviour tree retry the follow (fresh lock-on).
            return Status.EXECUTION_ERROR, "lost person while following"

        self.subtask_manager.hri.say("Okay, I will stop following you.", wait=False)
        return Status.EXECUTION_SUCCESS, "followed person until " + stop_reason

    ## HRI, Nav
    def guide_person_to(self, command: GuidePersonTo):
        """
        Guides a person to a specified target location.

        Args:
            complement (str): "person" or a name. If a name is provided, it can be used to improve interaction.
            characteristic (str, optional): location or room to guide the person to.

        Purpose:
            - Guide a person to a target location.

        Preconditions:
            - The robot must be positioned in front of the person.
            - If a name is specified, the robot already knows the name and face of the person.

        Behavior:
            - The robot expresses its intent to guide the person.
            - The robot moves to the specified location.
            - Optionally, the robot confirms that the person is following it.

        Postconditions:
            - Both the robot and the guided person are at the target location.
            - Store command execution status in the database.

        Pseudocode:
            - say(intent)
            - go(characteristic)

        """
        if isinstance(command, dict):
            command = GuidePersonTo(**command)

        self.subtask_manager.manipulation.move_to_position("nav_pose")
        location = self.subtask_manager.hri.query_location(command.destination_room)[0]
        target = location.subarea if location.subarea else location.area
        pretty_target = target.replace("_", " ")
        self.subtask_manager.hri.say(f"Now we will go to the {pretty_target}.", wait=False)
        result, error = self.subtask_manager.nav.move_to_location(location.area, location.subarea)

        self.subtask_manager.hri.say(f"We have arrived to {command.destination_room}!", wait=True)
        return Status.EXECUTION_SUCCESS, "arrived to " + command.destination_room

    ## HRI, Vision
    def get_person_info(self, command: GetPersonInfo):
        """
        Get specific information about a person.

        Args:
            complement (str): The type of information to retrieve. If "gesture" or "posture", the robot computes the information visually. If "name", the robot fetches the name from known names or interacts with the person if the name is unknown.
            characteristic (str): Always empty string.

        Preconditions:
            - The robot must be in front of the person.

        Behaviour:
            - If the complement is "gesture" or "posture", the robot computes the information visually.
            - If the complement is "name", the robot fetches the name from previously known names or interacts with the person if the name is not known.

        Postconditions:
            - The robot saves the specified information about the person in the database.

        Pseudocode:
            if complement == 'gesture' or complement == 'posture':
                return gest_posture_analyzer()
            else:
                return get_person_name()
        """

        if isinstance(command, dict):
            command = GetPersonInfo(**command)

        if command.info_type == "gesture":
            command.info_type = DetectBy.GESTURES.value
        elif command.info_type == "pose":
            command.info_type = DetectBy.POSES.value

        self.subtask_manager.manipulation.move_to_position("front_stare")

        self.subtask_manager.hri.say(f"I will search for the {command.info_type} of a person.")

        if command.info_type != "name":
            # find_person_info -> PersonPoseGesture service on gpsr_commands,
            # which annotates the GPSR feed (IMAGE_TOPIC) — show that feed.
            self.subtask_manager.hri.publish_display_topic(IMAGE_TOPIC)
            person_retries = 0
            while person_retries < 2:
                self.timeout(1)
                s, res = self.subtask_manager.vision.find_person_info(command.info_type)

                if s == Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say(
                        f"The person is {res}.",
                    )
                    return s, res
                person_retries += 1

            # Look for the person again but at a lower degree
            self.subtask_manager.manipulation.check_lower(30)
            person_retries = 0

            while person_retries < 2:
                self.timeout(1)
                s, res = self.subtask_manager.vision.find_person_info(command.info_type)

                if s == Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say(
                        f"The person is {res}.",
                    )
                    return s, res
                person_retries += 1

            self.subtask_manager.hri.say(
                f"I couldn't find the person's {command.info_type}.",
            )
            return Status.TARGET_NOT_FOUND, "Unkown"

        else:
            # get name
            self.subtask_manager.hri.publish_display_topic(FACE_RECOGNITION_IMAGE)
            # Without this the face_recognition node stays idle (run() bails on
            # vision_active False — e.g. after follow_person deactivates it):
            # the displayed feed is black and get_person_name never updates.
            self.subtask_manager.vision.activate_face_recognition()
            self.subtask_manager.hri.say(
                "I will check if I know your name.",
            )
            current_attempt = 0
            while current_attempt < 3:
                current_attempt += 1
                s, res = self.subtask_manager.vision.get_person_name()
                if s == Status.EXECUTION_SUCCESS and res != "Unknown":
                    self.subtask_manager.hri.say(f"Hi {res}, nice to meet you again!")
                    return Status.EXECUTION_SUCCESS, res
            else:
                self.subtask_manager.hri.say(
                    "Hi, I'm Frida. I don't think I know you yet.",
                )
                s, response = self.subtask_manager.hri.ask_and_confirm(
                    question="Can you please tell me your name?",
                    query="name",
                    context="The user was asked to say their name. We want to infer his name from the response",
                )
                if s == Status.EXECUTION_SUCCESS:
                    save_name_retries = 0
                    while save_name_retries < 3:
                        self.subtask_manager.hri.say(
                            "Please stand in front of me so I can save your face.",
                        )

                        if (
                            self.subtask_manager.vision.save_face_name(response)
                            == Status.EXECUTION_SUCCESS
                        ):
                            self.subtask_manager.hri.say(
                                f"Nice to meet you, {response}. I have saved your name.",
                            )
                            return Status.EXECUTION_SUCCESS, response

                        save_name_retries += 1

                    self.subtask_manager.hri.say(
                        "Sorry, I couldn't save your name.",
                    )
                    return Status.TARGET_NOT_FOUND, response
                else:
                    self.subtask_manager.hri.say(
                        "I couldn't undestand your name",
                    )
                    return Status.EXECUTION_ERROR, "name not found"

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def count_objects(self, object_name: str):
        self.subtask_manager.manipulation.move_to_position("table_stare")
        self.subtask_manager.hri.publish_display_topic(DETECTIONS_IMAGE_TOPIC)
        self.subtask_manager.hri.say(
            f"I am going to count {object_name}.",
        )

        # Get detections from object detector
        status, labels = self.subtask_manager.vision.count_objects(object_name)
        if status != Status.EXECUTION_SUCCESS or not labels:
            self.subtask_manager.hri.say(f"I didn't find any {object_name}.")
            # Back to the oriented live camera (the display default) — the raw
            # ZED topic is unrotated and has no detections drawn on it.
            self.subtask_manager.hri.publish_display_topic(IMAGE_ORIENTED_TOPIC)
            return Status.TARGET_NOT_FOUND, f"0 ({object_name} counted)"

        # Use LLM to count matching objects from detections
        status, count = self.subtask_manager.hri.count_from_detections(labels, object_name)
        if status == Status.EXECUTION_SUCCESS:
            self.subtask_manager.hri.say(f"I have counted {count} {object_name}.")
        else:
            self.subtask_manager.hri.say(f"I couldn't determine how many {object_name} there are.")

        self.subtask_manager.hri.publish_display_topic(IMAGE_ORIENTED_TOPIC)
        return status, str(count) + f" ({object_name} counted)"

    ## Manipulation, Vision
    def count(self, command: Count):
        """
        Count the number of specific targets based on the given parameters.

        Args:
            complement (str): Specifies the context for counting.
                Options include placement location or a room.
            characteristic (str): Specifies the type of target to count.
                Options include objects, "person with " + cloth + color, person with some posture,
                or person with some gesture.

        Purpose:
            - Compute the amount of a specific target to later communicate it to a person.

        Preconditions:
            - Assumes the robot is at a location.

        Behavior:
            - If a placement location is given, count the number of the specified item.
            - If a room is given, gaze at several non-overlapping points and return the
              sum of the target across the several pictures.

        Postconditions:
            - The count result is saved in the database.

        Pseudocode:
            For each degree in [-45, 0, 45]:
                Gaze at the specified degree.
                Add the count of the target in the frame to the total count.
            Store the total count.
        """

        if isinstance(command, dict):
            command = Count(**command)

        possibilities = (
            [v.value for v in Gestures] + [v.value for v in Poses] + ["clothes"] + ["color"]
        )
        possibilities = [p for p in possibilities if p != "unknown"]

        status, closest = self.subtask_manager.hri.find_closest(
            possibilities, command.target_to_count
        )
        value = closest.results[0]

        print(f"Value: {value}", command.target_to_count)

        if (
            "person" not in command.target_to_count.lower()
            and "people" not in command.target_to_count.lower()
        ):
            return self.count_objects(command.target_to_count)

        self.subtask_manager.manipulation.move_to_position("front_stare")

        counter = 0

        if not is_value_in_enum(value, Gestures) and not is_value_in_enum(value, Poses):
            clothing = command.target_to_count.lower().strip().split(" ")
            s, color_match = self.subtask_manager.hri.find_closest(self.color_list, clothing[0])
            cache_color = color_match.results[0]
            s, cloth_match = self.subtask_manager.hri.find_closest(
                self.clothe_list, " ".join(clothing[1:] if len(clothing) > 1 else clothing)
            )
            cache_cloth = cloth_match.results[0]
            value = f"{cache_color} {cache_cloth}s"
            command.target_to_count = value

        self.subtask_manager.hri.publish_display_topic(IMAGE_TOPIC)
        self.subtask_manager.hri.say(
            f"I am going to count the {value}.",
        )

        for degree in self.pan_angles:
            self.subtask_manager.manipulation.pan_to(degree)

            if is_value_in_enum(value, Gestures):
                status, count = self.subtask_manager.vision.count_by_gesture(value)
            elif is_value_in_enum(value, Poses):
                status, count = self.subtask_manager.vision.count_by_pose(value)
            else:
                status, count = self.subtask_manager.vision.count_by_color(cache_color, cache_cloth)

            if status == Status.EXECUTION_SUCCESS:
                counter += count
                self.subtask_manager.hri.say(f"I have counted {count} {command.target_to_count}.")

            elif status == Status.TARGET_NOT_FOUND:
                self.subtask_manager.hri.say(
                    f"I didn't find any {command.target_to_count}.",
                )

        self.subtask_manager.hri.say(
            f"I have counted {counter} {command.target_to_count} in the room.",
        )
        return Status.EXECUTION_SUCCESS, "counted " + str(counter) + " " + command.target_to_count

    def _approach_with_arm(self, point) -> bool:
        """Drive to a person point with the arm tucked (nav_pose), then look
        at them (front_stare) ready for the interaction that follows.

        The point is projected to the MAP frame FIRST: camera-frame points go
        stale the moment the arm moves (tucking changes the camera pose, and
        nav would project the point through the wrong transform)."""
        xy = self.subtask_manager.nav._resolve_map_xy(point)
        if xy is None:
            return False
        map_pt = PointStamped()
        map_pt.header.frame_id = "map"
        map_pt.point.x, map_pt.point.y = xy
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        nav_status, _ = self.subtask_manager.nav.approach_point(map_pt)
        # Face the person even after a partial approach: every follow-up step
        # (face rec, asking, giving) needs the camera looking front.
        self.subtask_manager.manipulation.move_to_position("front_stare")
        return nav_status == Status.EXECUTION_SUCCESS

    def _approach_found_person(self, point=None) -> bool:
        """Walk up to a found person instead of asking them to come over.

        Preferred path: `point` is the MATCHED person's own 3D point (from
        vision.last_person_points, filled by count_by_pose/gesture/color), so
        the robot approaches exactly the person that matched the attribute —
        not just the largest one in frame. Without a point, falls back to
        locking the tracker (largest person) and reading its point. Returns
        False if every step failed (caller then asks the person to approach)."""
        if point is None and self.subtask_manager.vision.last_person_points:
            # Nearest matched person (camera-frame points: norm = distance)
            point = min(
                self.subtask_manager.vision.last_person_points,
                key=lambda p: p.point.x**2 + p.point.y**2 + p.point.z**2,
            )
        if point is not None:
            return self._approach_with_arm(point)

        if self.subtask_manager.vision.track_person(True) != Status.EXECUTION_SUCCESS:
            return False
        try:
            status, tracked = self.subtask_manager.vision.get_tracked_person_point()
            if status != Status.EXECUTION_SUCCESS or tracked is None:
                return False
            return self._approach_with_arm(tracked)
        finally:
            self.subtask_manager.vision.track_person(False)

    def find_person(self, command: FindPersonByName):
        if isinstance(command, dict):
            command = FindPersonByName(**command)

        # GPSR's own annotated feed: person boxes + matched highlight from
        # gpsr_commands (the counts run there, NOT in hric_commands)
        self.subtask_manager.hri.publish_display_topic(IMAGE_TOPIC)
        self.subtask_manager.manipulation.move_to_position("front_stare")

        possibilities = (
            [v.value for v in Gestures] + [v.value for v in Poses] + ["clothes"] + ["color"]
        )

        possibilities = [p for p in possibilities if p != "unknown"]

        status, closest = self.subtask_manager.hri.find_closest(
            possibilities, command.attribute_value
        )
        value = closest.results[0]

        self.subtask_manager.manipulation.move_to_position("front_stare")

        if not is_value_in_enum(value, Gestures) and not is_value_in_enum(value, Poses):
            clothing = command.attribute_value.lower().strip().split(" ")
            s, color_match = self.subtask_manager.hri.find_closest(self.color_list, clothing[0])
            cache_color = color_match.results[0]
            s, cloth_match = self.subtask_manager.hri.find_closest(
                self.clothe_list, " ".join(clothing[1:] if len(clothing) > 1 else clothing)
            )
            cache_cloth = cloth_match.results[0]
            value = f"{cache_color} {cache_cloth}s"
            command.attribute_value = value

        self.subtask_manager.hri.say(
            f"Searching for {value}.",
        )

        for degree in self.pan_angles:
            self.subtask_manager.manipulation.pan_to(degree)

            if command.attribute_value == "":
                status, count = self.subtask_manager.vision.count_by_pose(Poses.STANDING.value)
            elif is_value_in_enum(value, Gestures):
                status, count = self.subtask_manager.vision.count_by_gesture(value)
            elif is_value_in_enum(value, Poses):
                status, count = self.subtask_manager.vision.count_by_pose(value)
            else:
                if cache_color is None or cache_cloth is None:
                    clothing = command.attribute_value.lower().strip().split(" ")
                    s, color_match = self.subtask_manager.hri.find_closest(
                        self.color_list, clothing[0]
                    )
                    cache_color = color_match.results[0]
                    s, cloth_match = self.subtask_manager.hri.find_closest(
                        self.clothe_list, " ".join(clothing[1:] if len(clothing) > 1 else clothing)
                    )
                    cache_cloth = cloth_match.results[0]

                status, count = self.subtask_manager.vision.count_by_color(cache_color, cache_cloth)

            # If next command is "go_to" dont ask to approach robot
            if status == Status.EXECUTION_SUCCESS and count > 0:
                self.subtask_manager.hri.say(
                    f"I found a {command.attribute_value}.",
                )
                if not self._approach_found_person():
                    self.subtask_manager.hri.say("Please approach me.")
                break

            elif status == Status.TARGET_NOT_FOUND:
                self.subtask_manager.hri.say(
                    f"I didn't find any person with {command.attribute_value}.",
                )
                self.subtask_manager.hri.say(
                    "Please approach me.",
                )

        return Status.EXECUTION_SUCCESS, "found" + command.attribute_value

    ## HRI, Manipulation, Nav, Vision
    def find_person_by_name(self, command: FindPersonByName):
        """
        Find a person by their name and save the information for further interactions. Further interaction may include: following the person,
        guiding the person, interacting with the person, describing the person, give an object to a person.

        Args:
            complement (str): The name of the person to search for.
            characteristic (str, optional): Always an empty string.

        Behavior:
            - The robot scans the room (people are filtered to the robot's
              current room polygon by gpsr_commands) and walks up to each
              not-yet-visited person, nearest first.
            - Visited people are remembered by their map position
              (FIND_PERSON_VISITED_RADIUS) and, best-effort, by face — so the
              same person is never questioned twice even when face
              recognition sees no face.
            - Identification is a closed yes/no question ("Are you <name>?")
              answered via the yes/no keyword spotter — no open-ended name
              extraction, which broke on noisy transcriptions.
            - If nobody matched after FIND_PERSON_MAX_ROUNDS, falls back to
              asking the target to come over and confirming the same way.

        Preconditions:
            - The robot must be at a location where it can search for people.

        Postconditions:
            - The robot approaches the person with the specified name.
            - The robot saves information about all the people it encounters.
        """
        if isinstance(command, dict):
            command = FindPersonByName(**command)

        # Face-recognition flow: show the annotated face feed
        self.subtask_manager.hri.publish_display_topic(FACE_RECOGNITION_IMAGE)
        self.subtask_manager.vision.activate_face_recognition()
        self.subtask_manager.manipulation.move_to_position("front_stare")
        target = self.subtask_manager.hri.remove_punctuation(command.name).lower()
        self.subtask_manager.hri.say(f"I'm looking for {command.name}.", wait=False)

        # ACTIVE search: the robot walks up to each person in the room instead
        # of asking the target to come. Between candidates it returns to this
        # safe pose. Visited people are remembered by MAP POSITION (primary —
        # works even when face recognition sees no face) and by face
        # (best-effort backup that survives the person moving around).
        s, safe_pose = self.subtask_manager.nav.get_current_pose()
        visited_names = set()
        visited_points = []

        def already_visited(p):
            return any(
                (p.point.x - v.point.x) ** 2 + (p.point.y - v.point.y) ** 2
                < FIND_PERSON_VISITED_RADIUS**2
                for v in visited_points
            )

        for round_idx in range(FIND_PERSON_MAX_ROUNDS):
            # Re-scan every round: people move, and fresh detections avoid
            # approaching stale camera-frame points after the base moved.
            persons = self._scan_persons_in_room()
            candidates = [p for p in persons if not already_visited(p)]
            if not candidates:
                break
            map_point = candidates[0]
            # Mark BEFORE approaching: a failed or partial approach must not
            # burn later rounds re-trying the same detection.
            visited_points.append(map_point)

            if not self._approach_with_arm(map_point):
                continue

            status, name = self.subtask_manager.vision.get_person_name(fresh_timeout=4.0)
            self.subtask_manager.hri.node.get_logger().info(f"Recognized: {name}.")
            if name:
                if name.lower() == target:
                    self.subtask_manager.hri.say("There you are, " + command.name + ".")
                    return Status.EXECUTION_SUCCESS, f"found {command.name}"
                if name.lower() in visited_names:
                    # Already met this person this search — move on.
                    self._back_to_safe(safe_pose)
                    continue

            # Closed yes/no question instead of asking for the name: name
            # extraction broke on noisy transcriptions ("Yeah, I'll do."),
            # while the yes/no keyword spotter is robust in competition noise.
            s, answer = self.subtask_manager.hri.confirm(
                f"Hi, I'm Frida. Are you {command.name}?", use_keyword=True, retries=2
            )
            if answer == "yes":
                # Save the face so later subtasks can recognize the target.
                self.subtask_manager.vision.save_face_name(command.name)
                self.subtask_manager.hri.say("Nice to meet you, " + command.name + ".")
                return Status.EXECUTION_SUCCESS, f"found {command.name}"

            # "no" or no answer: temp-save the face as backup dedup (fails
            # harmlessly when no face is visible — visited_points covers it).
            guest = name if name else f"guest {round_idx}"
            if not name:
                self.subtask_manager.vision.save_face_name(guest)
            visited_names.add(guest.lower())
            self.subtask_manager.hri.say("Sorry, I am looking for " + command.name + ".")
            self._back_to_safe(safe_pose)

        # Fallback (old behavior): ask the target to come to the robot.
        self.subtask_manager.hri.say(
            f"{command.name}, if you are here, please come and stand in front of me."
        )
        time.sleep(5)
        s, answer = self.subtask_manager.hri.confirm(
            f"Are you {command.name}?", use_keyword=True, retries=2
        )
        if answer == "yes":
            self.subtask_manager.vision.save_face_name(command.name)
            self.subtask_manager.hri.say("Nice to meet you, " + command.name + ".")
            return Status.EXECUTION_SUCCESS, f"found {command.name}"
        return Status.TARGET_NOT_FOUND, "person not found"

    def _back_to_safe(self, safe_pose):
        """Return to the pose where the person search started (arm tucked for
        the drive, then front_stare to rescan for the next candidate)."""
        if safe_pose is not None:
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self.subtask_manager.nav.move_to_pose(safe_pose)
        self.subtask_manager.manipulation.move_to_position("front_stare")

    def _scan_persons_in_room(self):
        """Localize the people in the room via count_person: gpsr_commands
        detects persons with the GENERIC COCO model (YoloDetect class 0 — the
        detect_objects path runs the competition-object model, which has NO
        person class), filters them to the robot's current room polygon and
        deprojects each to a camera-frame 3D point. Those points are projected
        to the map HERE, while the camera still holds the scan pose, so they
        stay valid after the arm moves. Pans through self.pan_angles when
        nobody is in the front view. Returns map-frame PointStamped list,
        nearest person first."""
        for pan in [0] + list(self.pan_angles):
            if pan:
                self.subtask_manager.manipulation.pan_to(pan)
                time.sleep(1.0)
            s, _count = self.subtask_manager.vision.count_person()
            points = list(self.subtask_manager.vision.last_person_points)
            points.sort(key=lambda p: p.point.x**2 + p.point.y**2 + p.point.z**2)
            map_points = []
            for p in points:
                xy = self.subtask_manager.nav._resolve_map_xy(p)
                if xy is None:
                    continue  # unprojectable -> can't be approached reliably
                mp = PointStamped()
                mp.header.frame_id = "map"
                mp.point.x, mp.point.y = xy
                map_points.append(mp)
            if map_points:
                if pan:
                    self.subtask_manager.manipulation.pan_to(0)
                return map_points
        self.subtask_manager.manipulation.pan_to(0)
        return []
