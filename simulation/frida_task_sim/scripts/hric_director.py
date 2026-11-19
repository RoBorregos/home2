#!/usr/bin/env python3
"""HRIC director: moves the guests and the bag, stands in for people perception
and judges what the robot does with them from ground truth."""

import json
import math
import re

import numpy as np
import rclpy
import tf2_ros
from frida_constants.hri_constants import (
    DOORBELL_ARMED_TOPIC,
    EI_DETECTION_TOPIC,
    TASK_STEP_TOPIC,
)
from frida_constants.vision_constants import (
    CAMERA_FRAME,
    CAMERA_ROTATION_TOPIC,
    CHECK_PERSON_TOPIC,
    DETECT_HAND_SERVICE,
    FIND_SEAT_TOPIC,
    FOLLOW_BY_TOPIC,
    FOLLOW_TOPIC,
    PERSON_LIST_TOPIC,
    QUERY_TOPIC,
    SAVE_NAME_TOPIC,
    SET_TARGET_TOPIC,
)
from frida_interfaces.msg import Person, PersonList
from frida_interfaces.srv import DetectHand, DetectPerson, FindSeat, Query, SaveName
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, Int16, String
from std_srvs.srv import SetBool

from frida_task_sim import catalog, people, scenario as scenarios, venue as venues
from frida_task_sim.gz import PoseStream
from frida_task_sim.world import WORLD_NAME

NOTES_TOPIC = "/sim/referee/notes"
ROBOT_MODEL = "frida"
# Half field of view of the camera as tangents (horizontal, vertical)
TAN_H, TAN_V = 1.21, 0.68
MAX_RANGE = 4.0
# Seat angle reported at the image edge (deg), as the real seat finder does
SEAT_MAX_DEGREE = 50.0
# The robot looks at someone when its camera axis is this close to their head (deg)
GAZE_TOLERANCE = 15.0
GAZE_SHARE = 0.7
SEAT_TOLERANCE = 20.0
PACE_AMPLITUDE = 0.20
PACE_PERIOD = 8.0
FOLLOW_DISTANCE = 1.1
DOORBELL_DELAY = 2.0
POINTING_TIME = 3.0
# When the gaze is checked after an introduction sentence starts (s)
INTRO_GAZE_DELAY = 4.0
# Finger pads sit this far along the finger link Z (as in grasp_attach.py)
PAD_OFFSET = 0.06

INTRO_FIRST = re.compile(r"introduce you to (.+?), their favorite drink is (.+?)\.?$")
INTRO_SECOND = re.compile(r"This is (.+?) and their favorite drink is (.+?)\.?$")
DESCRIPTION = re.compile(r"is already in the living room\.(.*)$", re.DOTALL)


class Director(Node):
    def __init__(self):
        super().__init__("hric_director")
        self.scenario = scenarios.load(
            self.declare_parameter("scenario_file", "").value
        )
        self.cast = {p["id"]: dict(p) for p in self.scenario["people"]}
        self.guests = [self.cast["guest_1"], self.cast["guest_2"]]
        self.seats = self.scenario["seats"]
        cfg = venues.load(self.scenario["venue"])["hric"]
        self.door_spot = cfg["door_spot"]
        self.poses = PoseStream(WORLD_NAME, names=[ROBOT_MODEL, "bag"])
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.step = ""
        self.arrived = 0
        # Where each guest is: parked, door, following or seated
        self.where = {g["id"]: "parked" for g in self.guests}
        self.faces: dict[str, str] = {}
        self.target = "area"
        # parked, hand (on the guest's hand), give (held into the gripper) or robot
        self.bag = "parked"
        self.hand_reached = False
        self.gaze = [0, 0]
        self.confirmations = 0
        self.one_shots = []
        self.rotation = 0

        group = ReentrantCallbackGroup()
        self.notes = self.create_publisher(String, NOTES_TOPIC, 50)
        self.door_event = self.create_publisher(String, EI_DETECTION_TOPIC, 10)
        self.face_pub = self.create_publisher(Point, FOLLOW_TOPIC, 10)
        self.person_list_pub = self.create_publisher(PersonList, PERSON_LIST_TOPIC, 10)
        self.create_subscription(String, "/speech/text_spoken", self._on_speech, 50)
        self.create_subscription(String, TASK_STEP_TOPIC, self._on_step, 10)
        self.create_subscription(Bool, DOORBELL_ARMED_TOPIC, self._on_armed, 10)
        self.create_subscription(
            Int16,
            CAMERA_ROTATION_TOPIC,
            lambda m: setattr(self, "rotation", m.data),
            10,
        )
        self.create_subscription(
            String, "/sim/grasp_attached_object", self._on_grasp, 10
        )
        for srv_type, name, handler in (
            (DetectPerson, CHECK_PERSON_TOPIC, self._detect_person),
            (SaveName, SAVE_NAME_TOPIC, self._save_name),
            (SaveName, FOLLOW_BY_TOPIC, self._follow_by_name),
            (FindSeat, FIND_SEAT_TOPIC, self._find_seat),
            (DetectHand, DETECT_HAND_SERVICE, self._detect_hand),
            (Query, QUERY_TOPIC, self._query),
            (SetBool, SET_TARGET_TOPIC, self._track),
        ):
            self.create_service(srv_type, name, handler, callback_group=group)
        self.create_timer(0.1, self._scene)
        self.create_timer(0.02, self._hold_bag, callback_group=group)
        # Wall clock: face following drops data older than 0.1 s of real time
        self.create_timer(
            0.05, self._publish_faces, callback_group=group, clock=rclpy.clock.Clock()
        )
        self.get_logger().info("HRIC director ready")

    # ── Notes to the referee ──

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def note(self, kind: str, **data):
        self.notes.publish(String(data=json.dumps({"type": kind, **data})))

    def judge(self, row: str, good: bool, title: str, detail: str):
        self.note("judge", row=row, good=good, title=title, detail=detail)

    def later(self, delay: float, action):
        """Run action once after delay seconds of sim time."""
        self.one_shots.append((self.now() + delay, action))

    # ── Geometry ──

    def head(self, person: dict) -> np.ndarray:
        x, y, z, _ = person["pose"]
        return np.array([x, y, z + people.HEAD_Z])

    def hand(self, person: dict) -> np.ndarray:
        x, y, z, yaw = person["pose"]
        offset = Rotation.from_euler("z", yaw).apply(people.HAND)
        return np.array([x, y, z]) + offset

    def camera(self):
        """(position, rotation) of the camera in the map, or None."""
        try:
            tf = self.tf_buffer.lookup_transform("map", CAMERA_FRAME, Time())
        except tf2_ros.TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])

    def in_camera(self, point: np.ndarray):
        """Point in the optical frame (x right, y down, z forward), or None."""
        camera = self.camera()
        return None if camera is None else camera[1].inv().apply(point - camera[0])

    def view(self, point: np.ndarray):
        """Normalized image offsets (right, up) of a visible point, or None."""
        p = self.in_camera(point)
        if p is None or p[2] < 0.2 or p[2] > MAX_RANGE:
            return None
        right, up = p[0] / p[2] / TAN_H, -p[1] / p[2] / TAN_V
        if self.rotation == 180:
            # The vision nodes turn the image upright when the camera hangs upside down
            right, up = -right, -up
        return (right, up) if abs(right) <= 1.0 and abs(up) <= 1.0 else None

    def gaze_error(self, point: np.ndarray) -> float:
        """Horizontal angle between the camera axis and a point (deg)."""
        p = self.in_camera(point)
        return 180.0 if p is None else abs(math.degrees(math.atan2(p[0], p[2])))

    def visible_people(self) -> list[dict]:
        """People in view, nearest first."""
        seen = []
        for person in self.cast.values():
            p = self.in_camera(self.head(person))
            if p is not None and self.view(self.head(person)) is not None:
                seen.append((p[2], person))
        return [person for _, person in sorted(seen, key=lambda s: s[0])]

    def move(self, person: dict, x: float, y: float, z: float, yaw: float):
        person["pose"] = [x, y, z, yaw]
        self.poses.set_pose(WORLD_NAME, person["id"], x, y, z, yaw)

    def occupied(self) -> set[int]:
        return {p["seat"] for p in self.cast.values() if p["seat"] is not None}

    # ── Task signals ──

    def _on_step(self, msg: String):
        self.step = msg.data

    def _on_armed(self, msg: Bool):
        if msg.data and self.arrived < len(self.guests):
            guest = self.guests[self.arrived]
            if self.where[guest["id"]] == "parked":
                self.where[guest["id"]] = "door"
                self.later(DOORBELL_DELAY, lambda: self._ring(guest))

    def _ring(self, guest: dict):
        self.move(guest, *self.door_spot, 0.0, -math.pi / 2)
        self.door_event.publish(String(data=json.dumps({"keyword": "doorbell"})))
        self.note("scene", detail=f"the doorbell rings, {guest['name']} is at the door")

    def _on_grasp(self, msg: String):
        if msg.data == "bag" and self.bag == "give":
            self.bag = "robot"
            self.judge("bag_handover", True, "BAG", "the robot holds the bag")
            if not self.hand_reached:
                self.judge(
                    "bag_on_structure",
                    False,
                    "BAG",
                    "the guest had to put it in the gripper",
                )
        elif not msg.data and self.bag == "robot":
            self.bag = "released"
            self.note("scene", detail="the robot lets go of the bag")
            # A real bag slips out of the open gripper; the rigid one would sit on it
            bag = self.poses.get("bag")
            if bag is not None:
                self.poses.set_pose(WORLD_NAME, "bag", bag[0][0], bag[0][1], 0.01)

    def _on_speech(self, msg: String):
        data = json.loads(msg.data)
        if data["who"] != "robot":
            return
        text = data["text"]
        guest = self.guests[min(self.arrived, len(self.guests) - 1)]
        if text.startswith("I heard someone at the door"):
            self.judge("doorbell", True, "DOORBELL", "heard and announced")
        elif text.startswith("Did you say"):
            self.confirmations += 1
        elif text.startswith("Nice to meet you"):
            self._greeted(guest)
        elif "Please follow me" in text and self.where[guest["id"]] == "door":
            self.where[guest["id"]] = "following"
        elif text.startswith("I see you brought a bag"):
            self.bag = "hand"
            self.note("scene", detail=f"{guest['name']} holds the bag out")
        elif text.startswith("Please hand me the bag"):
            self.bag, self.hand_reached = "give", True
        elif text.startswith("Place the bag in my gripper"):
            self.bag, self.hand_reached = "give", False
        elif text.startswith("Please take a seat"):
            self.later(POINTING_TIME, lambda: self._seat(guest))
        description = DESCRIPTION.search(text)
        if description:
            colour = self.guests[0]["colour"]
            told = colour in description.group(1).lower()
            self.judge(
                "attribute_correct" if told else "",
                told,
                "ATTRIBUTE",
                f"{'told' if told else 'did not tell'} the {colour} shirt of {self.guests[0]['name']}",
            )
        for pattern, about, listener in (
            (INTRO_FIRST, self.guests[0], self.guests[1]),
            (INTRO_SECOND, self.guests[1], self.guests[0]),
        ):
            intro = pattern.search(text)
            if intro:
                self._introduced(about, listener, intro.group(1), intro.group(2))

    def _greeted(self, guest: dict):
        looked, total = self.gaze
        share = looked / total if total else 0.0
        good = share >= GAZE_SHARE
        self.judge(
            "look_at_guest" if good else "",
            good,
            "GAZE",
            f"looked at {guest['name']} {share:.0%} of the conversation",
        )
        for _ in range(max(0, 2 - self.confirmations)):
            self.judge("no_confirmation", True, "QUESTIONS", "no confirmation asked")
        if self.confirmations:
            self.judge(
                "",
                False,
                "QUESTIONS",
                f"{self.confirmations} confirmation question(s)",
            )
        self.gaze, self.confirmations = [0, 0], 0

    def _seat(self, guest: dict):
        """The guest takes the free seat closest to where the camera points."""
        free = [i for i in range(len(self.seats)) if i not in self.occupied()]
        errors = {
            i: self.gaze_error(np.array([*self.seats[i], people.SEAT_HEIGHT]))
            for i in free
        }
        seat = min(errors, key=errors.get)
        good = errors[seat] <= SEAT_TOLERANCE
        self.judge(
            "offer_seat" if good else "",
            good,
            "SEAT",
            f"pointed {errors[seat]:.0f} deg away from free seat {seat + 1}",
        )
        guest["seat"] = seat
        self.where[guest["id"]] = "seated"
        self.move(guest, *self.seats[seat], -people.SEATED_DROP, 0.0)
        self.arrived += 1

    def _introduced(self, about: dict, listener: dict, name: str, drink: str):
        for said, truth, what in (
            (name, about["name"], "name"),
            (drink, about["drink"], "drink"),
        ):
            good = said.strip().lower() == truth.lower()
            self.judge(
                "intro_info" if good else "wrong_info",
                good,
                "INTRODUCTION",
                f"said the {what} {said} ({'right' if good else 'it is ' + truth})",
            )

        def check():
            error = self.gaze_error(self.head(listener))
            good = error <= SEAT_TOLERANCE
            self.judge(
                "intro_gaze" if good else "",
                good,
                "GAZE",
                f"{error:.0f} deg away from {listener['name']} while introducing {about['name']}",
            )

        self.later(INTRO_GAZE_DELAY, check)

    # ── Scene ──

    def _scene(self):
        now = self.now()
        due = [item for item in self.one_shots if item[0] <= now]
        self.one_shots = [item for item in self.one_shots if item[0] > now]
        for _, action in due:
            action()
        robot = self.poses.get(ROBOT_MODEL)
        for guest in self.guests:
            where = self.where[guest["id"]]
            if where == "door" and guest["pose"][0] < 10.0 and self.bag != "give":
                talking = self.step in ("greeting", "save_face")
                sway = PACE_AMPLITUDE * math.sin(2 * math.pi * now / PACE_PERIOD)
                x = self.door_spot[0] + (sway if talking else 0.0)
                self.move(guest, x, self.door_spot[1], 0.0, -math.pi / 2)
                if self.step == "greeting":
                    self.gaze[1] += 1
                    error = self.gaze_error(self.head(guest))
                    self.gaze[0] += error <= GAZE_TOLERANCE
            elif where == "following" and robot is not None:
                heading = robot[1].as_euler("zyx")[0]
                x = robot[0][0] - FOLLOW_DISTANCE * math.cos(heading)
                y = robot[0][1] - FOLLOW_DISTANCE * math.sin(heading)
                self.move(guest, x, y, 0.0, heading)

    def _hold_bag(self):
        """Keep the bag in the guest's hand, then in the gripper until it closes."""
        holder = self.guests[1]
        if self.bag == "hand":
            x, y, z = self.hand(holder)
            self.poses.set_pose(WORLD_NAME, "bag", x, y, z)
        elif self.bag == "give":
            grasp = self.grasp_point()
            if grasp is not None:
                z = grasp[2] - catalog.height("bag") / 2
                self.poses.set_pose(WORLD_NAME, "bag", grasp[0], grasp[1], z)

    def grasp_point(self):
        """Point between the finger pads in the world, as grasp_attach.py computes it."""
        robot = self.poses.get(ROBOT_MODEL)
        points = []
        for finger in ("right_finger", "left_finger"):
            try:
                tf = self.tf_buffer.lookup_transform("base_link", finger, Time())
            except tf2_ros.TransformException:
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            pad = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([0, 0, PAD_OFFSET])
            points.append(np.array([t.x, t.y, t.z]) + pad)
        if robot is None:
            return None
        return robot[0] + robot[1].apply((points[0] + points[1]) / 2.0)

    # ── People perception ──

    def _publish_faces(self):
        seen = self.visible_people()
        names = {pid: name for name, pid in self.faces.items()}
        msg = PersonList()
        for person in seen:
            msg.list.append(Person(name=names.get(person["id"], "Unknown")))
        self.person_list_pub.publish(msg)
        wanted = self.faces.get(self.target)
        match = next((p for p in seen if p["id"] == wanted), None)
        if match is None and self.target == "area" and seen:
            match = seen[0]
        if match is not None:
            right, up = self.view(self.head(match))
            self.face_pub.publish(Point(x=right, y=up))

    def _detect_person(self, request, response):
        response.success = bool(self.visible_people())
        return response

    def _save_name(self, request, response):
        seen = self.visible_people()
        if seen:
            self.faces[request.name] = seen[0]["id"]
        response.success = bool(seen)
        return response

    def _follow_by_name(self, request, response):
        self.target = request.name
        response.success = True
        return response

    def _find_seat(self, request, response):
        best = None
        for i, (x, y) in enumerate(self.seats):
            # A seat counts as seen when its column is in the image, whatever the tilt
            p = self.in_camera(np.array([x, y, people.SEAT_HEIGHT]))
            if i in self.occupied() or p is None or p[2] < 0.2:
                continue
            right = p[0] / p[2] / TAN_H * (-1 if self.rotation == 180 else 1)
            if abs(right) <= 1.0 and (best is None or abs(right) < abs(best)):
                best = right
        response.success = best is not None
        response.angle = 0.0 if best is None else best * SEAT_MAX_DEGREE
        return response

    def _detect_hand(self, request, response):
        hand = self.hand(self.guests[1])
        point = self.in_camera(hand)
        response.success = self.bag == "hand" and self.view(hand) is not None
        if response.success:
            response.point.header.frame_id = CAMERA_FRAME
            response.point.header.stamp = self.get_clock().now().to_msg()
            p = response.point.point
            p.x, p.y, p.z = float(point[0]), float(point[1]), float(point[2])
        return response

    def _query(self, request, response):
        seen = self.visible_people()
        person = seen[0] if seen else self.guests[0]
        clothes = "clothes" in request.query
        response.result = f"wearing a {person['colour']} shirt" if clothes else "0"
        response.success = True
        return response

    def _track(self, request, response):
        response.success = False
        response.message = "following a person is not simulated"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Director()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
