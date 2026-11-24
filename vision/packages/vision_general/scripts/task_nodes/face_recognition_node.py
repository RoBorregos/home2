#!/usr/bin/env python3
"""Face recognition ROS2 node. Model logic lives in models.face_recognition."""

import time
from collections import deque

import rclpy
import rclpy.duration
import tqdm
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Point
from std_msgs.msg import String

import cv2
import numpy as np

from frida_constants.vision_constants import (
    FACE_RECOGNITION_IMAGE,
    FOLLOW_BY_TOPIC,
    FOLLOW_TOPIC,
    IMAGE_ORIENTED_TOPIC,
    IS_TALKING_TIMEOUT,
    IS_TALKING_TOPIC,
    PERSON_LIST_TOPIC,
    PERSON_NAME_TOPIC,
    SAVE_NAME_TOPIC,
    TALKING_CROP_MARGIN,
    TALKING_CROP_MAX_SIZE,
    TALKING_MIN_FACE_WIDTH,
    TALKING_MIN_NEW_FRAMES,
    TALKING_REQUEST_FRAMES,
    TALKING_TRACK_GAP_SEC,
)

from frida_interfaces.msg import Person, PersonList
from frida_interfaces.srv import IsTalking, SaveName
from models.face_recognition import FaceModel, TRACK_THRESHOLD
from utils.debug_pub import DebugImagePublisher
from vision_runtime import VisionRuntime, spin

MAX_DEGREE = 1
PACKAGE_PATH = get_package_share_directory("vision_general")
KNOWN_FACES_PATH = PACKAGE_PATH + "/utils/known_faces"


class FaceRecognition(VisionRuntime):
    def __init__(self):
        super().__init__(
            "face_recognition",
            image_topic=IMAGE_ORIENTED_TOPIC,
            use_depth=True,
            use_camera_info=True,
            active_name="face_recognition",
            # Inference runs per frame in the image callback, one frame at a time.
            image_callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.pbar = tqdm.tqdm(total=2)

        self.create_service(
            SaveName,
            SAVE_NAME_TOPIC,
            self.new_name_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            SaveName,
            FOLLOW_BY_TOPIC,
            self.follow_by_name_callback,
            callback_group=self.callback_group,
        )

        self.is_talking_client = self.create_client(
            IsTalking,
            IS_TALKING_TOPIC,
            callback_group=self.callback_group,
        )

        self.follow_publisher = self.create_publisher(Point, FOLLOW_TOPIC, 10)
        self.view_pub = DebugImagePublisher(
            self, FACE_RECOGNITION_IMAGE, "face_recognition"
        )
        self.name_publisher = self.create_publisher(String, PERSON_NAME_TOPIC, 10)
        self.person_list_publisher = self.create_publisher(
            PersonList, PERSON_LIST_TOPIC, 10
        )

        self.verbose = self.declare_parameter("verbose", True)
        self.setup()

    def setup(self):
        default_name = self.declare_parameter("default_name", "ale").value
        self.face_model = FaceModel(KNOWN_FACES_PATH, default_name)
        self.get_logger().info("Loading InsightFace model…")
        for filename in self.face_model.load():
            self.get_logger().warn(f"No face found in {filename}, skipping.")
        self.pbar.update(1)

        self.new_name = ""
        self.annotated_frame = []
        self.image_view = None
        self.prev_faces: list = []
        self.curr_faces: list = []
        self.follow_name = "area"
        # Idle until the task manager activates it, unless launched with start_active.
        self.active = self.declare_parameter("start_active", False).value
        self.id = None
        self.processing_id = rclpy.duration.Infinite
        self.talk_tracks: list = []  # per-face crop buffers
        self.talk_pending: list = []  # (track, future, deadline, gen)
        self.last_talker = None  # {name, x, y, bbox}
        self.talker_gen = 0  # bumped by follow_by to ignore older answers
        self.area_target = None  # (x, y) followed last frame in "area" mode

        self.pbar.update(1)
        self.get_logger().info("Face Recognition Ready")

    # ── ROS callbacks ──

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def new_name_callback(self, req, res):
        self.get_logger().info("Executing service new face")
        self.new_name = req.name
        res.success = len(self.curr_faces) > 0
        if not res.success:
            self.get_logger().info("No face detected")
        return res

    def follow_by_name_callback(self, req, res):
        self.get_logger().info("Executing service follow by")
        self.follow_name = req.name
        self.last_talker = None
        self.talker_gen += 1
        res.success = len(self.curr_faces) > 0
        if not res.success:
            self.get_logger().info("No face detected")
        return res

    # ── Talking detection ──
    # Each detected face's crop goes into a buffer; full buffers are sent to
    # is_talking asynchronously and the answers are collected on later frames.

    def _add_talk_crop(self, frame, face, now):
        x1, y1, x2, y2 = face["bbox"]
        if x2 - x1 < TALKING_MIN_FACE_WIDTH:
            return
        threshold = max(TRACK_THRESHOLD, (x2 - x1) / 2)
        track = next(
            (
                t
                for t in self.talk_tracks
                if t["seen"] != now
                and abs(t["x"] - face["x"]) < threshold
                and abs(t["y"] - face["y"]) < threshold
            ),
            None,
        )
        if track is None:
            track = {"crops": deque(maxlen=TALKING_REQUEST_FRAMES), "new": 0}
            track["talking"] = False
            self.talk_tracks.append(track)
        elif now - track["seen"] > TALKING_TRACK_GAP_SEC:
            track["crops"].clear()  # crops must be a continuous sequence
        track.update(_talker_fields(face), seen=now)

        mw = int((x2 - x1) * TALKING_CROP_MARGIN)
        mh = int((y2 - y1) * TALKING_CROP_MARGIN)
        crop = frame[max(y1 - mh, 0) : y2 + mh, max(x1 - mw, 0) : x2 + mw]
        scale = min(1.0, TALKING_CROP_MAX_SIZE / max(crop.shape[:2]))
        track["crops"].append(
            cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        )
        track["new"] += 1

    def _update_talking(self, now):
        """Collect is_talking answers, drop lost faces and send full buffers."""
        for entry in list(self.talk_pending):
            track, future, deadline, gen = entry
            if not future.done() and now <= deadline:
                continue
            self.talk_pending.remove(entry)
            if not future.done():
                self.is_talking_client.remove_pending_request(future)
                continue
            track["talking"] = future.result().is_talking
            if track["talking"] and gen == self.talker_gen:
                self._set_last_talker(track)

        self.talk_tracks = [
            t for t in self.talk_tracks if now - t["seen"] <= TALKING_TRACK_GAP_SEC
        ]
        busy = [p[0] for p in self.talk_pending]
        for track in self.talk_tracks:
            if (
                len(track["crops"]) < TALKING_REQUEST_FRAMES
                or track["new"] < TALKING_MIN_NEW_FRAMES
                or any(track is b for b in busy)
            ):
                continue
            request = IsTalking.Request()
            request.frames = [
                self.bridge.cv2_to_imgmsg(c, "bgr8") for c in track["crops"]
            ]
            track["new"] = 0
            # is_talking answers one request at a time.
            deadline = now + IS_TALKING_TIMEOUT * (len(self.talk_pending) + 1)
            future = self.is_talking_client.call_async(request)
            self.talk_pending.append((track, future, deadline, self.talker_gen))

    def _reset_talking(self):
        for _, future, _, _ in self.talk_pending:
            if not future.done():
                self.is_talking_client.remove_pending_request(future)
        self.talk_tracks, self.talk_pending = [], []

    def _set_last_talker(self, track):
        prev = self.last_talker
        if prev is None or not _same_person(prev, track):
            # Keep the current talker while its latest answer still says talking.
            if prev and any(
                t["talking"] and _same_person(prev, t) for t in self.talk_tracks
            ):
                return
            self.get_logger().info(f"Last talker: {track['name']}")
        self.last_talker = _talker_fields(track)

    # ── Main loop ──

    def _image_callback(self, msg):
        super()._image_callback(msg)
        self.run()

    def run(self):
        if not self.active:
            self._reset_talking()
            return
        if self.image is None:
            return
        # Frame id is the source stamp; skip frames already processed.
        self.id = self.image_header.stamp if self.image_header is not None else None
        if self.id is not None and self.id == self.processing_id:
            return
        self.annotated_frame = self.image
        self._run_inference(self.image)

    def _run_inference(self, frame):
        self.processing_id = self.id
        annotated = frame.copy()
        height, width = frame.shape[:2]
        center = [width / 2, height / 2]
        talking_enabled = (
            self.follow_name == "area" and self.is_talking_client.service_is_ready()
        )
        now = time.time()

        faces = []
        face_list = PersonList()
        for ins_face in self.face_model.detect(frame):
            x1, y1, x2, y2 = ins_face.bbox.astype(int)
            box = (
                max(x1 - TRACK_THRESHOLD, 0),
                max(y1 - TRACK_THRESHOLD, 0),
                min(x2 + TRACK_THRESHOLD, width),
                min(y2 + TRACK_THRESHOLD, height),
            )
            face = {"x": (x1 + x2) / 2.0, "y": (y1 + y2) / 2.0}
            face.update(bbox=(x1, y1, x2, y2), box=box)
            prev = _nearest(self.prev_faces, face["x"], face["y"], TRACK_THRESHOLD)
            face["name"] = prev["name"] if prev else "Unknown"
            if face["name"] == "Unknown":
                face["name"] = self.face_model.match(
                    ins_face.embedding.astype(np.float32)
                )
            faces.append(face)
            if talking_enabled:
                self._add_talk_crop(frame, face, now)

            xc, yc = _box_center(box)
            p = Person()
            p.name = face["name"]
            p.x = int((xc - center[0]) * MAX_DEGREE / center[0])
            p.y = int((center[1] - yc) * MAX_DEGREE / center[1])
            face_list.list.append(p)
            _draw_face(
                annotated, box, face["name"], (255, 0, 0) if prev else (0, 0, 255)
            )

        if talking_enabled:
            self._update_talking(now)
        else:
            self._reset_talking()

        largest = max(
            faces,
            key=lambda f: (f["box"][2] - f["box"][0]) * (f["box"][3] - f["box"][1]),
            default=None,
        )
        if largest and self.new_name:
            left, top, right, bottom = largest["box"]
            if self.face_model.enroll(self.new_name, frame[top:bottom, left:right]):
                self.success(f"{self.new_name} face enrolled")
            else:
                self.get_logger().warn(
                    f"Could not enroll {self.new_name}: no face found in the saved crop"
                )
            largest["name"] = self.new_name
            face_list.list[faces.index(largest)].name = self.new_name
            self.new_name = ""

        # Target: specific name > last talker > last followed face > largest face.
        if self.follow_name != "area":
            target = next((f for f in faces if f["name"] == self.follow_name), None)
            self.area_target = None
        else:
            target = self._find_talker(faces)
            if target:
                left, top = target["box"][:2]
                cv2.putText(
                    annotated,
                    "TALKING",
                    (left + 6, max(top - 8, 20)),
                    cv2.FONT_HERSHEY_DUPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
            elif self.area_target:
                target = _nearest(faces, *self.area_target)
            target = target or largest
            self.area_target = (target["x"], target["y"]) if target else None

        self.curr_faces = self.prev_faces = faces
        self.annotated_frame = annotated
        self.person_list_publisher.publish(face_list)
        if target:
            self._publish_follow_face(
                *_box_center(target["box"]), target["name"], center
            )
        else:
            self.name_publisher.publish(String(data=""))
        self.view_pub.publish(annotated)

    def _find_talker(self, faces):
        """Face matching the last talker (by name, else by position); refreshes it."""
        talker = self.last_talker
        if talker is None:
            return None
        face = None
        if talker["name"] != "Unknown":
            face = next((f for f in faces if f["name"] == talker["name"]), None)
        face = face or _nearest(faces, talker["x"], talker["y"])
        if face:
            # Follow the talker while it moves; keep its name if now "Unknown".
            self.last_talker = _talker_fields(face)
            if face["name"] == "Unknown":
                self.last_talker["name"] = talker["name"]
        return face

    def _publish_follow_face(self, xc, yc, name, center):
        difx = 0.0 if xc == 0 else xc - center[0]
        dify = 0.0 if yc == 0 else center[1] - yc
        target = Point()
        target.x = difx * MAX_DEGREE / center[0]
        target.y = dify * MAX_DEGREE / center[1]
        self.follow_publisher.publish(target)
        self.name_publisher.publish(String(data=name))


def _talker_fields(face):
    return {k: face[k] for k in ("name", "x", "y", "bbox")}


def _box_center(box):
    left, top, right, bottom = box
    return (left + right) / 2.0, (top + bottom) / 2.0


def _nearest(faces, x, y, threshold=None):
    """Closest face to (x, y) within threshold (default: the face's own width)."""

    def near(f):
        th = threshold or max(TRACK_THRESHOLD, f["bbox"][2] - f["bbox"][0])
        return abs(f["x"] - x) < th and abs(f["y"] - y) < th

    return min(
        (f for f in faces if near(f)),
        key=lambda f: abs(f["x"] - x) + abs(f["y"] - y),
        default=None,
    )


def _same_person(a, b):
    # Unknown faces all share a name, so also compare position.
    return a["name"] == b["name"] and (
        a["name"] != "Unknown"
        or _nearest([b], a["x"], a["y"], TRACK_THRESHOLD) is not None
    )


def _draw_face(img, box, name, color):
    left, top, right, bottom = box
    cv2.rectangle(img, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
    cv2.rectangle(img, (left, top), (right, bottom), color, 2)
    cv2.putText(
        img,
        name,
        (left + 6, bottom - 6),
        cv2.FONT_HERSHEY_DUPLEX,
        1.0,
        (255, 255, 255),
        1,
    )


def main(args=None):
    rclpy.init(args=args)
    spin(FaceRecognition(), threads=5)


if __name__ == "__main__":
    main()
