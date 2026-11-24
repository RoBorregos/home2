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
    IS_TALKING_TOPIC,
    PERSON_LIST_TOPIC,
    PERSON_NAME_TOPIC,
    SAVE_NAME_TOPIC,
    TALKING_REQUEST_FRAMES,
)

from frida_interfaces.msg import Person, PersonList
from frida_interfaces.srv import IsTalking, SaveName
from models.face_recognition import FaceModel, TRACK_THRESHOLD
from utils.debug_pub import DebugImagePublisher
from vision_runtime import VisionRuntime, spin

MAX_DEGREE = 1
TALKING_CROP_MARGIN = 0.25
# Longest crop side sent to is_talking; keeps a request under the 8 MB iceoryx chunk.
TALKING_CROP_MAX_SIZE = 192
# Faces narrower than this (px) are background faces: MediaPipe rarely finds their
# landmarks, and their requests only delay the answers for the people talking.
TALKING_MIN_FACE_WIDTH = 30
# A track not seen for this long is dropped, and its crops restart if it reappears.
TALKING_TRACK_GAP_SEC = 0.5
# New crops a face needs before its window is sent to is_talking again.
TALKING_MIN_NEW_FRAMES = 15
IS_TALKING_TIMEOUT = 5.0
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
            # Inference runs inside the image callback; one frame at a time, and
            # frames that arrive meanwhile are replaced by the newest (QoS depth 1).
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
        # Per-face buffers of recent crops, filled from run()'s own detections.
        self.talk_tracks: list = []
        # In-flight is_talking requests: (track, future, deadline, gen).
        self.talk_pending: list = []
        # Last person who talked ({name, x, y, bbox}); followed when follow_name == "area".
        self.last_talker = None
        # Bumped by follow_by so answers to older requests are ignored.
        self.talker_gen = 0
        # Face followed last tick in "area" mode; kept instead of re-picking the largest.
        self.area_target = None

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
    # run() already detects every face on every frame; each face's crop goes into
    # a buffer, and when a buffer has enough new crops it is sent to is_talking
    # asynchronously. Answers are collected on the next frames.

    def _talking_enabled(self):
        return self.follow_name == "area" and self.is_talking_client.service_is_ready()

    def _add_talk_crop(self, frame, bbox, name, now):
        """Append this face's crop to its track (the nearest one, or a new one)."""
        x1, y1, x2, y2 = bbox
        if x2 - x1 < TALKING_MIN_FACE_WIDTH:
            return
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        # Scale with face size so a moving close face stays one track.
        threshold = max(TRACK_THRESHOLD, (x2 - x1) / 2)
        track = next(
            (
                t
                for t in self.talk_tracks
                if t["seen"] != now
                and abs(t["x"] - cx) < threshold
                and abs(t["y"] - cy) < threshold
            ),
            None,
        )
        if track is None:
            track = {
                "crops": deque(maxlen=TALKING_REQUEST_FRAMES),
                "new": 0,
                "talking": False,
            }
            self.talk_tracks.append(track)
        elif now - track["seen"] > TALKING_TRACK_GAP_SEC:
            # The crops must be a continuous sequence for MouthActivity.
            track["crops"].clear()
        track.update(name=name, x=cx, y=cy, bbox=(x1, y1, x2, y2), seen=now)

        height, width = frame.shape[:2]
        mw = int((x2 - x1) * TALKING_CROP_MARGIN)
        mh = int((y2 - y1) * TALKING_CROP_MARGIN)
        crop = frame[
            max(y1 - mh, 0) : min(y2 + mh, height),
            max(x1 - mw, 0) : min(x2 + mw, width),
        ]
        scale = TALKING_CROP_MAX_SIZE / max(*crop.shape[:2], 1)
        if scale < 1:
            crop = cv2.resize(
                crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )
        else:
            crop = crop.copy()
        track["crops"].append(crop)
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

        # A face lost for too long is dropped, and its crops restart if it reappears.
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
                self.bridge.cv2_to_imgmsg(crop, "bgr8") for crop in track["crops"]
            ]
            track["new"] = 0
            # is_talking answers one request at a time.
            deadline = now + IS_TALKING_TIMEOUT * (len(self.talk_pending) + 1)
            self.talk_pending.append(
                (
                    track,
                    self.is_talking_client.call_async(request),
                    deadline,
                    self.talker_gen,
                )
            )

    def _reset_talking(self):
        for _, future, _, _ in self.talk_pending:
            if not future.done():
                self.is_talking_client.remove_pending_request(future)
        self.talk_tracks, self.talk_pending = [], []

    @staticmethod
    def _same_person(a, b):
        # Unknown faces all share a name, so also compare position.
        if a["name"] != b["name"]:
            return False
        return a["name"] != "Unknown" or (
            abs(a["x"] - b["x"]) < TRACK_THRESHOLD
            and abs(a["y"] - b["y"]) < TRACK_THRESHOLD
        )

    def _set_last_talker(self, track):
        prev = self.last_talker
        # Keep the current talker while its own latest answer still says talking;
        # once it is silent (or gone), whoever talks takes over.
        if (
            prev is not None
            and not self._same_person(prev, track)
            and any(
                t["talking"] and self._same_person(prev, t) for t in self.talk_tracks
            )
        ):
            return
        self.last_talker = {
            "name": track["name"],
            "x": track["x"],
            "y": track["y"],
            "bbox": track["bbox"],
        }
        if prev is None or not self._same_person(prev, track):
            self.get_logger().info(f"Last talker: {track['name']}")

    # ── Main loop ──

    def _image_callback(self, msg):
        # Inference runs on every new frame instead of a fixed-rate timer, so the
        # talking crops are a dense sequence without a second detection pass.
        super()._image_callback(msg)
        self.run()

    def run(self):
        if not self.active:
            if self.talk_tracks or self.talk_pending:
                self._reset_talking()
            return
        if self.image is None:
            return
        # Frame id is the source stamp; skip frames already processed.
        self.id = self.image_header.stamp if self.image_header is not None else None
        if self.id is not None and self.id == self.processing_id:
            return
        frame = self.image
        self.annotated_frame = frame
        self._run_inference(frame)

    def _run_inference(self, frame):
        self.processing_id = self.id
        annotated = frame.copy()
        center = [frame.shape[1] / 2, frame.shape[0] / 2]

        detected_faces = self.face_model.detect(frame)
        talker = self.last_talker
        talking_enabled = self._talking_enabled()
        now = time.time()

        self.curr_faces = []
        face_list = PersonList()
        largest_area, largest_area_params, largest_face_name = 0, None, ""
        follow_face_params = None
        # Face in this frame matching the cached talker: by name, else nearest by position.
        talker_params, talker_name, talker_face = None, "", None
        talker_by_name, talker_dist = False, None
        sticky_params, sticky_name, sticky_dist = None, "", None
        detected = False

        for ins_face in detected_faces:
            x1, y1, x2, y2 = ins_face.bbox.astype(int)
            left = max(x1 - TRACK_THRESHOLD, 0)
            right = min(x2 + TRACK_THRESHOLD, frame.shape[1])
            top = max(y1 - TRACK_THRESHOLD, 0)
            bottom = min(y2 + TRACK_THRESHOLD, frame.shape[0])
            centerx = (x1 + x2) / 2.0
            centery = (y1 + y2) / 2.0

            name = "Unknown"
            tracked = False
            for prev in self.prev_faces:
                if (
                    abs(prev["x"] - centerx) < TRACK_THRESHOLD
                    and abs(prev["y"] - centery) < TRACK_THRESHOLD
                ):
                    name = prev["name"]
                    tracked = True
                    break

            if not tracked or name == "Unknown":
                name = self.face_model.match(ins_face.embedding.astype(np.float32))
            if talking_enabled:
                self._add_talk_crop(frame, (x1, y1, x2, y2), name, now)

            xc = left + (right - left) / 2.0
            yc = top + (bottom - top) / 2.0
            area = (right - left) * (bottom - top)

            self.curr_faces.append({"x": xc, "y": yc, "name": name})
            p = Person()
            p.name = name
            p.x = int((xc - center[0]) * MAX_DEGREE / center[0])
            p.y = int((center[1] - yc) * MAX_DEGREE / center[1])
            face_list.list.append(p)
            detected = True

            color = (255, 0, 0) if tracked else (0, 0, 255)
            cv2.rectangle(
                annotated, (left, bottom - 35), (right, bottom), color, cv2.FILLED
            )
            cv2.rectangle(annotated, (left, top), (right, bottom), color, 2)
            cv2.putText(
                annotated,
                name,
                (left + 6, bottom - 6),
                cv2.FONT_HERSHEY_DUPLEX,
                1.0,
                (255, 255, 255),
                1,
            )

            if area > largest_area:
                largest_area = area
                largest_area_params = (left, top, right, bottom)
                largest_face_name = name
            if self.follow_name == name:
                follow_face_params = (left, top, right, bottom)
            # Faces move between ticks (and the camera turns), so match by face size.
            threshold = max(TRACK_THRESHOLD, x2 - x1)
            if self.area_target is not None:
                dist = abs(self.area_target[0] - xc) + abs(self.area_target[1] - yc)
                if dist < threshold and (sticky_dist is None or dist < sticky_dist):
                    sticky_dist = dist
                    sticky_params = (left, top, right, bottom)
                    sticky_name = name
            if talker is not None and not talker_by_name:
                dx = abs(talker["x"] - centerx)
                dy = abs(talker["y"] - centery)
                if talker["name"] != "Unknown" and talker["name"] == name:
                    talker_by_name = True
                elif not (dx < threshold and dy < threshold):
                    continue
                elif talker_dist is not None and dx + dy >= talker_dist:
                    continue
                talker_dist = dx + dy
                talker_params = (left, top, right, bottom)
                talker_name = name
                talker_face = (centerx, centery, (x1, y1, x2, y2))

        if talking_enabled:
            self._update_talking(now)
        elif self.talk_tracks or self.talk_pending:
            self._reset_talking()

        xc = yc = 0.0
        if largest_area_params:
            left, top, right, bottom = largest_area_params
            xc = left + (right - left) / 2.0
            yc = top + (bottom - top) / 2.0
            if self.new_name:
                crop = frame[top:bottom, left:right]
                if self.face_model.enroll(self.new_name, crop):
                    self.success(f"{self.new_name} face enrolled")
                else:
                    self.get_logger().warn(
                        f"Could not enroll {self.new_name}: no face found in the saved crop"
                    )
                largest_face_name = self.new_name
                if talker_params == largest_area_params:
                    talker_name = self.new_name
                # Back-patch the enrolled name into this tick's outputs, so neither the
                # published list nor the position tracker carries "Unknown" for a face
                # we just named.
                for i, face in enumerate(self.curr_faces):
                    if face["x"] == xc and face["y"] == yc:
                        self.curr_faces[i]["name"] = self.new_name
                        face_list.list[i].name = self.new_name
                        break
                self.new_name = ""

        # Target: specific name > last talker > largest face.
        target_name = largest_face_name
        if self.follow_name != "area":
            if follow_face_params:
                left, top, right, bottom = follow_face_params
                xc = left + (right - left) / 2.0
                yc = top + (bottom - top) / 2.0
                target_name = self.follow_name
            else:
                detected = False
        elif talker is not None and talker_params:
            left, top, right, bottom = talker_params
            xc = left + (right - left) / 2.0
            yc = top + (bottom - top) / 2.0
            target_name = talker_name
            cv2.putText(
                annotated,
                "TALKING",
                (left + 6, max(top - 8, 20)),
                cv2.FONT_HERSHEY_DUPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            # Refresh the cache so the talker is followed while moving,
            # unless a new answer replaced it during this frame.
            if self.last_talker is talker:
                tx, ty, bbox = talker_face
                self.last_talker = {
                    "name": talker_name if talker_name != "Unknown" else talker["name"],
                    "x": tx,
                    "y": ty,
                    "bbox": bbox,
                }
        elif sticky_params:
            # No talker, or it is out of view (cache kept): stay on the last followed face.
            left, top, right, bottom = sticky_params
            xc = left + (right - left) / 2.0
            yc = top + (bottom - top) / 2.0
            target_name = sticky_name

        self.area_target = (xc, yc) if detected and self.follow_name == "area" else None
        self.prev_faces = self.curr_faces
        self.annotated_frame = annotated

        # Always publish the full list of recognized faces.
        self.person_list_publisher.publish(face_list)

        if detected:
            self._publish_follow_face(xc, yc, target_name, center)
        else:
            self.name_publisher.publish(String(data=""))

        self.view_pub.publish(annotated)

    def _publish_follow_face(self, xc, yc, name, center):
        difx = 0.0 if xc == 0 else xc - center[0]
        dify = 0.0 if yc == 0 else center[1] - yc
        target = Point()
        target.x = difx * MAX_DEGREE / center[0]
        target.y = dify * MAX_DEGREE / center[1]
        self.follow_publisher.publish(target)
        self.name_publisher.publish(String(data=name))


def main(args=None):
    rclpy.init(args=args)
    spin(FaceRecognition(), threads=5)


if __name__ == "__main__":
    main()
