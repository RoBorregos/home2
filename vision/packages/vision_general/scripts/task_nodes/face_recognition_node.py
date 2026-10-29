#!/usr/bin/env python3
"""Face recognition ROS2 node. Model logic lives in models.face_recognition."""

import time
import rclpy
import rclpy.duration
import tqdm
from ament_index_python.packages import get_package_share_directory
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
    TALKING_HISTORY_FRAMES,
    TALKING_PEOPLE_TOPIC,
)

from frida_interfaces.msg import Person, PersonList
from frida_interfaces.srv import IsTalking, SaveName, TalkingPeople
from models.face_recognition import FaceModel, TRACK_THRESHOLD
from utils.debug_pub import DebugImagePublisher
from vision_runtime import VisionRuntime, spin

MAX_DEGREE = 1
TALKING_CROP_MARGIN = 0.25
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
        self.create_service(
            TalkingPeople,
            TALKING_PEOPLE_TOPIC,
            self.talking_people_callback,
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
        self.create_timer(0.2, self.run, callback_group=self.callback_group)

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
        # Idle until the task manager activates it.
        self.active = False
        self.is_processing = False
        self.id = None
        self.processing_id = rclpy.duration.Infinite

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
        res.success = len(self.curr_faces) > 0
        if not res.success:
            self.get_logger().info("No face detected")
        return res

    def _collect_face_crops(self, timeout):
        tracks = []
        last_stamp = None
        deadline = time.time() + timeout

        while time.time() < deadline:
            if tracks and all(
                len(t["crops"]) >= TALKING_HISTORY_FRAMES for t in tracks
            ):
                break

            frame = self.image
            stamp = self.image_header.stamp if self.image_header is not None else None
            if frame is None or stamp == last_stamp:
                time.sleep(0.005)
                continue
            last_stamp = stamp

            height, width = frame.shape[:2]
            for face in self.face_model.detect(frame):
                x1, y1, x2, y2 = face.bbox.astype(int)
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

                track = next(
                    (
                        t
                        for t in tracks
                        if abs(t["x"] - cx) < TRACK_THRESHOLD
                        and abs(t["y"] - cy) < TRACK_THRESHOLD
                    ),
                    None,
                )
                if track is None:
                    track = {
                        "name": self.face_model.match(
                            face.embedding.astype(np.float32)
                        ),
                        "crops": [],
                    }
                    tracks.append(track)
                track.update(x=cx, y=cy, bbox=(x1, y1, x2, y2))

                if len(track["crops"]) < TALKING_HISTORY_FRAMES:
                    mw = int((x2 - x1) * TALKING_CROP_MARGIN)
                    mh = int((y2 - y1) * TALKING_CROP_MARGIN)
                    crop = frame[
                        max(y1 - mh, 0) : min(y2 + mh, height),
                        max(x1 - mw, 0) : min(x2 + mw, width),
                    ].copy()
                    track["crops"].append(crop)

        return tracks

    def talking_people_callback(self, req, res):
        self.get_logger().info("Executing service talking people")
        res.success = False
        res.people = []

        if not self.is_talking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("is_talking service not available")
            return res

        timeout = req.timeout if req.timeout > 0 else 5.0

        # Pause run() while this thread uses the face model.
        while self.is_processing:
            time.sleep(0.01)
        self.is_processing = True
        try:
            tracks = self._collect_face_crops(timeout)
        finally:
            self.is_processing = False

        if not tracks:
            self.get_logger().info("No faces detected")
            return res

        height, width = self.image.shape[:2]
        answered = 0
        for track in tracks:
            if len(track["crops"]) < TALKING_HISTORY_FRAMES:
                continue

            request = IsTalking.Request()
            request.frames = [
                self.bridge.cv2_to_imgmsg(crop, "bgr8") for crop in track["crops"]
            ]
            response = self.call_service(self.is_talking_client, request, 5.0)
            if response is None:
                self.get_logger().warn(f"is_talking timed out for {track['name']}")
                continue

            answered += 1
            self.get_logger().info(f"{track['name']}: {response.message}")
            if not response.is_talking:
                continue

            x1, y1, x2, y2 = track["bbox"]
            person = Person()
            person.name = track["name"]
            person.x = int((track["x"] - width / 2) * MAX_DEGREE / (width / 2))
            person.y = int((height / 2 - track["y"]) * MAX_DEGREE / (height / 2))
            person.x1, person.y1 = int(x1), int(y1)
            person.x2, person.y2 = int(x2), int(y2)
            res.people.append(person)

        res.success = answered > 0
        return res

    # ── Main loop ──

    def run(self):
        if not self.active or self.is_processing:
            return
        if self.image is None:
            self.get_logger().info("No image", throttle_duration_sec=5.0)
            return
        # Frame id is the source stamp; skip frames already processed.
        self.id = self.image_header.stamp if self.image_header is not None else None
        if self.id is not None and self.id == self.processing_id:
            return
        frame = self.image
        self.annotated_frame = frame
        self.is_processing = True
        try:
            self._run_inference(frame)
        finally:
            self.is_processing = False

    def _run_inference(self, frame):
        self.processing_id = self.id
        annotated = frame.copy()
        center = [frame.shape[1] / 2, frame.shape[0] / 2]

        detected_faces = self.face_model.detect(frame)

        self.curr_faces = []
        face_list = PersonList()
        largest_area, largest_area_params, largest_face_name = 0, None, ""
        follow_face_params = None
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
                # Back-patch the enrolled name into this tick's outputs, so neither the
                # published list nor the position tracker carries "Unknown" for a face
                # we just named.
                for i, face in enumerate(self.curr_faces):
                    if face["x"] == xc and face["y"] == yc:
                        self.curr_faces[i]["name"] = self.new_name
                        face_list.list[i].name = self.new_name
                        break
                self.new_name = ""

        if self.follow_name != "area":
            if follow_face_params:
                left, top, right, bottom = follow_face_params
                xc = left + (right - left) / 2.0
                yc = top + (bottom - top) / 2.0
                largest_face_name = self.follow_name
            else:
                detected = False

        self.prev_faces = self.curr_faces
        self.annotated_frame = annotated

        # Always publish the full list of recognized faces.
        self.person_list_publisher.publish(face_list)

        if detected:
            self._publish_follow_face(xc, yc, largest_face_name, center)
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
