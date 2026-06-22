#!/usr/bin/env python3

"""
Node to recognize known people and
name new faces.
- Publisher for largest face detected
coordinates for arm following.
- Service to assign name to face.
"""

import os
import pathlib

import cv2
import numpy as np
import rclpy
import rclpy.duration
import tqdm
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from insightface.app import FaceAnalysis
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from frida_constants.vision_constants import (
    CAMERA_INFO_TOPIC,
    DEPTH_IMAGE_TOPIC,
    FACE_RECOGNITION_IMAGE,
    FOLLOW_BY_TOPIC,
    FOLLOW_TOPIC,
    IMAGE_ORIENTED_TOPIC,
    PERSON_LIST_TOPIC,
    PERSON_NAME_TOPIC,
    SAVE_NAME_TOPIC,
)
from frida_interfaces.msg import Person, PersonList
from frida_interfaces.srv import SaveName


def _insightface_providers() -> list:
    cache_dir = os.environ.get("TENSORRT_CACHE_DIR")
    return [
        (
            "TensorrtExecutionProvider",
            {
                "trt_engine_cache_enable": True,
                "trt_engine_cache_path": cache_dir,
                "trt_fp16_enable": True,
            },
        ),
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]


DEFAULT_NAME = "ale"
TRACK_THRESHOLD = 50
MATCH_THRESHOLD = 0.35
MAX_DEGREE = 1
RESIZE_FACTOR = 1

PATH = str(pathlib.Path(__file__).parent)
PATH = get_package_share_directory("vision_general")
KNOWN_FACES_PATH = PATH + "/utils/known_faces"

INSIGHTFACE_MODEL = "buffalo_sc"


class FaceRecognition(Node):
    def __init__(self):
        super().__init__("face_recognition")
        self.bridge = CvBridge()
        self.pbar = tqdm.tqdm(total=2)
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self._img_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.image_subscriber = self.create_subscription(
            Image,
            IMAGE_ORIENTED_TOPIC,
            self.image_callback,
            self._img_qos,
            callback_group=self.callback_group,
        )
        self.depth_subscriber = self.create_subscription(
            Image,
            DEPTH_IMAGE_TOPIC,
            self.depth_callback,
            self._img_qos,
            callback_group=self.callback_group,
        )
        self.image_info_subscriber = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.image_info_callback,
            self._img_qos,
            callback_group=self.callback_group,
        )

        self.new_name_service = self.create_service(
            SaveName,
            SAVE_NAME_TOPIC,
            self.new_name_callback,
            callback_group=self.callback_group,
        )
        self.follow_by_service = self.create_service(
            SaveName,
            FOLLOW_BY_TOPIC,
            self.follow_by_name_callback,
            callback_group=self.callback_group,
        )

        self.follow_publisher = self.create_publisher(Point, FOLLOW_TOPIC, 10)
        self.view_pub = self.create_publisher(Image, FACE_RECOGNITION_IMAGE, 10)
        self.name_publisher = self.create_publisher(String, PERSON_NAME_TOPIC, 10)
        self.person_list_publisher = self.create_publisher(
            PersonList, PERSON_LIST_TOPIC, 10
        )

        self.verbose = self.declare_parameter("verbose", True)
        self.annotated_frame = []
        self.vision_active = False
        self.is_processing = False

        self.create_subscription(
            Bool,
            "/vision/face_recognition/active",
            self._active_callback,
            10,
            callback_group=self.callback_group,
        )

        self.setup()
        self.create_timer(0.2, self.run, callback_group=self.callback_group)

    def setup(self):
        """Initialise InsightFace pipeline and load known-face embeddings."""

        self.get_logger().info(f"Loading InsightFace model '{INSIGHTFACE_MODEL}'")
        self.app = FaceAnalysis(
            name=INSIGHTFACE_MODEL,
            providers=_insightface_providers(),
        )
        self.app.prepare(ctx_id=0, det_size=(640, 640))
        self.pbar.update(1)

        self.new_name = ""
        self.image_view = None
        self.image = None
        self.prev_faces = []
        self.curr_faces = []
        self.depth_image = []
        self.follow_name = "area"
        self.id = None
        self.processing_id = rclpy.duration.Infinite

        self.default_name = self.declare_parameter("default_name", DEFAULT_NAME)
        self.default_name = self.default_name.value

        self.people_encodings: list[np.ndarray] = []
        self.people_names: list[str] = []
        self.people_encodings.append(np.zeros(512, dtype=np.float32))
        self.people_names.append("random")

        self.clear()
        self.process_imgs()
        self.pbar.update(1)
        self.get_logger().info("Face Recognition Ready")

    def image_callback(self, data):
        """Callback to get image from camera"""
        self.id = data.header.stamp
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def depth_callback(self, data):
        """Callback to receive depth image from camera"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except Exception as e:
            print(f"Error: {e}")

    def image_info_callback(self, data):
        """Callback to receive camera info"""
        self.imageInfo = data

    def new_name_callback(self, req, res):
        """Callback to add a new face to the known faces"""
        self.get_logger().info("Executing service new face")
        self.new_name = req.name
        if len(self.curr_faces) == 0:
            self.get_logger().info("No face detected")
            res.success = False
        else:
            self.get_logger().info(f"New name: {self.new_name}")
            res.success = True
        return res

    def follow_by_name_callback(self, req, res):
        """Callback to follow face by name or area"""
        self.get_logger().info("Executing service follow by")
        self.follow_name = req.name
        if len(self.curr_faces) == 0:
            self.get_logger().info("No face detected")
            res.success = False
        else:
            self.get_logger().info(f"New name: {self.follow_name}")
            res.success = True
        return res

    @staticmethod
    def _cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
        """Return cosine similarity in [−1, 1]. Higher = more similar."""
        a = a / (np.linalg.norm(a) + 1e-10)
        b = b / (np.linalg.norm(b) + 1e-10)
        return float(np.dot(a, b))

    @staticmethod
    def _cosine_distances(known: list[np.ndarray], query: np.ndarray) -> np.ndarray:
        """Return 1 − cosine_similarity for each embedding in *known*."""
        if not known:
            return np.array([])
        mat = np.stack(known)
        mat = mat / (np.linalg.norm(mat, axis=1, keepdims=True) + 1e-10)
        q = query / (np.linalg.norm(query) + 1e-10)
        return 1.0 - mat @ q

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def publish_image(self) -> None:
        """Publish image with annotations"""
        if len(self.annotated_frame) > 0:
            self.view_pub.publish(
                self.bridge.cv2_to_imgmsg(self.annotated_frame, "bgr8")
            )

    def process_imgs(self) -> None:
        """Make encodings of known people images"""
        self.get_logger().info("Processing images")
        for filename in os.listdir(KNOWN_FACES_PATH):
            if filename == ".DS_Store":
                continue
            self.process_img(filename)

    def clear(self) -> None:
        """Clear previous results"""
        for filename in os.listdir(KNOWN_FACES_PATH):
            if (
                filename == ".DS_Store"
                or filename == "random.png"
                or filename == self.default_name + ".png"
            ):
                continue
            file_path = os.path.join(KNOWN_FACES_PATH, filename)
            os.remove(file_path)

    def apply_clahe(self, bgr_img: np.ndarray) -> np.ndarray:
        try:
            lab = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2LAB)
            l_img, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(16, 16))
            l_img = clahe.apply(l_img)
            lab = cv2.merge((l_img, a, b))
            return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        except Exception:
            return bgr_img

    def process_img(self, filename: str) -> None:
        """Process image, obtain encodings and add to known people"""
        img_path = f"{KNOWN_FACES_PATH}/{filename}"
        img_bgr = cv2.imread(img_path)
        if img_bgr is None:
            self.get_logger().warn(f"Could not load image: {img_path}")
            return

        img_bgr = self.apply_clahe(img_bgr)
        faces = self.app.get(img_bgr)
        if len(faces) == 0:
            self.get_logger().warn(f"No face found in {filename}, skipping.")
            return

        face = max(faces, key=lambda f: _bbox_area(f.bbox))
        embedding = face.embedding.astype(np.float32)

        self.people_encodings.append(embedding)
        self.people_names.append(filename[:-4])

    def save_face(self, name: str, xc: float, yc: float) -> None:
        """Save face to list and return Person message"""
        self.curr_faces.append({"x": xc, "y": yc, "name": name})
        curr_person = Person()
        curr_person.name = name
        curr_person.x = int((xc - self.center[0]) * MAX_DEGREE / self.center[0])
        curr_person.y = int((self.center[1] - yc) * MAX_DEGREE / self.center[1])
        self.face_list.list.append(curr_person)

    def assign_name(
        self,
        left: float,
        top: float,
        bottom: float,
        right: float,
        xc: float,
        yc: float,
    ) -> None:
        """Assign name to largest face detected"""
        crop = self.frame[top:bottom, left:right]
        img_name = f"{self.new_name}.png"
        save_path = f"{KNOWN_FACES_PATH}/{img_name}"
        cv2.imwrite(save_path, crop)

        self.process_img(img_name)

        for i, face in enumerate(self.curr_faces):
            if face["x"] == xc and face["y"] == yc:
                self.curr_faces[i]["name"] = self.new_name
                self.face_list.list[i].name = self.new_name
                break

        self.success(f"{self.new_name} face enrolled")
        self.new_name = ""

    def publish_follow_face(self, xc: float, yc: float, largest_face_name: str) -> None:
        """Publish coordinates for arm to follow face"""
        difx = 0.0 if xc == 0 else xc - self.center[0]
        dify = 0.0 if yc == 0 else self.center[1] - yc
        move_x = difx * MAX_DEGREE / self.center[0]
        move_y = dify * MAX_DEGREE / self.center[1]

        target = Point()
        target.x = move_x
        target.y = move_y
        self.follow_publisher.publish(target)

        person_seen = String()
        person_seen.data = largest_face_name
        self.name_publisher.publish(person_seen)
        self.person_list_publisher.publish(self.face_list)

    def _active_callback(self, msg):
        if msg.data == self.vision_active:
            return
        self.vision_active = msg.data
        self.get_logger().info(f"Face recognition active: {self.vision_active}")

    def run(self) -> None:
        """Run face recognition algorithm"""
        if not self.vision_active:
            return
        if self.is_processing:
            return
        if self.image is None:
            self.get_logger().info("No image")
            return

        self.annotated_frame = self.image

        if self.id == self.processing_id:
            return

        self.is_processing = True
        try:
            self._run_inference()
        finally:
            self.is_processing = False

    def _run_inference(self) -> None:
        """Actual face recognition inference (called by run with lock)."""
        self.processing_id = self.id

        self.frame = self.image
        self.annotated_frame = self.frame.copy()
        self.center = [self.frame.shape[1] / 2, self.frame.shape[0] / 2]
        detected_faces = self.app.get(self.apply_clahe(self.frame))

        largest_area = 0
        follow_face_params = None
        largest_area_params = None
        largest_face_name = ""

        self.curr_faces = []
        self.face_list = PersonList()
        detected = False

        for ins_face in detected_faces:
            x1, y1, x2, y2 = ins_face.bbox.astype(int)

            left = max(x1 - TRACK_THRESHOLD, 0)
            right = min(x2 + TRACK_THRESHOLD, self.frame.shape[1])
            top = max(y1 - TRACK_THRESHOLD, 0)
            bottom = min(y2 + TRACK_THRESHOLD, self.frame.shape[0])

            centerx = (x1 + x2) / 2.0
            centery = (y1 + y2) / 2.0

            name = "Unknown"
            flag = False
            for prev_face in self.prev_faces:
                if (
                    abs(prev_face["x"] - centerx) < TRACK_THRESHOLD
                    and abs(prev_face["y"] - centery) < TRACK_THRESHOLD
                ):
                    name = prev_face["name"]
                    flag = True
                    break

            if not flag or name == "Unknown":
                query_embedding = ins_face.embedding.astype(np.float32)

                if len(self.people_encodings) > 0:
                    distances = self._cosine_distances(
                        self.people_encodings, query_embedding
                    )
                    best_match_idx = int(np.argmin(distances))
                    best_similarity = 1.0 - float(distances[best_match_idx])

                    if best_similarity >= MATCH_THRESHOLD and best_match_idx != 0:
                        name = self.people_names[best_match_idx]

            xc = left + (right - left) / 2.0
            yc = top + (bottom - top) / 2.0
            area = (right - left) * (bottom - top)

            self.save_face(name, xc, yc)
            detected = True

            color = (255, 0, 0) if flag else (0, 0, 255)
            cv2.rectangle(
                self.annotated_frame,
                (left, bottom - 35),
                (right, bottom),
                color,
                cv2.FILLED,
            )
            cv2.rectangle(
                self.annotated_frame,
                (left, top),
                (right, bottom),
                color,
                2,
            )
            cv2.putText(
                self.annotated_frame,
                name,
                (left + 6, bottom - 6),
                cv2.FONT_HERSHEY_DUPLEX,
                1.0,
                (255, 255, 255),
                1,
            )

            if area > largest_area:
                largest_area = area
                largest_area_params = [left, top, right, bottom]
                largest_face_name = name

            if self.follow_name == name:
                follow_face_params = [left, top, right, bottom]

        xc = 0.0
        yc = 0.0

        if largest_area != 0:
            left, top, right, bottom = largest_area_params
            xc = left + (right - left) / 2.0
            yc = top + (bottom - top) / 2.0

            if self.new_name != "":
                largest_face_name = self.new_name
                self.assign_name(left, top, bottom, right, xc, yc)

        if self.follow_name != "area":
            if follow_face_params is not None:
                left, top, right, bottom = follow_face_params
                xc = left + (right - left) / 2.0
                yc = top + (bottom - top) / 2.0
                largest_face_name = self.follow_name
            else:
                detected = False

        self.prev_faces = self.curr_faces

        if detected:
            self.publish_follow_face(xc, yc, largest_face_name)
        else:
            self.name_publisher.publish(String(data=""))

        self.publish_image()


def _bbox_area(bbox) -> float:
    """Return pixel area of an InsightFace bbox [x1, y1, x2, y2]."""
    x1, y1, x2, y2 = bbox
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FaceRecognition()
        executor = rclpy.executors.MultiThreadedExecutor(5)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
