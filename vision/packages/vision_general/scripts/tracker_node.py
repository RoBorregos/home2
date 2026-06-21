#!/usr/bin/env python3

"""
Node to track a single person and re-id them if necessary.
Requires 2 terminals minimum (3 if using pose/color detection via moondream).

--- Terminal 1: Run the tracker node ---
    ros2 run vision_general tracker_node

--- Terminal 2: Call a tracking service ---

    Option 1) Track the largest person (default):
        ros2 service call /vision/set_tracking_target std_srvs/srv/SetBool "{data: true}"

    Option 2) Track by gesture (e.g. waving):
        ros2 service call /vision/set_tracking_target_by frida_interfaces/srv/TrackBy \
            "{track_enabled: true, track_by: 'gestures', value: 'waving'}"

    Option 3) Track by pose (standing, sitting, lying down):
        ros2 service call /vision/set_tracking_target_by frida_interfaces/srv/TrackBy \
            "{track_enabled: true, track_by: 'poses', value: 'standing'}"

    Option 4) Track by clothing color:
        ros2 service call /vision/set_tracking_target_by frida_interfaces/srv/TrackBy \
            "{track_enabled: true, track_by: 'color', value: 'red shirt'}"

    Disable tracking:
        ros2 service call /vision/set_tracking_target std_srvs/srv/SetBool "{data: false}"

    Check if tracking is active:
        ros2 service call /vision/is_tracking std_srvs/srv/Trigger

--- Terminal 3 (Orin only): Run the zed initialziation 
    zed

--- Terminal 3 (optional): Moondream vision model (required for poses/color) ---
    ros2 run vision_general moondream_node
"""

import cv2
import time
import numpy as np
from PIL import Image as PILImage
import tqdm
import torch.nn as nn
import torch
from vision_general.utils.calculations import (
    get2DCentroid,
    deproject_pixel_to_point,
)

import copy
import threading
import rclpy
from rclpy.node import Node
from vision_general.utils.ros_utils import wait_for_future
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped

from vision_general.utils.reid_model import (
    load_network,
    compare_images,
    compare_images_batch,
    extract_feature_from_img,
    extract_feature_from_img_batch,
    get_structure,
)

from vision_general.utils.deep_sort.tracker import Tracker as DeepSORTTracker
from vision_general.utils.deep_sort.detection import Detection as DeepSORTDetection
from vision_general.utils.deep_sort.nn_matching import NearestNeighborDistanceMetric
from vision_general.utils.trt_utils import load_yolo_trt

from std_srvs.srv import SetBool, Trigger
from frida_interfaces.srv import TrackBy, CropQuery
from pose_detection import PoseDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    SET_TARGET_TOPIC,
    SET_TARGET_BY_TOPIC,
    TRACKER_IMAGE_TOPIC,
    DEPTH_IMAGE_TOPIC,
    RESULTS_TOPIC,
    CAMERA_INFO_TOPIC,
    CENTROID_TOPIC,
    CROP_QUERY_TOPIC,
    IS_TRACKING_TOPIC,
    FLIP_TRACKER_TOPIC,
)
from frida_constants.vision_enums import DetectBy
from std_msgs.msg import Bool

CONF_THRESHOLD = 0.6
DEPTH_THRESHOLD_NS = 50_000_000  # 50 ms in nanoseconds
REID_EXTRACT_FREQ = 0.3
MAX_EMBEDDINGS = 128
DEEPSORT_MAX_COSINE_DISTANCE = 0.3
DEEPSORT_NN_BUDGET = 100
DEEPSORT_MAX_AGE = 100
DEEPSORT_N_INIT = 3
DEPTH_JUMP_THRESHOLD = 0.5  # Max allowed depth change (meters) between frames


class SingleTracker(Node):
    def __init__(self):
        super().__init__("tracker_node")

        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )

        self.image_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.image_subscriber = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            qos,
            callback_group=self.image_callback_group,
        )

        depth_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        self.depth_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.depth_subscriber = self.create_subscription(
            Image,
            DEPTH_IMAGE_TOPIC,
            self.depth_callback,
            depth_qos,
            callback_group=self.depth_callback_group,
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, qos
        )

        self.service_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.set_target_service = self.create_service(
            SetBool,
            SET_TARGET_TOPIC,
            self.set_target_callback,
            callback_group=self.service_callback_group,
        )

        self.set_target_by_service = self.create_service(
            TrackBy,
            SET_TARGET_BY_TOPIC,
            self.set_target_by_callback,
            callback_group=self.service_callback_group,
        )

        self.get_is_tracking_service = self.create_service(
            Trigger,
            IS_TRACKING_TOPIC,
            self.get_is_tracking_callback,
            callback_group=self.service_callback_group,
        )

        self.is_tracking_result = False

        self.results_publisher = self.create_publisher(PointStamped, RESULTS_TOPIC, 10)

        self.image_publisher = self.create_publisher(Image, TRACKER_IMAGE_TOPIC, 10)

        self.centroid_publisher = self.create_publisher(Point, CENTROID_TOPIC, 10)

        self.moondream_client = self.create_client(
            CropQuery, CROP_QUERY_TOPIC, callback_group=self.callback_group
        )
        self.create_subscription(
            Bool,
            FLIP_TRACKER_TOPIC,
            self._flip_callback,
            10,
            callback_group=self.callback_group,
        )

        self.verbose = self.declare_parameter("verbose", True)
        self.setup()
        self.flip_image = False
        self.last_reid_extraction = time.time()
        self.timer_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        # 10 Hz tracking timer. Stage 0 uses ByteTrack (no per-frame ReID): each
        # cycle is detector-bound and fast, so the realized rate is detector-limited
        # (not timer-limited). 20 Hz was tried but the old per-frame SWIN ReID was
        # the real bottleneck.
        self.create_timer(0.1, self.run, callback_group=self.timer_callback_group)
        self.create_timer(
            0.1, self.publish_image, callback_group=self.timer_callback_group
        )

    def setup(self):
        """Load models and initial variables"""
        self.target_set = False
        # Serializes the tracker timer (run) and the set_target service so they
        # never touch the shared self.frame / TensorRT self.model concurrently.
        self._infer_lock = threading.Lock()
        self.image = None
        self.image_time = None
        self.frame_id = "zed_left_camera_optical_frame"
        self.person_data = {
            "id": None,
            "embeddings": None,
            "num_embeddings": 0,
            "forward": None,
            "backward": None,
            "left": None,
            "right": None,
            "coordinates": [],
        }
        self.depth_image_time = None
        self.last_depth = None
        pbar = tqdm.tqdm(total=2, desc="Loading models")

        # Load YOLO with TensorRT acceleration for Orin AGX
        self.model = load_yolo_trt("yolov8n.pt")
        pbar.update(1)
        self.pose_detection = PoseDetection()
        pbar.update(1)

        # Stage 0: per-frame tracking is ultralytics ByteTrack (self.model.track) —
        # NO per-frame appearance ReID. The heavy SWIN ReID is NOT loaded at startup;
        # Stage 1 will lazy-load a lightweight re-acquisition model (OSNet / face) on
        # demand. DeepSORT is replaced by ByteTrack (see _track()).
        self.model_reid = None
        self.deepsort_tracker = None

        self.output_image = []
        self.depth_image = []

        pbar.close()
        self.get_logger().info("Single Tracker Ready (ByteTrack, Stage 0)")

    def image_callback(self, data):
        """Callback to receive image from camera"""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image_time = data.header.stamp

    def depth_callback(self, data):
        """Callback to receive depth image from camera"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image = depth_image
            self.depth_image_time = data.header.stamp
            self.get_logger().info("Depth image received", once=True)
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")

    def get_is_tracking_callback(self, request, response):
        response = Trigger.Response()
        response.success = self.is_tracking_result
        if self.is_tracking_result:
            self.get_logger().info("Tracking")
        else:
            self.get_logger().info("Not tracking")
        return response

    def image_info_callback(self, data):
        """Callback to receive camera info"""
        self.imageInfo = data

    def _flip_callback(self, msg):
        if msg.data != self.flip_image:
            self.flip_image = msg.data
            self.get_logger().info(f"Flip image set to: {self.flip_image}")

    def set_target_callback(self, request, response):
        """Callback to set the target to track"""
        self.target_set = request.data
        if self.target_set:
            response.success = self.set_target()
            self.get_logger().info("Tracking enabled: Target set")
        else:
            response.success = True
            self.get_logger().info("Tracking disabled")
        return response

    def set_target_by_callback(self, request, response):
        """Callback to set target by pose, gesture, clothes, etc"""
        self.target_set = request.track_enabled
        if self.target_set:
            response.success = self.set_target(request.track_by, request.value)
            self.get_logger().info(
                f"Tracking enabled: Target set by {request.track_by}"
            )
        else:
            response.success = True
            self.get_logger().info("Tracking disabled")
        return response

    def publish_image(self):
        """Publish the image to the camera topic"""
        if len(self.output_image) != 0:
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def _extract_reid_feature_np(self, cropped_image):
        """Extract ReID feature as numpy array for DeepSORT."""
        pil_image = PILImage.fromarray(cropped_image)
        with torch.no_grad():
            embedding = extract_feature_from_img(pil_image, self.model_reid)
        return embedding.cpu().numpy().flatten()

    def _track(self, frame):
        """Per-frame multi-object tracking via ultralytics ByteTrack (Stage 0).

        Appearance-free: no per-detection ReID embedding. persist=True keeps the
        ByteTrack state (and thus the track-ids) across calls. Returns the SAME
        structure the downstream consumers expect — a list of
        {"track_id", "x1", "y1", "x2", "y2"} dicts (person class only).
        """
        frame_h, frame_w = frame.shape[:2]
        results = self.model.track(
            frame,
            classes=0,
            persist=True,
            tracker="bytetrack.yaml",
            verbose=False,
        )
        tracked = []
        if not results:
            return tracked
        boxes = results[0].boxes
        if boxes is None or boxes.id is None:
            return tracked
        ids = boxes.id.int().cpu().tolist()
        xyxy = boxes.xyxy.cpu().tolist()
        confs = boxes.conf.cpu().tolist()
        for tid, (bx1, by1, bx2, by2), conf in zip(ids, xyxy, confs):
            if conf < CONF_THRESHOLD:
                continue
            x1 = max(0, min(int(round(bx1)), frame_w - 1))
            y1 = max(0, min(int(round(by1)), frame_h - 1))
            x2 = max(0, min(int(round(bx2)), frame_w))
            y2 = max(0, min(int(round(by2)), frame_h))
            if x2 <= x1 or y2 <= y1:
                continue
            tracked.append(
                {"track_id": int(tid), "x1": x1, "y1": y1, "x2": x2, "y2": y2}
            )
        return tracked

    def _run_deepsort(self, frame, yolo_results):
        """[Stage 0: UNUSED — replaced by _track()/ByteTrack] DeepSORT on YOLO dets."""
        frame_h, frame_w = frame.shape[:2]  # ✅ use distinct names

        detections = []
        for out in yolo_results:
            for box in out.boxes:
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]

                # Ensure the bounding box is within the frame
                x1 = max(0, min(x1, frame_w - 1))
                y1 = max(0, min(y1, frame_h - 1))
                x2 = max(0, min(x2, frame_w))
                y2 = max(0, min(y2, frame_h))

                prob = round(box.conf[0].item(), 2)
                if prob < CONF_THRESHOLD:
                    continue
                cropped = frame[y1:y2, x1:x2]
                if cropped.size == 0:
                    continue
                feature = self._extract_reid_feature_np(cropped)
                w = x2 - x1
                h = y2 - y1
                det = DeepSORTDetection(
                    tlwh=np.array([x1, y1, w, h], dtype=np.float64),
                    confidence=prob,
                    feature=feature,
                )
                detections.append(det)

        self.deepsort_tracker.predict()
        self.deepsort_tracker.update(detections)

        tracks = []
        for track in self.deepsort_tracker.tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue
            bbox = track.to_tlbr()

            x1 = max(0, int(bbox[0]))
            y1 = max(0, int(bbox[1]))
            x2 = min(frame_w, int(bbox[2]))
            y2 = min(frame_h, int(bbox[3]))

            tracks.append(
                {
                    "track_id": track.track_id,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                }
            )
        return tracks

    def _reset_person_data(self):
        """Reset the person data"""
        self.person_data["id"] = None
        self.person_data["embeddings"] = None
        self.person_data["num_embeddings"] = 0
        self.person_data["forward"] = None
        self.person_data["backward"] = None
        self.person_data["left"] = None
        self.person_data["right"] = None
        self.last_depth = None

    def set_target(self, track_by="largest_person", value=""):
        # Serialize against the run() timer: wait for any in-flight inference, then
        # run exclusively, so self.frame / the TensorRT model aren't touched
        # concurrently (was: AttributeError 'NoneType' has no attribute 'shape').
        with self._infer_lock:
            return self._set_target_impl(track_by, value)

    def _set_target_impl(self, track_by="largest_person", value=""):
        """Set the target to track (Default: Largest person in frame)"""
        if self.image is None:
            self.get_logger().warn("No image available")
            self.is_tracking_result = False
            return False

        self.get_logger().info(f"Setting target by {track_by} with value {value}")
        self.person_data["id"] = None
        self.person_data["embeddings"] = None
        self.person_data["num_embeddings"] = 0

        self.frame = copy.deepcopy(self.image)

        if self.flip_image:
            self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)

        self.output_image = self.frame.copy()

        # ByteTrack runs continuously; SELECT the operator from the CURRENT tracks
        # (no DeepSORT n_init confirmation loop needed) — Stage 0. Track + crops all
        # use the same (flipped, if enabled) self.frame for consistency.
        tracked_people = self._track(self.frame)

        largest_person = {
            "id": None,
            "area": 0,
            "bbox": None,
        }
        response_clean = ""

        for person in tracked_people:
            x1, y1, x2, y2 = person["x1"], person["y1"], person["x2"], person["y2"]
            track_id = person["track_id"]

            cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

            area = (x2 - x1) * (y2 - y1)
            if track_by == "largest_person":
                if area > largest_person["area"]:
                    largest_person["id"] = track_id
                    largest_person["area"] = area
                    largest_person["bbox"] = (x1, y1, x2, y2)
            else:
                cropped_image = self.frame[y1:y2, x1:x2]

                if track_by == DetectBy.GESTURES.value:
                    self.get_logger().info(f"Detecting gesture {value} ")
                    pose = self.pose_detection.detectGesture(cropped_image)
                    response_clean = pose.value

                elif track_by == DetectBy.POSES.value:
                    prompt = "Respond 'standing' if the person in the image is standing, 'sitting' if the person in the image is sitting, 'lying down' if the person in the image is lying down or 'unknown' if the person is not doing any of the previous."
                    status, response_q = self.moondream_crop_query(
                        prompt, [float(y1), float(x1), float(y2), float(x2)]
                    )
                    if status:
                        self.get_logger().info(f"The person is {response_q}.")
                        response_clean = response_q.replace(" ", "_")
                        response_clean = response_clean.replace("_", "", 1)

                elif track_by == DetectBy.COLOR.value:
                    prompt = f"Reply only with 1 if the person is wearing {value}. Otherwise, reply only with 0."
                    status, response_q = self.moondream_crop_query(
                        prompt, [float(y1), float(x1), float(y2), float(x2)]
                    )
                    if status:
                        response_clean = response_q.strip()

                if response_clean == value or response_clean == "1":
                    self.success(f"Target found by {track_by}: {response_clean}")
                    largest_person["id"] = track_id
                    largest_person["area"] = area
                    largest_person["bbox"] = (x1, y1, x2, y2)
                else:
                    self.get_logger().warn(
                        f"Person detected with {track_by}: {response_clean} but not {value}"
                    )
                    cropped_image = self.frame[y1:y2, x1:x2]

                    if track_by == DetectBy.GESTURES.value:
                        self.get_logger().info(f"Detecting gesture {value} ")
                        pose = self.pose_detection.detectGesture(cropped_image)
                        response_clean = pose.value
                        print("POSEEEE", response_clean)

                    elif track_by == DetectBy.POSES.value:
                        prompt = "Respond 'standing' if the person in the image is standing, 'sitting' if the person in the image is sitting, 'lying down' if the person in the image is lying down or 'unknown' if the person is not doing any of the previous."
                        status, response_q = self.moondream_crop_query(
                            prompt, [float(y1), float(x1), float(y2), float(x2)]
                        )

                        if status:
                            self.get_logger().info(f"The person is {response_q}.")
                            response_clean = response_q.replace(" ", "_")
                            response_clean = response_clean.replace("_", "", 1)

                    elif track_by == DetectBy.COLOR.value:
                        prompt = f"Reply only with 1 if the person is wearing {value}. Otherwise, reply only with 0."
                        status, response_q = self.moondream_crop_query(
                            prompt, [float(y1), float(x1), float(y2), float(x2)]
                        )
                        if status:
                            response_clean = response_q.strip()

                    if value == "wavingCustomer":
                        pts, kpc = self.pose_detection._get_keypoints(cropped_image)
                        raising = self.pose_detection.is_waving_from_keypoints(pts, kpc)
                        if raising:
                            print("Is rising hand")
                            response_clean = value

                    if response_clean == value or response_clean == "1":
                        if pose.value == value:
                            self.success(f"Target found by {track_by}: {pose.value}")
                        elif response_clean == value:
                            self.success(
                                f"Target found by {track_by}: {response_clean}"
                            )
                        elif response_clean == "1":
                            self.success(f"Target found by {track_by}: {value}")
                        largest_person["id"] = track_id
                        largest_person["area"] = area
                        largest_person["bbox"] = (x1, y1, x2, y2)
                        # cv2.im(cropped_image)
                        self.customer_image = copy.deepcopy(cropped_image)
                    else:
                        self.get_logger().warn(
                            f"Person detected with {track_by}: {response_clean} but not {value}"
                        )

        if largest_person["id"] is not None:
            self.person_data["id"] = largest_person["id"]
            self.success(f"Target set: {largest_person['id']}")
            cv2.rectangle(
                self.output_image,
                largest_person["bbox"][:2],
                largest_person["bbox"][2:],
                (0, 255, 0),
                2,
            )
            cv2.putText(
                self.output_image,
                f"Target by {track_by}: {largest_person['id']}",
                largest_person["bbox"][:2],
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            return True
        else:
            self.get_logger().warn("No person found")
            return False

    def moondream_crop_query(self, prompt: str, bbox: list[float]) -> tuple[int, str]:
        """Makes a query of the current image using moondream."""
        self.get_logger().info(f"Querying image with prompt: {prompt}")

        height, width = self.image.shape[:2]

        ymin = bbox[0] / height
        xmin = bbox[1] / width
        ymax = bbox[2] / height
        xmax = bbox[3] / width

        request = CropQuery.Request()
        request.query = prompt
        request.ymin = ymin
        request.xmin = xmin
        request.ymax = ymax
        request.xmax = xmax

        future = self.moondream_client.call_async(request)
        future = wait_for_future(future, 15)
        result = future.result()
        if result is None:
            self.get_logger().error("Moondream service returned None.")
            return 0, "0"
        if result.success:
            self.get_logger().info(f"Moondream result: {result.result}")
            return 1, result.result

    def run(self):
        # Timer entry. Skip this tick if an inference is already in flight (another
        # run() or a set_target): serializes access to the shared self.frame and the
        # persistent ByteTrack state (self.model.track(persist=True)) so overlapping
        # ticks can't stack up or race set_target (set_target was crashing on
        # frame=None before this guard).
        if not self.target_set:
            self.is_tracking_result = False
            return
        if not self._infer_lock.acquire(blocking=False):
            return
        try:
            self._run_impl()
        finally:
            self._infer_lock.release()

    def _run_impl(self):
        """Main loop to run the tracker"""

        if not self.target_set:
            self.is_tracking_result = False
            return

        self.frame = copy.deepcopy(self.image)
        if self.frame is None:
            self.get_logger().error("No image available")
            return

        if self.flip_image:
            self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)

        self.output_image = self.frame.copy()

        # Per-frame detection + ByteTrack (motion-only, no per-frame ReID) — Stage 0
        start_time = time.time()
        tracked_people = self._track(self.frame)
        self.get_logger().info(
            f"Det+Tracking took {time.time() - start_time:.2f}s | People: {len(tracked_people)}"
        )

        if self.person_data["id"] is None:
            self.frame = None
            return

        person_in_frame = False

        for person in tracked_people:
            x1, y1, x2, y2 = person["x1"], person["y1"], person["x2"], person["y2"]
            track_id = person["track_id"]

            if track_id == self.person_data["id"]:
                person_in_frame = True
                self.person_data["coordinates"] = (x1, y1, x2, y2)
                cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            else:
                cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (255, 0, 0), 2)

            cv2.putText(
                self.output_image,
                f"person {track_id}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
            )

        # TODO Stage 1: when the locked track-id disappears (person_in_frame is
        # False but other tracks exist), run a LIGHTWEIGHT OSNet body-ReID and/or an
        # InsightFace face match over the current tracks to re-acquire the SAME
        # operator, then set self.person_data["id"] to the re-found track and
        # person_in_frame = True. Stage 0 has NO per-frame / per-loss ReID: it relies
        # purely on the ByteTrack track-id, which ultralytics persists across short
        # occlusions.

        if person_in_frame:
            self.is_tracking_result = True
            if self.depth_image_time is None or self.image_time is None:
                self.get_logger().warn(
                    f"Depth image not available (depth_time={self.depth_image_time is not None}, "
                    f"image_time={self.image_time is not None})"
                )
                self.frame = None
                return
            depth_stamp_ns = (
                self.depth_image_time.sec * 1_000_000_000
                + self.depth_image_time.nanosec
            )
            image_stamp_ns = (
                self.image_time.sec * 1_000_000_000 + self.image_time.nanosec
            )
            stamp_diff_ns = abs(depth_stamp_ns - image_stamp_ns)
            self.get_logger().debug(
                f"Stamp diff: {stamp_diff_ns / 1e6:.1f} ms "
                f"(depth={self.depth_image_time.sec}.{self.depth_image_time.nanosec:09d}, "
                f"rgb={self.image_time.sec}.{self.image_time.nanosec:09d})"
            )
            if len(self.depth_image) > 0 and stamp_diff_ns < DEPTH_THRESHOLD_NS:
                print("Entro al otro papu")
                coords = PointStamped()
                coords.header.frame_id = self.frame_id
                coords.header.stamp = self.depth_image_time
                point2D = get2DCentroid(
                    self.person_data["coordinates"], self.depth_image
                )
                point2D_x_coord = float(point2D[1])
                point2D_x_coord_normalized = (
                    point2D_x_coord / (self.frame.shape[1] / 2)
                ) - 1
                point2Dpoint = Point()
                point2Dpoint.x = float(point2D_x_coord_normalized)
                point2Dpoint.y = 0.0
                point2Dpoint.z = 0.0
                self.centroid_publisher.publish(point2Dpoint)

                # Robust depth: median from inner 50% of bounding box
                x1, y1, x2, y2 = self.person_data["coordinates"]
                bw, bh = x2 - x1, y2 - y1
                inner_x1 = x1 + bw // 4
                inner_x2 = x2 - bw // 4
                inner_y1 = y1 + bh // 4
                inner_y2 = y2 - bh // 4
                roi = self.depth_image[inner_y1:inner_y2, inner_x1:inner_x2]
                valid = roi[np.isfinite(roi) & (roi > 0.0)]
                if valid.size == 0:
                    self.frame = None
                    return
                depth = float(np.median(valid))

                # Reject sudden depth jumps
                if (
                    self.last_depth is not None
                    and abs(depth - self.last_depth) > DEPTH_JUMP_THRESHOLD
                ):
                    self.frame = None
                    return
                self.last_depth = depth

                point_2d_temp = (point2D[1], point2D[0])
                point3D = deproject_pixel_to_point(self.imageInfo, point_2d_temp, depth)
                coords.point.x = float(point3D[0])
                coords.point.y = float(point3D[1])
                coords.point.z = float(point3D[2])
                self.results_publisher.publish(coords)
            else:
                self.get_logger().warn("Depth image not available")
        else:
            self.is_tracking_result = False

        self.frame = None


def main(args=None):
    rclpy.init(args=args)
    node = SingleTracker()

    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
