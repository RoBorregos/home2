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
import torch
from vision_general.utils.calculations import (
    deproject_pixel_to_point,
)

import queue
import threading
import rclpy
from rclpy.node import Node
from vision_general.utils.ros_utils import wait_for_future
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped

from vision_general.utils.reid_model import (
    extract_feature_from_img,
    get_structure,
    load_network,
)

from vision_general.utils.deep_sort.detection import Detection as DeepSORTDetection
from vision_general.utils.trt_utils import load_yolo_trt
from vision_general.utils.debug_pub import DebugImagePublisher

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
    CAMERA_ROTATION_TOPIC,
)
from frida_constants.vision_enums import DetectBy
from std_msgs.msg import Bool, Int16

CONF_THRESHOLD = 0.6
# RGB/depth pairing gate. The ZED publishes at ~15 Hz (~66 ms between frames);
# a 50 ms gate rejected legitimate same-grab pairs and silently dropped 3D
# publishes, so allow up to one frame period of skew.
DEPTH_THRESHOLD_NS = 100_000_000  # 100 ms in nanoseconds
REID_EXTRACT_FREQ = 0.3
MAX_EMBEDDINGS = 128
# Re-acquisition: cosine-similarity floor to accept a current track as the SAME
# operator after the ByteTrack id is lost, and how often to attempt it (ReID
# extraction is heavy, so don't run it every tick).
REID_MATCH_THRESHOLD = 0.6
REID_REACQUIRE_FREQ = 0.3
DEEPSORT_MAX_COSINE_DISTANCE = 0.3
DEEPSORT_NN_BUDGET = 100
DEEPSORT_MAX_AGE = 100
DEEPSORT_N_INIT = 3
DEPTH_JUMP_THRESHOLD = 0.5  # Max allowed depth change (meters) between frames
# A depth jump can be REAL (the publisher skipped a few ticks while the person
# kept walking). Accept the new depth after this many consecutive rejects —
# otherwise one legit >threshold move wedges the 3D publisher silent forever
# and the base halts on the smoother's lost-person watchdog.
DEPTH_JUMP_MAX_REJECTS = 3


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

        self.image_publisher = DebugImagePublisher(self, TRACKER_IMAGE_TOPIC, "tracker")

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
        # Stay in sync with vision.camera_upside_down(): while the wrist camera is
        # inverted (e.g. carrying a bag) the tracker must flip the frame so detection
        # runs upright. 180 -> flip; 0 -> no flip (90/270 unsupported by this node).
        self.create_subscription(
            Int16,
            CAMERA_ROTATION_TOPIC,
            self._rotation_callback,
            10,
            callback_group=self.callback_group,
        )

        self.verbose = self.declare_parameter("verbose", True)
        self.setup()
        self.flip_image = False
        self.last_reid_extraction = time.time()
        self.timer_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        # 20 Hz tracking timer. The non-blocking _infer_lock skip makes the loop
        # self-pacing: the realized rate is detector-limited, so a fast timer just
        # removes the artificial cap. ReID (SWIN) no longer runs on this path —
        # gallery embeds and re-acquisition matching happen on the background
        # worker — so ticks stay at detector latency (~15-30 ms on the Orin).
        self.create_timer(0.05, self.run, callback_group=self.timer_callback_group)
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
            "embeddings": [],
            "num_embeddings": 0,
            "forward": None,
            "backward": None,
            "left": None,
            "right": None,
            "coordinates": [],
        }
        self.depth_image_time = None
        self.last_depth = None
        self._depth_reject_count = 0
        # Re-acquisition state: ReID model loads in a BACKGROUND thread (best-effort),
        # plus throttles for gallery extraction / re-acquire attempts.
        self.reid_failed = False
        self._reid_loading = False
        self.last_reacq_attempt = 0.0
        # All SWIN forwards (gallery embeds + re-acquisition matching) run on this
        # single worker so they never block the tracking tick. Jobs carry a target
        # generation stamp; stale jobs from a previous target are discarded.
        self._reid_jobs = queue.Queue(maxsize=4)
        self._target_gen = 0
        self._pending_reacquire = None
        threading.Thread(target=self._reid_worker_loop, daemon=True).start()
        # Whether debug annotation is drawn this tick (someone is watching).
        self._draw_debug = False
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

    def _rotation_callback(self, msg):
        """Mirror vision.camera_upside_down(): only 0 and 180 are meaningful for
        this node (it rotates by 180 deg). 90/270 are logged and treated as 0."""
        value = int(msg.data) % 360
        if value not in (0, 180):
            self.get_logger().warn(
                f"Camera rotation {value} not supported by tracker (only 0/180); "
                f"treating as 0"
            )
        flip = value == 180
        if flip != self.flip_image:
            self.flip_image = flip
            self.get_logger().info(
                f"Camera rotation {value} -> flip_image={self.flip_image}"
            )

    def _to_raw_coords(self, x, y, width, height):
        """Map a pixel from the (possibly flipped-for-detection) color frame back to
        the RAW camera frame. The tracker flips only the COLOR frame 180 deg for
        upright detection; depth is never rotated, so every depth sample and the 3D
        deprojection must use raw pixels or the point comes out mirrored."""
        if self.flip_image:
            return (width - 1 - x, height - 1 - y)
        return (x, y)

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
        self.image_publisher.publish(self.output_image)

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
        self.person_data["embeddings"] = []
        self.person_data["num_embeddings"] = 0
        self.person_data["forward"] = None
        self.person_data["backward"] = None
        self.person_data["left"] = None
        self.person_data["right"] = None
        self.last_depth = None
        self._depth_reject_count = 0
        # Invalidate any queued/pending ReID work for the previous target.
        self._target_gen += 1
        self._pending_reacquire = None

    # ----------------------------------------------------------- re-acquisition
    def _ensure_reid_model(self):
        """True only once the SWIN model is loaded. The load takes several seconds,
        so it runs in a BACKGROUND thread — doing it inline here would block the
        tracker timer, stall RESULTS_TOPIC, and make person_goal_smoother miss the
        follow goal (the base then never moves). Until it finishes, re-acquisition is
        simply unavailable; ByteTrack tracking keeps running uninterrupted."""
        if self.model_reid is not None:
            return True
        if self.reid_failed:
            return False
        if not self._reid_loading:
            self._reid_loading = True
            threading.Thread(target=self._load_reid_model, daemon=True).start()
        return False

    def _load_reid_model(self):
        """Background worker: build + load the SWIN ReID model. Best-effort — on
        failure latch reid_failed and fall back to ByteTrack-only tracking."""
        try:
            self.get_logger().info("Loading ReID model (SWIN) in background ...")
            model = load_network(get_structure())
            model.classifier.classifier = torch.nn.Sequential()
            model.eval()
            if torch.cuda.is_available():
                model = model.cuda()
            self.model_reid = model
            self.get_logger().info("ReID model ready")
        except Exception as e:
            self.get_logger().error(f"ReID load failed ({e}); re-acquisition disabled")
            self.reid_failed = True
        finally:
            self._reid_loading = False

    def _embed_crop(self, crop_bgr):
        """Normalized ReID embedding (1-D tensor) for a BGR person crop, or None."""
        if crop_bgr is None or crop_bgr.size == 0:
            return None
        if not self._ensure_reid_model():
            return None
        try:
            pil = PILImage.fromarray(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB))
            with torch.no_grad():
                feat = extract_feature_from_img(pil, self.model_reid)
            return feat.flatten().float()
        except Exception as e:
            self.get_logger().warn(f"ReID embed failed: {e}")
            return None

    def _reid_worker_loop(self):
        """Background consumer for ALL ReID inference. A SWIN forward is 30-100 ms
        on the Orin; running it here (instead of on the tracking timer, where it
        used to run inline every REID_EXTRACT_FREQ) keeps the tick at detector
        latency so RESULTS_TOPIC/centroid never stall. Jobs stamped with an old
        target generation are dropped — they belong to a previous target."""
        while True:
            kind, gen, payload = self._reid_jobs.get()
            try:
                if gen != self._target_gen:
                    continue
                if kind == "gallery":
                    feat = self._embed_crop(payload)
                    if feat is None or gen != self._target_gen:
                        continue
                    gallery = self.person_data["embeddings"]
                    gallery.append(feat)
                    if len(gallery) > MAX_EMBEDDINGS:
                        gallery.pop(0)
                    self.person_data["num_embeddings"] = len(gallery)
                elif kind == "reacquire":
                    match = self._match_gallery(payload)
                    if gen == self._target_gen:
                        self._pending_reacquire = match
            except Exception as e:
                self.get_logger().warn(f"ReID worker error: {e}")

    def _match_gallery(self, crops):
        """Worker-side: best gallery match among [(track_id, crop), ...]; returns
        the track id if its similarity clears REID_MATCH_THRESHOLD, else None."""
        best_id, best_sim = None, 0.0
        for tid, crop in crops:
            feat = self._embed_crop(crop)
            if feat is None:
                continue
            sim = self._max_similarity(feat)
            if sim > best_sim:
                best_sim, best_id = sim, tid
        if best_id is not None and best_sim >= REID_MATCH_THRESHOLD:
            self.get_logger().info(
                f"ReID matched lost target to track {best_id} (sim={best_sim:.2f})"
            )
            return best_id
        return None

    def _store_target_embedding(self, crop_bgr):
        """While the target is visible, queue its appearance for the gallery
        (throttled to REID_EXTRACT_FREQ, capped at MAX_EMBEDDINGS in the worker).
        The crop is copied because self.frame is reused next tick."""
        if time.time() - self.last_reid_extraction < REID_EXTRACT_FREQ:
            return
        if crop_bgr is None or crop_bgr.size == 0:
            return
        if not self._ensure_reid_model():
            return
        self.last_reid_extraction = time.time()
        try:
            self._reid_jobs.put_nowait(("gallery", self._target_gen, crop_bgr.copy()))
        except queue.Full:
            pass

    def _max_similarity(self, feat):
        """Best cosine similarity of feat against the gallery (embeddings are already
        unit-normalized, so a dot product is the cosine). 0.0 if the gallery is empty."""
        gallery = self.person_data["embeddings"]
        if not gallery:
            return 0.0
        g = torch.stack([t.to(feat.device) for t in gallery])
        return float(torch.matmul(g, feat).max().item())

    def _try_reacquire(self, tracked_people, frame):
        """The locked ByteTrack id is gone but other people are visible. Matching
        runs on the ReID worker: this consumes a finished match (if its track id is
        still live) and enqueues a new matching job, throttled. Never blocks."""
        # Consume a match the worker finished since the last tick.
        pending = self._pending_reacquire
        if pending is not None:
            self._pending_reacquire = None
            if any(p["track_id"] == pending for p in tracked_people):
                self.get_logger().info(f"Re-acquired target as track {pending}")
                return pending
        if not self.person_data["embeddings"]:
            return None
        if time.time() - self.last_reacq_attempt < REID_REACQUIRE_FREQ:
            return None
        self.last_reacq_attempt = time.time()
        crops = [
            (
                p["track_id"],
                frame[p["y1"] : p["y2"], p["x1"] : p["x2"]].copy(),
            )
            for p in tracked_people
        ]
        try:
            self._reid_jobs.put_nowait(("reacquire", self._target_gen, crops))
        except queue.Full:
            pass
        return None

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
        self.person_data["embeddings"] = []
        self.person_data["num_embeddings"] = 0
        self.last_reacq_attempt = 0.0
        self.last_depth = None
        self._depth_reject_count = 0
        # New target: invalidate queued/pending ReID work from the previous one.
        self._target_gen += 1
        self._pending_reacquire = None

        self.frame = self.image.copy()

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
                        self.customer_image = cropped_image.copy()
                    else:
                        self.get_logger().warn(
                            f"Person detected with {track_by}: {response_clean} but not {value}"
                        )

        if largest_person["id"] is not None:
            self.person_data["id"] = largest_person["id"]
            self.success(f"Target set: {largest_person['id']}")
            # Kick off the ReID-model load NOW (non-blocking, background thread) so it
            # is ready soon after follow starts, and force the first gallery extraction
            # on the next run tick. The load never blocks the service or the tracker.
            self.last_reid_extraction = 0.0
            self._ensure_reid_model()
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

        if self.image is None:
            self.get_logger().error("No image available")
            return
        self.frame = self.image.copy()

        if self.flip_image:
            self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)

        # Annotation costs a full-frame copy + cv2 draws every tick; skip all of
        # it unless something is actually watching the debug topics.
        self._draw_debug = self.image_publisher.has_subscribers()
        self.output_image = self.frame.copy() if self._draw_debug else []

        # Per-frame detection + ByteTrack (motion-only, no per-frame ReID) — Stage 0
        start_time = time.time()
        tracked_people = self._track(self.frame)
        self.get_logger().info(
            f"Det+Tracking took {time.time() - start_time:.2f}s | People: {len(tracked_people)}",
            throttle_duration_sec=2.0,
        )

        if self.person_data["id"] is None:
            self.frame = None
            return

        person_in_frame = False

        for person in tracked_people:
            x1, y1, x2, y2 = person["x1"], person["y1"], person["x2"], person["y2"]
            track_id = person["track_id"]

            is_target = track_id == self.person_data["id"]
            if is_target:
                person_in_frame = True
                self.person_data["coordinates"] = (x1, y1, x2, y2)

            if self._draw_debug:
                color = (0, 255, 0) if is_target else (255, 0, 0)
                cv2.rectangle(self.output_image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    self.output_image,
                    f"person {track_id}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2,
                )

        # Appearance ReID (Stage 1) closes the gap where ByteTrack drops the track-id
        # after a real loss (occlusion / out-of-frame) and assigns a NEW id when the
        # person reappears — without this the operator could never be re-followed.
        if person_in_frame:
            # Visible: keep refreshing the target's appearance gallery (throttled).
            tx1, ty1, tx2, ty2 = self.person_data["coordinates"]
            self._store_target_embedding(self.frame[ty1:ty2, tx1:tx2])
        elif tracked_people:
            # Lost the locked id but people are visible -> ReID-match them to the
            # gallery and re-lock onto the best match.
            new_id = self._try_reacquire(tracked_people, self.frame)
            if new_id is not None:
                self.person_data["id"] = new_id
                self.last_depth = None  # depth continuity broke across the loss
                self._depth_reject_count = 0
                for person in tracked_people:
                    if person["track_id"] == new_id:
                        x1, y1, x2, y2 = (
                            person["x1"],
                            person["y1"],
                            person["x2"],
                            person["y2"],
                        )
                        self.person_data["coordinates"] = (x1, y1, x2, y2)
                        if self._draw_debug:
                            cv2.rectangle(
                                self.output_image, (x1, y1), (x2, y2), (0, 255, 255), 3
                            )
                        person_in_frame = True
                        break

        if not person_in_frame:
            self.is_tracking_result = False
            self.frame = None
            return

        self.is_tracking_result = True
        frame_h, frame_w = self.frame.shape[:2]
        x1, y1, x2, y2 = self.person_data["coordinates"]
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        # Arm-follow centroid: the person's horizontal offset in the UPRIGHT
        # detection frame, normalized to [-1, 1]. It needs NO depth, so publish it
        # for EVERY tracked frame (it used to sit behind the depth gate below —
        # a depth hiccup froze the arm, the person left the FOV, follow lost them).
        point2Dpoint = Point()
        point2Dpoint.x = float(cx / (frame_w / 2) - 1.0)
        point2Dpoint.y = 0.0
        point2Dpoint.z = 0.0
        self.centroid_publisher.publish(point2Dpoint)

        if self.depth_image_time is None or self.image_time is None:
            self.get_logger().warn(
                f"Depth image not available (depth_time={self.depth_image_time is not None}, "
                f"image_time={self.image_time is not None})",
                throttle_duration_sec=2.0,
            )
            self.frame = None
            return
        depth_stamp_ns = (
            self.depth_image_time.sec * 1_000_000_000
            + self.depth_image_time.nanosec
        )
        image_stamp_ns = self.image_time.sec * 1_000_000_000 + self.image_time.nanosec
        stamp_diff_ns = abs(depth_stamp_ns - image_stamp_ns)
        if len(self.depth_image) == 0 or stamp_diff_ns >= DEPTH_THRESHOLD_NS:
            self.get_logger().warn(
                f"Depth/RGB not paired (diff {stamp_diff_ns / 1e6:.0f} ms) — "
                f"skipping 3D point this tick",
                throttle_duration_sec=2.0,
            )
            self.frame = None
            return

        # Depth + 3D use the RAW camera frame. The color frame may be flipped
        # 180 deg for upright detection, but depth is never rotated, so map the
        # detection-frame bbox/centroid back to raw pixels first — otherwise
        # the depth ROI and the deprojected point come out mirrored.
        rx1, ry1 = self._to_raw_coords(x1, y1, frame_w, frame_h)
        rx2, ry2 = self._to_raw_coords(x2, y2, frame_w, frame_h)
        rx1, rx2 = sorted((rx1, rx2))
        ry1, ry2 = sorted((ry1, ry2))

        # Robust depth: median from inner 50% of the (raw) bounding box
        bw, bh = rx2 - rx1, ry2 - ry1
        inner_x1 = rx1 + bw // 4
        inner_x2 = rx2 - bw // 4
        inner_y1 = ry1 + bh // 4
        inner_y2 = ry2 - bh // 4
        roi = self.depth_image[inner_y1:inner_y2, inner_x1:inner_x2]
        valid = roi[np.isfinite(roi) & (roi > 0.0)]
        if valid.size == 0:
            self.frame = None
            return
        depth = float(np.median(valid))

        # Reject sudden depth jumps — but only DEPTH_JUMP_MAX_REJECTS times in a
        # row. A persistent "jump" means the person really is at the new depth
        # (they kept walking while publishes were skipped); without the counter
        # one legit move >threshold rejected every frame after it forever, and
        # the smoother's watchdog halted the base on a person we still tracked.
        if (
            self.last_depth is not None
            and abs(depth - self.last_depth) > DEPTH_JUMP_THRESHOLD
            and self._depth_reject_count < DEPTH_JUMP_MAX_REJECTS
        ):
            self._depth_reject_count += 1
            self.frame = None
            return
        self._depth_reject_count = 0
        self.last_depth = depth

        coords = PointStamped()
        coords.header.frame_id = self.frame_id
        coords.header.stamp = self.depth_image_time
        raw_cx, raw_cy = self._to_raw_coords(cx, cy, frame_w, frame_h)
        point3D = deproject_pixel_to_point(self.imageInfo, (raw_cx, raw_cy), depth)
        coords.point.x = float(point3D[0])
        coords.point.y = float(point3D[1])
        coords.point.z = float(point3D[2])
        self.results_publisher.publish(coords)

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
