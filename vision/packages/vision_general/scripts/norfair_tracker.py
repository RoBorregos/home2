#!/usr/bin/env python3
"""
Node for tracking one person using Norfair and re-id if it's necessary
Use YOLO for detect person and Norfair to track the person with 2 points per bounding box
"""

import time
import copy
from typing import List, Dict, Optional, Tuple
import cv2
import numpy as np
from ultralytics import YOLO
from norfair import Detection, Tracker

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.time import Time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from std_srvs.srv import SetBool, Trigger

from PIL import Image as PILImage
import torch
import torch.nn as nn

# Utils
from vision_general.utils.calculations import (
    get2DCentroid,
    get_depth,
    deproject_pixel_to_point,
)

# Person ReID 
from vision_general.utils.reid_model import (
    load_network,
    extract_feature_from_img,
    get_structure,
)

# Constants
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    TRACKER_IMAGE_TOPIC,
    RESULTS_TOPIC,
    CENTROID_TOIC,
    SET_TARGET_TOPIC,
)

CONF_THRESHOLD = 0.60
SYNC_TOL_MS = 30.0


class NorfairSingleTracker(Node):
    def __init__(self):
        super().__init__("norfair_single_tracker_node")

        self.bridge = CvBridge()
        self.callback_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # Subscribers
        self.image_sub = self.create_subscription(Image, CAMERA_TOPIC, self.image_cb, qos)
        self.depth_sub = self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_cb, qos)
        self.info_sub = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.info_cb, qos)

        # Publishers
        self.debug_img_pub = self.create_publisher(Image, TRACKER_IMAGE_TOPIC, 10)
        self.results_pub = self.create_publisher(PointStamped, RESULTS_TOPIC, 10)
        self.centroid_pub = self.create_publisher(Point, CENTROID_TOIC, 10)

        # Services
        self.set_target_srv = self.create_service(SetBool, SET_TARGET_TOPIC, self.set_target_cb)
        self.is_tracking_srv = self.create_service(Trigger, "/vision/is_tracking", self.is_tracking_cb)

        # Parameters
        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("use_classes_filter", True)   # True -> classes=0 for COCO person
        self.declare_parameter("classes", 0)

        # Norfair (2-point) 
        """ 
        Uses 2 point to avoid confusion between two people who are very close together, (center and top) 
        """
        self.declare_parameter("norfair_distance_threshold", 65.0)
        self.declare_parameter("norfair_hit_counter_max", 25)
        self.declare_parameter("norfair_init_delay", 2)

        # Debug
        self.declare_parameter("publish_debug_image", True)

        # Reacquire by nearest not largest 
        self.declare_parameter("reacquire_max_px", 140.0)  # max jump in pixels for nearest reacquire

        # Person ReID
        self.declare_parameter("reid_enable", True)
        self.declare_parameter("reid_period_s", 0.35)      
        self.declare_parameter("reid_threshold", 0.70)     
        self.declare_parameter("reid_gallery_max", 32)

        self.yolo_model_path = self.get_parameter("yolo_model").value
        self.use_classes_filter = bool(self.get_parameter("use_classes_filter").value)
        self.classes = self.get_parameter("classes").value

        self.reacquire_max_px = float(self.get_parameter("reacquire_max_px").value)

        self.reid_enable = bool(self.get_parameter("reid_enable").value)
        self.reid_period_s = float(self.get_parameter("reid_period_s").value)
        self.reid_threshold = float(self.get_parameter("reid_threshold").value)
        self.reid_gallery_max = int(self.get_parameter("reid_gallery_max").value)

        # Load YOLO
        self.model = YOLO(self.yolo_model_path)
        self.get_logger().info(f"Loaded YOLO model: {self.yolo_model_path}")

        # Norfair tracker (2-point distance)
        """
        Initialize Norfair using a custom distance function that compares TWO points per detection:
        - center of the bounding box
         - top-center of the bounding box
        """
        self.norfair_tracker = Tracker(
            distance_function=self._two_point_distance, # average Euclidean distance of the 2 points
            distance_threshold=float(self.get_parameter("norfair_distance_threshold").value),
            hit_counter_max=int(self.get_parameter("norfair_hit_counter_max").value),
            initialization_delay=int(self.get_parameter("norfair_init_delay").value),
        )
        self.get_logger().info("Norfair 2-point tracker")

        # Load Person ReID model
        """ 
        Used for re-identification when the tracker loses the target
        """
        self.model_reid = None
        self.use_gpu = torch.cuda.is_available()

        if self.reid_enable:
            structure = get_structure()
            self.model_reid = load_network(structure)

            # Remove classifier head 
            if hasattr(self.model_reid, "classifier") and hasattr(self.model_reid.classifier, "classifier"):
                self.model_reid.classifier.classifier = nn.Sequential()

            if self.use_gpu:
                self.model_reid = self.model_reid.cuda()

            self.model_reid.eval()
            self.get_logger().info(f"ReID model (GPU={self.use_gpu})")

        # State
        self.image_bgr: Optional[np.ndarray] = None
        self.depth_img: Optional[np.ndarray] = None
        self.image_stamp = None
        self.depth_stamp = None
        self.cam_info: Optional[CameraInfo] = None

        self.frame_id = "zed_left_camera_optical_frame"

        self.target_set = False
        self.target_id: Optional[int] = None
        self.target_bbox: Optional[Tuple[int, int, int, int]] = None

        self.is_tracking_result = False
        self.debug_frame: Optional[np.ndarray] = None

        # For nearest reacquire
        self.last_target_center: Optional[Tuple[float, float]] = None
        self.last_target_bbox: Optional[Tuple[int, int, int, int]] = None

        # ReID gallery
        self.last_reid_time = 0.0
        self.target_gallery: List[np.ndarray] = []  # list of L2-normalized embeddings (CPU)

        # Timers
        self.create_timer(0.10, self.run)
        self.create_timer(0.10, self.publish_debug_image)

        self.get_logger().info("NorfairSingleTracker")

    # ROS Callbacks
    def image_cb(self, msg: Image):
        self.image_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.image_stamp = msg.header.stamp

    def depth_cb(self, msg: Image):
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f"Depth decode error: {e}")

    def info_cb(self, msg: CameraInfo):
        self.cam_info = msg

    def is_tracking_cb(self, request, response):
        response.success = bool(self.is_tracking_result)
        response.message = "tracking" if self.is_tracking_result else "not_tracking"
        return response

    def set_target_cb(self, request: SetBool.Request, response: SetBool.Response):
        self.target_set = bool(request.data)

        if self.target_set:
            ok = self.set_target_largest()
            response.success = bool(ok)
            response.message = "target_set" if ok else "failed_to_set_target"
        else:
            self.target_id = None
            self.target_bbox = None
            self.last_target_center = None
            self.last_target_bbox = None
            self.is_tracking_result = False
            response.success = True
            response.message = "tracking_disabled"

        return response

    # YOLO Detection
    def _yolo_detect(self, frame_bgr: np.ndarray) -> List[Dict]:
        """
        Returns list of detections
        """
        classes_arg = self.classes if self.use_classes_filter else None

        results = self.model.predict(
            frame_bgr,
            conf=CONF_THRESHOLD,
            classes=classes_arg,
            verbose=False,
        )

        bboxes: List[Dict] = []
        for out in results:
            for box in out.boxes:
                conf = float(box.conf[0].item())
                if conf < CONF_THRESHOLD:
                    continue
                x1, y1, x2, y2 = [int(round(x)) for x in box.xyxy[0].tolist()]
                bboxes.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2, "conf": conf})
        return bboxes

    # Norfair 2-point detection and distance
    def _bboxes_to_norfair(self, bboxes: List[Dict]) -> List[Detection]:
        """
        2-point representation
          p0 = bbox center
          p1 = bbox top-center
        """
        detections: List[Detection] = []
        for b in bboxes:
            x1, y1, x2, y2 = b["x1"], b["y1"], b["x2"], b["y2"]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            tcx = cx
            tcy = float(y1)

            points = np.array([[cx, cy], [tcx, tcy]], dtype=np.float32)

            detections.append(
                Detection(
                    points=points,
                    scores=np.array([b["conf"], b["conf"]], dtype=np.float32),
                )
            )
        return detections

    def _two_point_distance(self, detection: Detection, tracked_obj) -> float:
        """
        Average Euclidean distance over the 2 points.
        """
        det_pts = detection.points
        trk_pts = tracked_obj.estimate

        # Fallback if shapes mismatch
        if det_pts is None or trk_pts is None:
            return 1e9
        if det_pts.shape != trk_pts.shape:
            det_pts = det_pts[:1]
            trk_pts = trk_pts[:1]

        d = np.linalg.norm(det_pts - trk_pts, axis=1)  # per-point distances
        return float(np.mean(d))

    def _bbox_area(self, b: Dict) -> int:
        return max(0, b["x2"] - b["x1"]) * max(0, b["y2"] - b["y1"])

    def _bbox_center(self, b: Dict) -> Tuple[float, float]:
        return ((b["x1"] + b["x2"]) / 2.0, (b["y1"] + b["y2"]) / 2.0)

    def _match_track_to_bbox(self, track_obj, bboxes: List[Dict]) -> Optional[Dict]:
        """
        It gives track_obj.estimate (2 points) and match using the CENTER point estimate
        """
        if not bboxes:
            return None

        ex, ey = track_obj.estimate[0]  # center point
        best = None
        best_d = 1e18

        for b in bboxes:
            cx, cy = self._bbox_center(b)
            d = (cx - ex) ** 2 + (cy - ey) ** 2
            if d < best_d:
                best_d = d
                best = b

        return best

    # Target selection (largest)
    def set_target_largest(self) -> bool:
        """
        Select target based on largest bounding box area among current tracks.
        """
        if self.image_bgr is None:
            self.get_logger().warn("No image")
            self.is_tracking_result = False
            return False

        frame = copy.deepcopy(self.image_bgr)
        bboxes = self._yolo_detect(frame)
        detections = self._bboxes_to_norfair(bboxes)
        tracks = self.norfair_tracker.update(detections)

        if not tracks or not bboxes:
            self.get_logger().warn("No detections")
            self.is_tracking_result = False
            return False

        best_id = None
        best_bbox = None
        best_area = -1

        for t in tracks:
            b = self._match_track_to_bbox(t, bboxes)
            if b is None:
                continue
            area = self._bbox_area(b)
            if area > best_area:
                best_area = area
                best_id = int(t.id)
                best_bbox = (b["x1"], b["y1"], b["x2"], b["y2"])

        if best_id is None:
            self.get_logger().warn("Failed")
            self.is_tracking_result = False
            return False

        self.target_id = best_id
        self.target_bbox = best_bbox

        # Initialize last center/bounding box for nearest reacquire
        x1, y1, x2, y2 = best_bbox
        self.last_target_center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        self.last_target_bbox = best_bbox

        self.get_logger().info(f"Target set ID: {self.target_id}")
        return True

    # Re-acquire: nearest to last bounding box center 
    def _reacquire_by_nearest(self, tracks, bboxes) -> bool:
        """
        Reacquire target by choosing the track whose matched bounding box center is
        closest to last_target_center not largest
        """
        if self.last_target_center is None:
            return False
        if not tracks or not bboxes:
            return False

        lx, ly = self.last_target_center
        best_id = None
        best_bbox = None
        best_dist = 1e18

        for t in tracks:
            b = self._match_track_to_bbox(t, bboxes)
            if b is None:
                continue
            cx, cy = self._bbox_center(b)
            dist = ((cx - lx) ** 2 + (cy - ly) ** 2) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best_id = int(t.id)
                best_bbox = (b["x1"], b["y1"], b["x2"], b["y2"])

        if best_id is None:
            return False

        if best_dist > self.reacquire_max_px:
            return False

        self.target_id = best_id
        self.target_bbox = best_bbox
        self.get_logger().info(f"Reacquired by nearest: ID={best_id}, dist={best_dist:.1f}px")
        return True

    # Person ReID (embeddings from person bbox crop)
    def _extract_person_embedding(self, crop_bgr: np.ndarray) -> Optional[np.ndarray]:
        """
        Extract person embedding from a BGR crop (full body bounding box).
        Returns L2-normalized embedding.
        """
        if (not self.reid_enable) or (self.model_reid is None):
            return None
        if crop_bgr is None or crop_bgr.size == 0:
            return None

        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(crop_rgb)

        with torch.no_grad():
            emb_t = extract_feature_from_img(pil_img, self.model_reid)
            if emb_t is None:
                return None

            emb_t = emb_t.squeeze()

            if isinstance(emb_t, torch.Tensor):
                emb_t = emb_t.detach()
                if emb_t.is_cuda:
                    emb_t = emb_t.cpu()
                emb = emb_t.numpy()
            else:
                emb = np.array(emb_t)

        emb = emb.astype(np.float32)

        norm = np.linalg.norm(emb) + 1e-9
        emb = emb / norm
        return emb

    def _cosine_sim(self, a: np.ndarray, b: np.ndarray) -> float:
        # If both are L2-normalized, cosine similarity = dot product
        return float(np.dot(a, b))

    def _best_gallery_sim(self, emb: np.ndarray) -> float:
        if emb is None or len(self.target_gallery) == 0:
            return -1.0
        return max(self._cosine_sim(emb, g) for g in self.target_gallery)

    def _reacquire_by_reid(self, tracks, bboxes, frame_bgr) -> bool:
        """
        Reacquire by selecting track whose person crop best matches the target gallery
        """
        if not self.reid_enable or len(self.target_gallery) == 0:
            return False
        if not tracks or not bboxes:
            return False

        best_id = None
        best_bbox = None
        best_sim = -1.0

        for t in tracks:
            b = self._match_track_to_bbox(t, bboxes)
            if b is None:
                continue

            x1, y1, x2, y2 = b["x1"], b["y1"], b["x2"], b["y2"]
            crop = frame_bgr[y1:y2, x1:x2]  

            emb = self._extract_person_embedding(crop)
            if emb is None:
                continue

            sim = self._best_gallery_sim(emb)
            if sim > best_sim:
                best_sim = sim
                best_id = int(t.id)
                best_bbox = (x1, y1, x2, y2)

        if best_id is None:
            return False
        if best_sim < self.reid_threshold:
            return False

        self.target_id = best_id
        self.target_bbox = best_bbox
        self.get_logger().info(f"Reacquired by ReID: ID={best_id}, sim={best_sim:.3f}")
        return True

    # Main 
    def run(self):
        if not self.target_set:
            self.is_tracking_result = False
            return

        if self.image_bgr is None:
            self.is_tracking_result = False
            return

        frame = copy.deepcopy(self.image_bgr)
        self.debug_frame = frame.copy()

        # Detect
        t0 = time.time()
        bboxes = self._yolo_detect(frame)
        det_dt = time.time() - t0

        # Track (Norfair 2-point)
        detections = self._bboxes_to_norfair(bboxes)
        tracks = self.norfair_tracker.update(detections)

        self.get_logger().info(
            f"YOLO det: {det_dt:.2f}s | tracks: {len(tracks)} | dets: {len(bboxes)}"
        )

        # If target not set yet, auto set largest
        if self.target_id is None:
            if not self.set_target_largest():
                self.is_tracking_result = False
                cv2.putText(self.debug_frame, "NO TARGET", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                return

        # Find target track by ID
        target_track = None
        for t in tracks:
            if int(t.id) == int(self.target_id):
                target_track = t
                break

        # If target lost: reacquire nearest -> reid -> largest
        if target_track is None:
            self.is_tracking_result = False

            if self._reacquire_by_nearest(tracks, bboxes):
                return

            if self._reacquire_by_reid(tracks, bboxes, frame):
                return

            if self.set_target_largest():
                return

            cv2.putText(self.debug_frame, "TARGET LOST", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            return

        # Match target track to bounding box
        b = self._match_track_to_bbox(target_track, bboxes)
        if b is None:
            self.is_tracking_result = False
            return

        x1, y1, x2, y2 = b["x1"], b["y1"], b["x2"], b["y2"]
        self.target_bbox = (x1, y1, x2, y2)
        self.is_tracking_result = True

        # Update last known center/bounding box for nearest reacquire
        self.last_target_bbox = self.target_bbox
        self.last_target_center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

        # Draw all detections
        for bb in bboxes:
            cv2.rectangle(self.debug_frame, (bb["x1"], bb["y1"]), (bb["x2"], bb["y2"]), (255, 0, 0), 2)

        # Highlight target 
        cv2.rectangle(self.debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.putText(self.debug_frame, f"TARGET ID: {self.target_id}", (x1, max(0, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Update ReID gallery periodically 
        if self.reid_enable and (time.time() - self.last_reid_time) > self.reid_period_s:
            self.last_reid_time = time.time()
            person_crop = frame[y1:y2, x1:x2]
            emb = self._extract_person_embedding(person_crop)
            if emb is not None:
                self.target_gallery.append(emb)
                if len(self.target_gallery) > self.reid_gallery_max:
                    self.target_gallery = self.target_gallery[-self.reid_gallery_max:]
                cv2.putText(self.debug_frame, f"ReID gallery: {len(self.target_gallery)}",
                            (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # Publish centroid normalized X and 3D point 
        if self.depth_img is None or self.depth_stamp is None or self.image_stamp is None:
            self.get_logger().warn("Depth not available")
            return

        dt_ns = (Time.from_msg(self.depth_stamp) - Time.from_msg(self.image_stamp)).nanoseconds
        dt_ms = abs(dt_ns) / 1e6
        if dt_ms > SYNC_TOL_MS:
            self.get_logger().warn(f"Depthand RGB not synced (dt={dt_ms:.1f}ms)")
            return

        if self.cam_info is None:
            self.get_logger().warn("Camera not available")
            return

        # 2D centroid in pixel coordinates  (row, col)
        point2D = get2DCentroid(self.target_bbox, self.depth_img)
        px_col = float(point2D[1])

        # Normalize X to [-1, 1]
        norm_x = (px_col / (frame.shape[1] / 2.0)) - 1.0

        centroid_msg = Point()
        centroid_msg.x = float(norm_x)
        centroid_msg.y = 0.0
        centroid_msg.z = 0.0
        self.centroid_pub.publish(centroid_msg)

        # 3D deprojection
        depth = get_depth(self.depth_img, point2D)
        point_2d_uv = (point2D[1], point2D[0])  # (u,v)=(col,row)
        p3 = deproject_pixel_to_point(self.cam_info, point_2d_uv, depth)

        coords = PointStamped()
        coords.header.frame_id = self.frame_id
        coords.header.stamp = self.depth_stamp
        coords.point.x = float(p3[0])
        coords.point.y = float(p3[1])
        coords.point.z = float(p3[2])
        self.results_pub.publish(coords)

    # Debug image publishing
    def publish_debug_image(self):
        if not bool(self.get_parameter("publish_debug_image").value):
            return
        if self.debug_frame is None:
            return
        msg = self.bridge.cv2_to_imgmsg(self.debug_frame, "bgr8")
        self.debug_img_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NorfairSingleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()