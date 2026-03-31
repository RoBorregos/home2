#!/usr/bin/env python3

import cv2
import numpy as np
import os
from frida_constants.vision_enums import Gestures
from math import degrees, acos
from ultralytics import YOLO

# ── YOLO COCO keypoint indices ──
NOSE = 0
LEFT_SHOULDER = 5
RIGHT_SHOULDER = 6
LEFT_ELBOW = 7
RIGHT_ELBOW = 8
LEFT_WRIST = 9
RIGHT_WRIST = 10
LEFT_HIP = 11
RIGHT_HIP = 12
LEFT_KNEE = 13
RIGHT_KNEE = 14
LEFT_ANKLE = 15
RIGHT_ANKLE = 16

KP_CONF = 0.3


def load_yolo_pose(model_name="yolo11m-pose.pt"):
    """Load YOLO pose model with automatic TensorRT export for Orin AGX."""
    engine_path = model_name.replace(".pt", ".engine")
    if os.path.exists(engine_path):
        print(f"[PoseDetection] Loading TensorRT engine: {engine_path}")
        return YOLO(engine_path, task="pose")

    print(f"[PoseDetection] Loading YOLO pose model: {model_name}")
    model = YOLO(model_name)
    try:
        print(
            "[PoseDetection] Exporting to TensorRT (first run only, may take a few minutes)..."
        )
        model.export(format="engine", half=True, device=0, imgsz=640)
        print(f"[PoseDetection] TensorRT engine saved: {engine_path}")
        return YOLO(engine_path, task="pose")
    except Exception as e:
        print(f"[PoseDetection] TensorRT export failed ({e}), using PyTorch model")
        return model


class PoseDetection:
    def __init__(self):
        print("Pose Detection Ready (YOLO TensorRT)")
        self.yolo_pose = load_yolo_pose("yolo11m-pose.pt")

    # ── Core keypoint extraction ──

    def _get_keypoints(self, image):
        """Run YOLO pose on image, return (points, conf) for first person.
        points: (17, 2) normalized coords, conf: (17,) confidence scores.
        Returns (None, None) if no person detected."""
        results = self.yolo_pose(image, verbose=False)
        if (
            not results
            or results[0].keypoints is None
            or results[0].keypoints.xy is None
            or len(results[0].keypoints.xy) == 0
        ):
            return None, None

        points = results[0].keypoints.xyn[0].cpu().numpy()
        conf = (
            results[0].keypoints.conf[0].cpu().numpy()
            if results[0].keypoints.conf is not None
            else np.ones(17, dtype=np.float32)
        )
        return points, conf

    def detect(self, frame):
        """Run pose detection, return raw YOLO results."""
        return self.yolo_pose(frame, verbose=False)

    # ── Geometry helpers ──

    def get_angle(self, p1, p2, p3):
        """Angle at p2 in the triangle p1-p2-p3 (degrees)."""
        p1, p2, p3 = np.array(p1[:2]), np.array(p2[:2]), np.array(p3[:2])
        l1, l2, l3 = (
            np.linalg.norm(p2 - p3),
            np.linalg.norm(p1 - p3),
            np.linalg.norm(p1 - p2),
        )
        denom = 2 * l1 * l2
        if denom == 0:
            return 180.0
        return abs(degrees(acos(np.clip((l1**2 + l2**2 - l3**2) / denom, -1.0, 1.0))))

    # ── Visibility checks ──

    def is_chest_visible(self, image):
        points, conf = self._get_keypoints(image)
        if points is None:
            return False
        return conf[LEFT_SHOULDER] > 0.5 and conf[RIGHT_SHOULDER] > 0.5

    def are_arms_down(self, points, conf):
        if (
            conf[LEFT_WRIST] < KP_CONF
            or conf[RIGHT_WRIST] < KP_CONF
            or conf[LEFT_SHOULDER] < KP_CONF
            or conf[RIGHT_SHOULDER] < KP_CONF
        ):
            return True
        left_arm_down = points[LEFT_WRIST][1] > points[LEFT_SHOULDER][1] + 0.1
        right_arm_down = points[RIGHT_WRIST][1] > points[RIGHT_SHOULDER][1] + 0.1
        return left_arm_down and right_arm_down

    # ── Gesture detection ──

    def detectGesture(self, image):
        """Detect gesture using YOLO pose keypoints (TensorRT accelerated)."""
        points, conf = self._get_keypoints(image)
        gesture = Gestures.UNKNOWN

        if points is None:
            return gesture

        if conf[LEFT_SHOULDER] < 0.5 or conf[RIGHT_SHOULDER] < 0.5:
            return gesture

        mid_x = (points[LEFT_SHOULDER][0] + points[RIGHT_SHOULDER][0]) / 2

        if self.are_arms_down(points, conf):
            return gesture

        if self._is_raising_left_arm(mid_x, points, conf):
            gesture = Gestures.RAISING_LEFT_ARM
        elif self._is_pointing_left(mid_x, points, conf):
            gesture = Gestures.POINTING_LEFT
        elif self._is_raising_right_arm(mid_x, points, conf):
            gesture = Gestures.RAISING_RIGHT_ARM
        elif self._is_pointing_right(mid_x, points, conf):
            gesture = Gestures.POINTING_RIGHT
        elif self._is_waving(points, conf):
            gesture = Gestures.WAVING

        return gesture

    def is_waving_customer(self, image):
        """Check if person has one hand raised (waving)."""
        points, conf = self._get_keypoints(image)
        if points is None:
            return False
        try:
            if (
                points[RIGHT_WRIST][1] < points[RIGHT_SHOULDER][1]
                and points[LEFT_WRIST][1] > points[LEFT_SHOULDER][1]
            ):
                return True
            if (
                points[LEFT_WRIST][1] < points[LEFT_SHOULDER][1]
                and points[RIGHT_WRIST][1] > points[RIGHT_SHOULDER][1]
            ):
                return True
            return False
        except Exception:
            return False

    # ── Gesture sub-checks ──

    def _is_waving(self, points, conf):
        left_angle = 0
        if (
            conf[LEFT_SHOULDER] >= KP_CONF
            and conf[LEFT_ELBOW] >= KP_CONF
            and conf[LEFT_WRIST] >= KP_CONF
        ):
            left_angle = self.get_angle(
                points[LEFT_SHOULDER], points[LEFT_ELBOW], points[LEFT_WRIST]
            )

        right_angle = 0
        if (
            conf[RIGHT_SHOULDER] >= KP_CONF
            and conf[RIGHT_ELBOW] >= KP_CONF
            and conf[RIGHT_WRIST] >= KP_CONF
        ):
            right_angle = self.get_angle(
                points[RIGHT_SHOULDER], points[RIGHT_ELBOW], points[RIGHT_WRIST]
            )

        return left_angle > 27 or right_angle > 27

    def _is_pointing_left(self, mid_x, points, conf):
        # YOLO has no fingertip keypoints — wrist is the closest proxy
        if (
            conf[RIGHT_WRIST] < KP_CONF
            or conf[LEFT_WRIST] < KP_CONF
            or conf[LEFT_SHOULDER] < KP_CONF
        ):
            return False
        distance_left = points[LEFT_WRIST][0] - points[LEFT_SHOULDER][0]
        return points[RIGHT_WRIST][0] > mid_x or 0.28 < distance_left < 0.6

    def _is_pointing_right(self, mid_x, points, conf):
        if (
            conf[RIGHT_WRIST] < KP_CONF
            or conf[LEFT_WRIST] < KP_CONF
            or conf[RIGHT_SHOULDER] < KP_CONF
        ):
            return False
        distance_right = points[RIGHT_SHOULDER][0] - points[RIGHT_WRIST][0]
        return points[LEFT_WRIST][0] < mid_x or 0.28 < distance_right < 0.6

    def _is_raising_left_arm(self, mid_x, points, conf):
        if (
            conf[LEFT_SHOULDER] < KP_CONF
            or conf[LEFT_ELBOW] < KP_CONF
            or conf[LEFT_WRIST] < KP_CONF
        ):
            return False
        angle = self.get_angle(
            points[LEFT_SHOULDER], points[LEFT_ELBOW], points[LEFT_WRIST]
        )
        return (
            angle < 35
            and points[LEFT_WRIST][1] < points[LEFT_SHOULDER][1]
            and points[LEFT_ELBOW][1] < points[LEFT_SHOULDER][1]
        )

    def _is_raising_right_arm(self, mid_x, points, conf):
        if (
            conf[RIGHT_SHOULDER] < KP_CONF
            or conf[RIGHT_ELBOW] < KP_CONF
            or conf[RIGHT_WRIST] < KP_CONF
        ):
            return False
        angle = self.get_angle(
            points[RIGHT_SHOULDER], points[RIGHT_ELBOW], points[RIGHT_WRIST]
        )
        return (
            angle < 35
            and points[RIGHT_WRIST][1] < points[RIGHT_SHOULDER][1]
            and points[RIGHT_ELBOW][1] < points[RIGHT_SHOULDER][1]
        )

    # ── Sitting detection (YOLO pose, pixel coords) ──

    def _joint_angle_vectors(self, vec_a, vec_b):
        norm_a, norm_b = np.linalg.norm(vec_a), np.linalg.norm(vec_b)
        if norm_a == 0 or norm_b == 0:
            return 180.0
        cosine = np.clip(np.dot(vec_a, vec_b) / (norm_a * norm_b), -1.0, 1.0)
        return float(np.degrees(np.arccos(cosine)))

    def _is_sitting_side(
        self,
        points,
        scores,
        side,
        keypoints,
        keypoint_score,
        knee_max_angle,
        hip_max_angle,
    ):
        shoulder_idx = keypoints[f"shoulder_{side}"]
        hip_idx = keypoints[f"hip_{side}"]
        knee_idx = keypoints[f"knee_{side}"]
        ankle_idx = keypoints[f"ankle_{side}"]

        if min(scores[hip_idx], scores[knee_idx], scores[ankle_idx]) < keypoint_score:
            return False

        hip, knee, ankle = points[[hip_idx, knee_idx, ankle_idx]]
        torso_vec = (
            points[shoulder_idx] - hip
            if scores[shoulder_idx] >= keypoint_score
            else np.array([0.0, -1.0], dtype=np.float32)
        )

        knee_angle = self._joint_angle_vectors(hip - knee, ankle - knee)
        hip_angle = self._joint_angle_vectors(torso_vec, knee - hip)
        return knee_angle <= knee_max_angle and hip_angle <= hip_max_angle

    def _get_pose_points_scores(self, image):
        results = self.yolo_pose(image, verbose=False)
        if (
            not results
            or results[0].keypoints is None
            or results[0].keypoints.xy is None
        ):
            return None, None
        points_batch = results[0].keypoints.xy.cpu().numpy()
        scores_batch = (
            results[0].keypoints.conf.cpu().numpy()
            if results[0].keypoints.conf is not None
            else np.ones(points_batch.shape[:2], dtype=np.float32)
        )
        return points_batch, scores_batch

    def is_sitting_yolo(self, image):
        """Detect sitting pose using YOLO pose keypoints."""
        keypoint_score = 0.2
        knee_max_angle = 120.0
        hip_max_angle = 150.0
        keypoints = {
            "shoulder_l": 5,
            "shoulder_r": 6,
            "hip_l": 11,
            "hip_r": 12,
            "knee_l": 13,
            "knee_r": 14,
            "ankle_l": 15,
            "ankle_r": 16,
        }

        points_batch, scores_batch = self._get_pose_points_scores(image)
        if points_batch is None:
            return False

        for points, scores in zip(points_batch, scores_batch):
            for side in ("l", "r"):
                if self._is_sitting_side(
                    points,
                    scores,
                    side,
                    keypoints,
                    keypoint_score,
                    knee_max_angle,
                    hip_max_angle,
                ):
                    return True
        return False

    # ── Person orientation ──

    def personAngle(self, image):
        """Determine person orientation (forward/backward/left/right) using YOLO pose.
        Uses nose confidence as proxy for face visibility (replaces mediapipe z-depth)."""
        points, conf = self._get_keypoints(image)
        if points is None:
            return None

        if (
            conf[LEFT_SHOULDER] < 0.5
            or conf[RIGHT_SHOULDER] < 0.5
            or conf[LEFT_HIP] < 0.5
            or conf[RIGHT_HIP] < 0.5
        ):
            return None

        shoulder_width = np.sqrt(
            (points[LEFT_SHOULDER][0] - points[RIGHT_SHOULDER][0]) ** 2
            + (points[LEFT_SHOULDER][1] - points[RIGHT_SHOULDER][1]) ** 2
        )
        torso_height = np.sqrt(
            (points[LEFT_SHOULDER][0] - points[LEFT_HIP][0]) ** 2
            + (points[LEFT_SHOULDER][1] - points[LEFT_HIP][1]) ** 2
        )

        if torso_height == 0:
            return None

        s2t_ratio = shoulder_width / torso_height

        if s2t_ratio > 0.5:
            # Nose confidence as face visibility proxy
            orientation = "forward" if conf[NOSE] > 0.5 else "backward"
        else:
            # Side view — higher-confidence shoulder faces the camera
            orientation = (
                "left" if conf[LEFT_SHOULDER] > conf[RIGHT_SHOULDER] else "right"
            )

        return orientation

    # ── Visualization ──

    def draw_landmarks(self, image, points, conf):
        """Draw keypoints and arm connections on image."""
        if points is None:
            return
        h, w = image.shape[:2]
        draw_indices = [
            LEFT_SHOULDER,
            LEFT_ELBOW,
            LEFT_WRIST,
            RIGHT_SHOULDER,
            RIGHT_ELBOW,
            RIGHT_WRIST,
            LEFT_HIP,
            RIGHT_HIP,
        ]

        for idx in draw_indices:
            if conf[idx] > 0.5:
                x, y = int(points[idx][0] * w), int(points[idx][1] * h)
                cv2.circle(image, (x, y), 5, (0, 255, 0), -1)

        # Draw arm connections
        arms = [
            (LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST),
            (RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST),
        ]
        for s, e, wr in arms:
            if conf[s] > 0.5 and conf[e] > 0.5 and conf[wr] > 0.5:
                sp = (int(points[s][0] * w), int(points[s][1] * h))
                ep = (int(points[e][0] * w), int(points[e][1] * h))
                wp = (int(points[wr][0] * w), int(points[wr][1] * h))
                cv2.line(image, sp, ep, (0, 0, 255), 2)
                cv2.line(image, ep, wp, (0, 0, 255), 2)
                angle = self.get_angle(points[s], points[e], points[wr])
                cv2.putText(
                    image,
                    f"{int(angle)}",
                    (ep[0] - 20, ep[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    2,
                )


def main():
    pose_detection = PoseDetection()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        gesture = pose_detection.detectGesture(frame)
        points, conf = pose_detection._get_keypoints(frame)
        pose_detection.draw_landmarks(frame, points, conf)

        cv2.putText(
            frame,
            f"Gesture: {gesture.value}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        cv2.imshow("Pose and Gesture Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
