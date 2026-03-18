#!/usr/bin/env python3

import cv2
import numpy as np
import os
from frida_constants.vision_enums import Gestures
from math import degrees, acos
from ultralytics import YOLO
from vision_general.utils.trt_utils import load_yolo_trt

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

KP_CONF = 0.3


def load_yolo_pose(model_name="yolo11m-pose.pt"):
    """Load YOLO pose model with automatic TensorRT export for Orin AGX."""
    return load_yolo_trt(model_name, task="pose")


class PoseDetection:
    def __init__(self):
        print("Pose Detection Ready (YOLO TensorRT)")
        self.yolo_pose = load_yolo_pose("yolo11m-pose.pt")

    def _get_keypoints(self, image):
        """Run YOLO pose, return (points, conf) for first detected person.
        points: (17, 2) normalized, conf: (17,)."""
        results = self.yolo_pose(image, verbose=False)
        if (not results or results[0].keypoints is None or
                results[0].keypoints.xy is None or len(results[0].keypoints.xy) == 0):
            return None, None
        points = results[0].keypoints.xyn[0].cpu().numpy()
        conf = (results[0].keypoints.conf[0].cpu().numpy()
                if results[0].keypoints.conf is not None
                else np.ones(17, dtype=np.float32))
        return points, conf

    def get_angle(self, p1, p2, p3):
        p1, p2, p3 = np.array(p1[:2]), np.array(p2[:2]), np.array(p3[:2])
        l1, l2, l3 = np.linalg.norm(p2 - p3), np.linalg.norm(p1 - p3), np.linalg.norm(p1 - p2)
        denom = 2 * l1 * l2
        if denom == 0:
            return 180.0
        return abs(degrees(acos(np.clip((l1**2 + l2**2 - l3**2) / denom, -1.0, 1.0))))

    def is_chest_visible(self, image):
        points, conf = self._get_keypoints(image)
        if points is None:
            return False
        return conf[LEFT_SHOULDER] > 0.5 and conf[RIGHT_SHOULDER] > 0.5

    def are_arms_down(self, points, conf):
        if (conf[LEFT_WRIST] < KP_CONF or conf[RIGHT_WRIST] < KP_CONF or
                conf[LEFT_SHOULDER] < KP_CONF or conf[RIGHT_SHOULDER] < KP_CONF):
            return True
        left_arm_down = points[LEFT_WRIST][1] > points[LEFT_SHOULDER][1] + 0.1
        right_arm_down = points[RIGHT_WRIST][1] > points[RIGHT_SHOULDER][1] + 0.1
        return left_arm_down and right_arm_down

    def detectGesture(self, image):
        points, conf = self._get_keypoints(image)
        gestures = Gestures.UNKNOWN

        if points is None:
            return gestures
        if conf[LEFT_SHOULDER] < 0.5 or conf[RIGHT_SHOULDER] < 0.5:
            return gestures

        mid_x = (points[LEFT_SHOULDER][0] + points[RIGHT_SHOULDER][0]) / 2

        if self.are_arms_down(points, conf):
            return gestures
        elif self._is_raising_left_arm(mid_x, points, conf):
            gestures = Gestures.RAISING_LEFT_ARM
        elif self._is_pointing_left(mid_x, points, conf):
            gestures = Gestures.POINTING_LEFT
        elif self._is_raising_right_arm(mid_x, points, conf):
            gestures = Gestures.RAISING_RIGHT_ARM
        elif self._is_pointing_right(mid_x, points, conf):
            gestures = Gestures.POINTING_RIGHT
        elif self._is_waving(points, conf):
            gestures = Gestures.WAVING

        return gestures

    def _is_waving(self, points, conf):
        left_angle = 0
        if (conf[LEFT_SHOULDER] >= KP_CONF and conf[LEFT_ELBOW] >= KP_CONF and
                conf[LEFT_WRIST] >= KP_CONF):
            left_angle = self.get_angle(points[LEFT_SHOULDER], points[LEFT_ELBOW], points[LEFT_WRIST])
        right_angle = 0
        if (conf[RIGHT_SHOULDER] >= KP_CONF and conf[RIGHT_ELBOW] >= KP_CONF and
                conf[RIGHT_WRIST] >= KP_CONF):
            right_angle = self.get_angle(points[RIGHT_SHOULDER], points[RIGHT_ELBOW], points[RIGHT_WRIST])
        return left_angle > 27 or right_angle > 27

    def _is_pointing_left(self, mid_x, points, conf):
        if (conf[RIGHT_WRIST] < KP_CONF or conf[LEFT_WRIST] < KP_CONF or
                conf[LEFT_SHOULDER] < KP_CONF):
            return False
        distance_left = points[LEFT_WRIST][0] - points[LEFT_SHOULDER][0]
        return points[RIGHT_WRIST][0] > mid_x or 0.28 < distance_left < 0.6

    def _is_pointing_right(self, mid_x, points, conf):
        if (conf[RIGHT_WRIST] < KP_CONF or conf[LEFT_WRIST] < KP_CONF or
                conf[RIGHT_SHOULDER] < KP_CONF):
            return False
        distance_right = points[RIGHT_SHOULDER][0] - points[RIGHT_WRIST][0]
        return points[LEFT_WRIST][0] < mid_x or 0.28 < distance_right < 0.6

    def _is_raising_left_arm(self, mid_x, points, conf):
        if (conf[LEFT_SHOULDER] < KP_CONF or conf[LEFT_ELBOW] < KP_CONF or
                conf[LEFT_WRIST] < KP_CONF):
            return False
        angle = self.get_angle(points[LEFT_SHOULDER], points[LEFT_ELBOW], points[LEFT_WRIST])
        distance_left = points[LEFT_SHOULDER][1] - points[LEFT_WRIST][1]
        return (angle < 27 and
                points[LEFT_WRIST][1] < points[LEFT_SHOULDER][1] and
                points[LEFT_ELBOW][1] < points[LEFT_SHOULDER][1] and
                distance_left > 0.25)

    def _is_raising_right_arm(self, mid_x, points, conf):
        if (conf[RIGHT_SHOULDER] < KP_CONF or conf[RIGHT_ELBOW] < KP_CONF or
                conf[RIGHT_WRIST] < KP_CONF):
            return False
        angle = self.get_angle(points[RIGHT_SHOULDER], points[RIGHT_ELBOW], points[RIGHT_WRIST])
        distance_right = points[RIGHT_SHOULDER][1] - points[RIGHT_WRIST][1]
        return (angle < 27 and
                points[RIGHT_WRIST][1] < points[RIGHT_SHOULDER][1] and
                points[RIGHT_ELBOW][1] < points[RIGHT_SHOULDER][1] and
                distance_right > 0.25)


def main():
    pose_detection = PoseDetection()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gesture = pose_detection.detectGesture(frame)
        cv2.putText(frame, f"Gesture: {gesture.value}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Pose and Gesture Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
