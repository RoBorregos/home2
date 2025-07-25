#!/usr/bin/env python3

import cv2
import mediapipe as mp
import numpy as np
from frida_constants.vision_enums import Gestures
from math import degrees, acos
import time


class PoseDetection:
    def __init__(self):
        print("Pose Detection Ready")
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()
        self.mp_drawing = mp.solutions.drawing_utils

    def detect(self, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb_frame)
        return results

    def draw_landmarks(self, image, results, mp_pose):
        image_height, image_width, _ = image.shape
        landmarks_to_draw = [
            self.mp_pose.PoseLandmark.LEFT_SHOULDER,
            self.mp_pose.PoseLandmark.LEFT_ELBOW,
            self.mp_pose.PoseLandmark.LEFT_WRIST,
            self.mp_pose.PoseLandmark.RIGHT_SHOULDER,
            self.mp_pose.PoseLandmark.RIGHT_ELBOW,
            self.mp_pose.PoseLandmark.RIGHT_WRIST,
            self.mp_pose.PoseLandmark.LEFT_HIP,
            self.mp_pose.PoseLandmark.RIGHT_HIP,
        ]

        for landmark in landmarks_to_draw:
            if results.pose_landmarks is not None:
                landmark_data = results.pose_landmarks.landmark[landmark]
                if landmark_data.visibility > 0.5:
                    x, y = (
                        int(landmark_data.x * image_width),
                        int(landmark_data.y * image_height),
                    )
                    cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
            self.draw_connections(image, results, mp_pose)

    def draw_connections(self, image, results, mp_pose):
        # Define relevant connections (Shoulders -> Elbows -> Wrists and Hips -> Knees -> Ankles)
        connections = [
            (
                mp_pose.PoseLandmark.LEFT_SHOULDER,
                mp_pose.PoseLandmark.LEFT_ELBOW,
                mp_pose.PoseLandmark.LEFT_WRIST,
            ),
            (
                mp_pose.PoseLandmark.RIGHT_SHOULDER,
                mp_pose.PoseLandmark.RIGHT_ELBOW,
                mp_pose.PoseLandmark.RIGHT_WRIST,
            ),
        ]
        for start_idx, mid_idx, end_idx in connections:
            self.draw_line_and_angle(image, results, start_idx, mid_idx, end_idx)

    def draw_line_and_angle(self, image, results, start_idx, mid_idx, end_idx):
        if results.pose_landmarks:
            start, mid, end = (
                results.pose_landmarks.landmark[start_idx],
                results.pose_landmarks.landmark[mid_idx],
                results.pose_landmarks.landmark[end_idx],
            )
            if start.visibility > 0.5 and mid.visibility > 0.5 and end.visibility > 0.5:
                start_point = (
                    int(start.x * image.shape[1]),
                    int(start.y * image.shape[0]),
                )
                mid_point = (int(mid.x * image.shape[1]), int(mid.y * image.shape[0]))
                end_point = (int(end.x * image.shape[1]), int(end.y * image.shape[0]))
                cv2.line(image, start_point, mid_point, (0, 0, 255), 2)
                cv2.line(image, mid_point, end_point, (0, 0, 255), 2)
                angle = self.get_angle(start, mid, end)
                cv2.putText(
                    image,
                    f"{int(angle)} - {start_idx} {end_idx}",
                    (mid_point[0] - 20, mid_point[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    2,
                )

    def get_angle(self, p1, p2, p3):
        p1, p2, p3 = (
            np.array([p1.x, p1.y]),
            np.array([p2.x, p2.y]),
            np.array([p3.x, p3.y]),
        )
        l1, l2, l3 = (
            np.linalg.norm(p2 - p3),
            np.linalg.norm(p1 - p3),
            np.linalg.norm(p1 - p2),
        )
        return abs(degrees(acos((l1**2 + l2**2 - l3**2) / (2 * l1 * l2))))

    def is_visible(self, landmarks, indices):
        return all(landmarks.landmark[idx].visibility > 0.5 for idx in indices)

    def is_chest_visible(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        if not results.pose_landmarks:
            return False

        landmarks = results.pose_landmarks.landmark

        # Extract required landmarks using the preferred format
        required = [
            landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER],
            landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER],
        ]

        # Check if all landmarks are confidently visible (threshold: 0.5)
        return all(lm.visibility > 0.5 for lm in required)

    def are_arms_down(self, pose_landmarks):
        left_wrist = pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST]
        right_wrist = pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST]
        left_shoulder = pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = pose_landmarks.landmark[
            self.mp_pose.PoseLandmark.RIGHT_SHOULDER
        ]

        # If both wrists are below their respective shoulders by a significant margin, arms are considered down
        left_arm_down = left_wrist.y > left_shoulder.y + 0.1
        right_arm_down = right_wrist.y > right_shoulder.y + 0.1

        return left_arm_down and right_arm_down

    def detectGesture(self, image):
        # Detect hand gestures using mediapipe hands
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results_p = self.pose.process(image_rgb)

        gestures = Gestures.UNKNOWN

        if results_p.pose_landmarks:
            print("FOUND LANDMARKS")
            mid_x = self.get_midpoint_x(results_p)

            if not self.is_chest_visible(image) or self.are_arms_down(
                results_p.pose_landmarks
            ):
                print(f"Detected gesture: {gestures}")
                return gestures

            # Left hand
            elif self.is_raising_left_arm(mid_x, results_p):
                gestures = Gestures.RAISING_LEFT_ARM

            elif self.is_pointing_left(mid_x, results_p):
                gestures = Gestures.POINTING_LEFT

            # Right hand
            elif self.is_raising_right_arm(mid_x, results_p):
                gestures = Gestures.RAISING_RIGHT_ARM

            elif self.is_pointing_right(mid_x, results_p):
                gestures = Gestures.POINTING_RIGHT

            elif self.is_waving(results_p):
                gestures = Gestures.WAVING

        print(f"Detected gesture: {gestures}")
        return gestures

    def is_waving_customer(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        try:
            landmarks = results.pose_landmarks.landmark

            left_shoulder = landmarks[11]  # mp_pose.PoseLandmark.LEFT_SHOULDER
            # left_elbow = landmarks[13]  # mp_pose.PoseLandmark.LEFT_ELBOW
            left_wrist = landmarks[15]  # mp_pose.PoseLandmark.LEFT_WRIST

            right_shoulder = landmarks[12]  # mp_pose.PoseLandmark.RIGHT_SHOULDER
            # right_elbow = landmarks[14]  # mp_pose.PoseLandmark.RIGHT_ELBOW
            right_wrist = landmarks[16]  # mp_pose.PoseLandmark.RIGHT_WRIST

            if right_wrist.y < right_shoulder.y and left_wrist.y > left_shoulder.y:
                return True
            if left_wrist.y < left_shoulder.y and right_wrist.y > right_shoulder.y:
                return True
            return False
        except Exception:
            return False

    def is_closer_to_left_shoulder(self, wrist, left_shoulder, right_shoulder):
        return abs(wrist.x - left_shoulder.x) < abs(wrist.x - right_shoulder.x)

    def is_waving(self, results):
        landmarks = results.pose_landmarks.landmark

        left_shoulder = landmarks[11]  # mp_pose.PoseLandmark.LEFT_SHOULDER
        left_elbow = landmarks[13]  # mp_pose.PoseLandmark.LEFT_ELBOW
        left_wrist = landmarks[15]  # mp_pose.PoseLandmark.LEFT_WRIST

        right_shoulder = landmarks[12]  # mp_pose.PoseLandmark.RIGHT_SHOULDER
        right_elbow = landmarks[14]  # mp_pose.PoseLandmark.RIGHT_ELBOW
        right_wrist = landmarks[16]  # mp_pose.PoseLandmark.RIGHT_WRIST

        angle_r = self.get_angle(right_shoulder, right_elbow, right_wrist)

        angle_l = self.get_angle(left_shoulder, left_elbow, left_wrist)

        if (
            angle_l > 27
            # and left_wrist.y < left_shoulder.y
        ):
            return True

        elif (
            angle_r > 27
            # and right_wrist.y < right_shoulder.y
        ):
            return True

    def get_midpoint_x(self, results):
        landmarks = results.pose_landmarks.landmark

        left_shoulder = landmarks[11]  # mp_pose.PoseLandmark.LEFT_SHOULDER
        right_shoulder = landmarks[12]  # mp_pose.PoseLandmark.RIGHT_SHOULDER

        mid = (left_shoulder.x + right_shoulder.x) / 2

        return mid

    def is_pointing_left(self, mid_x, results):
        """Detects if the hand is pointing left across the chest."""
        landmarks = results.pose_landmarks.landmark

        right_index = landmarks[20]

        left_shoulder = landmarks[11]
        left_index = landmarks[19]

        distance_left = left_index.x - left_shoulder.x

        if right_index.x > mid_x or 0.28 < distance_left < 0.6:
            return True

    def is_pointing_right(self, mid_x, results):
        """Detects if the hand is pointing right across the chest."""
        landmarks = results.pose_landmarks.landmark

        right_shoulder = landmarks[12]
        right_index = landmarks[20]

        left_index = landmarks[19]

        distance_right = right_shoulder.x - right_index.x

        if left_index.x < mid_x or 0.28 < distance_right < 0.6:
            return True

    def is_raising_left_arm(self, mid_x, results):
        landmarks = results.pose_landmarks.landmark

        left_shoulder = landmarks[11]  # mp_pose.PoseLandmark.LEFT_SHOULDER
        left_elbow = landmarks[13]  # mp_pose.PoseLandmark.LEFT_ELBOW
        left_wrist = landmarks[15]  # mp_pose.PoseLandmark.LEFT_WRIST
        # left_index = landmarks[19]

        angle = self.get_angle(left_shoulder, left_elbow, left_wrist)

        if (
            angle < 35
            and left_wrist.y < left_shoulder.y
            and left_elbow.y < left_shoulder.y
        ):
            return True

        return False

    def is_raising_right_arm(self, mid_x, results):
        landmarks = results.pose_landmarks.landmark

        right_shoulder = landmarks[12]  # mp_pose.PoseLandmark.RIGHT_SHOULDER
        right_elbow = landmarks[14]  # mp_pose.PoseLandmark.RIGHT_ELBOW
        right_wrist = landmarks[16]  # mp_pose.PoseLandmark.RIGHT_WRIST
        # right_index = landmarks[20]

        angle = self.get_angle(right_shoulder, right_elbow, right_wrist)

        if (
            angle < 35
            and right_wrist.y < right_shoulder.y
            and right_elbow.y < right_shoulder.y
        ):
            return True

        return False

    def personAngle(self, image):
        # Preprocess the image
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Process the image to detect pose landmarks
        results = self.pose.process(image_rgb)

        # Check if pose landmarks are detected
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            # Extract the key landmarks: shoulders, hips, and nose
            left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
            right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]
            nose = landmarks[self.mp_pose.PoseLandmark.NOSE]

            # Ensure all key landmarks have sufficient visibility
            if (
                left_shoulder.visibility > 0.5
                and right_shoulder.visibility > 0.5
                and left_hip.visibility > 0.5
                and right_hip.visibility > 0.5
            ):
                # Calculate shoulder width and torso height
                shoulder_width = np.sqrt(
                    (left_shoulder.x - right_shoulder.x) ** 2
                    + (left_shoulder.y - right_shoulder.y) ** 2
                )
                torso_height = np.sqrt(
                    (left_shoulder.x - left_hip.x) ** 2
                    + (left_shoulder.y - left_hip.y) ** 2
                )

                # Handle cases where the torso height is zero
                if torso_height == 0:
                    print("Error: Torso height is zero.")
                    time.sleep(0.5)
                    return None

                # Calculate S2T ratio
                s2t_ratio = shoulder_width / torso_height

                # Determine orientation based on heuristic thresholds
                if s2t_ratio > 0.5:  # Forward or backward
                    if nose.z < 0:  # Face is visible
                        orientation = "forward"
                    else:  # Face is not visible
                        orientation = "backward"
                elif s2t_ratio <= 0.5:  # Side views
                    # Check which side is closer
                    if left_shoulder.z < right_shoulder.z:
                        orientation = "left"
                    else:
                        orientation = "right"
                else:
                    orientation = None

                return orientation

        return None


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

        gesture, results = pose_detection.detectGesture(frame)

        pose_detection.draw_landmarks(frame, results, pose_detection.mp_pose)

        cv2.putText(
            frame,
            f"Gesture: {[g.value for g in gesture]}",
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
