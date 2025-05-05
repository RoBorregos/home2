#!/usr/bin/env python3

import cv2
import mediapipe as mp
import numpy as np
from frida_constants.vision_enums import Poses, Gestures
from math import degrees, acos, sqrt
import time


class PoseDetection:
    def __init__(self):
        print("Pose Detection Ready")
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False, min_detection_confidence=0.5
        )
        self.mp_hands = mp.solutions.hands

    def getCenterPerson(self, image):
        # Process the image
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)

        if results.pose_landmarks is not None:
            landmarks = results.pose_landmarks.landmark
            shoulder_right = landmarks[12]
            shoulder_left = landmarks[11]

            x_center = (shoulder_right.x + shoulder_left.x) / 2
            y_center = (shoulder_right.y + shoulder_left.y) / 2

            cv2.circle(
                image,
                (int(x_center * image.shape[1]), int(y_center * image.shape[0])),
                5,
                (0, 0, 255),
                -1,
            )
            # cv2.imshow("Annotated Image", image)

            return x_center * image.shape[1], y_center * image.shape[0]

        return None, None

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
            self.mp_pose.PoseLandmark.LEFT_KNEE,
            self.mp_pose.PoseLandmark.RIGHT_KNEE,
            self.mp_pose.PoseLandmark.LEFT_ANKLE,
            self.mp_pose.PoseLandmark.RIGHT_ANKLE,
        ]

        for landmark in landmarks_to_draw:
            landmark_data = results.pose_landmarks.landmark[landmark]
            if landmark_data.visibility > 0.5:
                x, y = (
                    int(landmark_data.x * image_width),
                    int(landmark_data.y * image_height),
                )
                cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
        self.draw_connections(image, results, mp_pose)
        self.draw_hand_landmarks(image)

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
            (
                mp_pose.PoseLandmark.LEFT_HIP,
                mp_pose.PoseLandmark.LEFT_KNEE,
                mp_pose.PoseLandmark.LEFT_ANKLE,
            ),
            (
                mp_pose.PoseLandmark.RIGHT_HIP,
                mp_pose.PoseLandmark.RIGHT_KNEE,
                mp_pose.PoseLandmark.RIGHT_ANKLE,
            ),
        ]
        for start_idx, mid_idx, end_idx in connections:
            self.draw_line_and_angle(image, results, start_idx, mid_idx, end_idx)

    def draw_line_and_angle(self, image, results, start_idx, mid_idx, end_idx):
        start, mid, end = (
            results.pose_landmarks.landmark[start_idx],
            results.pose_landmarks.landmark[mid_idx],
            results.pose_landmarks.landmark[end_idx],
        )
        if start.visibility > 0.5 and mid.visibility > 0.5 and end.visibility > 0.5:
            start_point = (int(start.x * image.shape[1]), int(start.y * image.shape[0]))
            mid_point = (int(mid.x * image.shape[1]), int(mid.y * image.shape[0]))
            end_point = (int(end.x * image.shape[1]), int(end.y * image.shape[0]))
            cv2.line(image, start_point, mid_point, (0, 0, 255), 2)
            cv2.line(image, mid_point, end_point, (0, 0, 255), 2)
            angle = self.get_angle(start, mid, end)
            cv2.putText(
                image,
                f"{int(angle)}",
                (mid_point[0] - 20, mid_point[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                2,
            )

    def draw_hand_landmarks(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                for landmark in hand_landmarks.landmark:
                    x, y = (
                        int(landmark.x * image.shape[1]),
                        int(landmark.y * image.shape[0]),
                    )
                    cv2.circle(image, (x, y), 5, (255, 0, 0), -1)

                self.mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing.DrawingSpec(
                        color=(0, 0, 255), thickness=2, circle_radius=4
                    ),
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
        return all(landmarks[idx].visibility > 0.5 for idx in indices)

    def are_legs_visible(self, landmarks):
        # Check visibility of knees and ankles
        left_knee = landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE]
        right_knee = landmarks[self.mp_pose.PoseLandmark.RIGHT_KNEE]

        return left_knee.visibility > 0.5 and right_knee.visibility > 0.5

    def detectPose(self, image, return_results=False):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        pose_result = Poses.UNKNOWN
        left_knee_angle = right_knee_angle = 0

        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            if not self.are_legs_visible(landmarks):
                return pose_result, None
            left_knee_angle, right_knee_angle = self.get_leg_angles(landmarks)

            # Debug: print angles
            # print(f"Left knee angle: {left_knee_angle}, Right knee angle: {right_knee_angle}")

            # Standing: legs extended and torso is vertical
            if (
                left_knee_angle < 2.2
                and right_knee_angle < 2.2
                and not self.is_body_horizontal(landmarks)
            ):
                pose_result = Poses.STANDING

            # Lying down: legs extended and torso is horizontal
            elif self.is_body_horizontal(landmarks):
                pose_result = Poses.LYING_DOWN

            # Sitting: knees are bent, above the ankles, bent in depth, and close to hip
            elif (
                left_knee_angle > 2.2
                and right_knee_angle > 2.2
                and self.is_knee_above_ankles(landmarks)
            ):
                pose_result = Poses.SITTING

        if pose_result != Poses.UNKNOWN:
            print(f"Pose detected: {pose_result.value}")
            time.sleep(0.5)
        return (pose_result, results) if return_results else pose_result

    def is_knee_bent_in_depth(self, landmarks):
        left_knee = landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE]
        left_ankle = landmarks[self.mp_pose.PoseLandmark.LEFT_ANKLE]
        right_knee = landmarks[self.mp_pose.PoseLandmark.RIGHT_KNEE]
        right_ankle = landmarks[self.mp_pose.PoseLandmark.RIGHT_ANKLE]

        return left_knee.z < left_ankle.z - 0.1 and right_knee.z < right_ankle.z - 0.1

    def is_knee_close_to_hip(self, landmarks):
        left_knee = landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE]
        left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
        right_knee = landmarks[self.mp_pose.PoseLandmark.RIGHT_KNEE]
        right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]

        left_dist = abs(left_knee.y - left_hip.y)
        right_dist = abs(right_knee.y - right_hip.y)

        return left_dist < 0.15 and right_dist < 0.15  # Ajusta el umbral según tu caso

    def is_knee_above_ankles(self, landmarks):
        left_knee = landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE]
        right_knee = landmarks[self.mp_pose.PoseLandmark.RIGHT_KNEE]
        left_ankle = landmarks[self.mp_pose.PoseLandmark.LEFT_ANKLE]
        right_ankle = landmarks[self.mp_pose.PoseLandmark.RIGHT_ANKLE]

        return left_knee.y < left_ankle.y and right_knee.y < right_ankle.y

    def get_leg_angles(self, landmarks):
        # Get keypoints for each leg
        left_ankle = landmarks[self.mp_pose.PoseLandmark.LEFT_ANKLE]
        left_knee = landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE]
        left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
        right_ankle = landmarks[self.mp_pose.PoseLandmark.RIGHT_ANKLE]
        right_knee = landmarks[self.mp_pose.PoseLandmark.RIGHT_KNEE]
        right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]

        # Compute angle at each knee
        left_knee_angle = self.get_angle(left_ankle, left_knee, left_hip)
        right_knee_angle = self.get_angle(right_ankle, right_knee, right_hip)
        return left_knee_angle, right_knee_angle

    def is_body_horizontal(self, landmarks):
        # Average Y positions of shoulders and hips
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]

        avg_shoulder_y = (left_shoulder.y + right_shoulder.y) / 2
        avg_hip_y = (left_hip.y + right_hip.y) / 2

        # If shoulders and hips are at similar vertical levels, assume lying down
        return abs(avg_shoulder_y - avg_hip_y) < 0.1

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
        results_h = self.hands.process(image_rgb)
        results_p = self.pose.process(image_rgb)

        gesture = Gestures.UNKNOWN

        if results_p.pose_landmarks:
            if not self.is_chest_visible(image) or self.are_arms_down(
                results_p.pose_landmarks
            ):
                print("Chest not visible or arms down")
                return gesture
            elif self.is_raising_left_arm(results_p.pose_landmarks):
                gesture = Gestures.RAISING_LEFT_ARM
            elif self.is_raising_right_arm(results_p.pose_landmarks):
                gesture = Gestures.RAISING_RIGHT_ARM

        if gesture != Gestures.UNKNOWN:
            print(f"Detected gesture: {gesture.value}")
            time.sleep(0.5)
            return gesture

        if results_h.multi_hand_landmarks:
            for hand_landmarks in results_h.multi_hand_landmarks:
                # Ensure pose landmarks exist for validation
                if results_p.pose_landmarks:
                    pose_landmarks = results_p.pose_landmarks.landmark
                    mid_x = self.get_midpoint_x(pose_landmarks)

                    # Identify the hand based on landmarks and arm position
                    wrist = hand_landmarks.landmark[
                        mp.solutions.hands.HandLandmark.WRIST
                    ]

                    # Check if wrist is closer to left or right shoulder based on y-axis of wrist and shoulder
                    if self.is_closer_to_left_shoulder(
                        wrist,
                        pose_landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER],
                        pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER],
                    ):  # Left hand
                        # Check angle for left hand
                        angle = self.get_angle(
                            pose_landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER],
                            pose_landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW],
                            pose_landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST],
                        )

                        print(f"Left hand angle: {angle}")
                        # time.sleep(0.5)
                        # Check gestures for the left hand
                        if self.is_waving(hand_landmarks) or angle > 45:
                            gesture = Gestures.WAVING
                            break
                        elif self.is_pointing_left(
                            hand_landmarks, mid_x, pose_landmarks
                        ):
                            gesture = Gestures.POINTING_LEFT
                            break
                    else:  # Right hand
                        # Check angle for right hand
                        angle = self.get_angle(
                            pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER],
                            pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW],
                            pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST],
                        )
                        print(f"Right hand angle: {angle}")
                        # time.sleep(0.5)
                        # Check gestures for the right hand
                        if self.is_waving(hand_landmarks) or angle > 45:
                            gesture = Gestures.WAVING
                            break
                        elif self.is_pointing_right(
                            hand_landmarks, mid_x, pose_landmarks
                        ):
                            gesture = Gestures.POINTING_RIGHT
                            break

        if gesture != gesture.UNKNOWN:
            print(f"Detected gesture: {gesture.value}")
            time.sleep(0.5)
        return gesture

    def is_closer_to_left_shoulder(self, wrist, left_shoulder, right_shoulder):
        return abs(wrist.x - left_shoulder.x) < abs(wrist.x - right_shoulder.x)

    def is_waving(self, hand_landmarks):
        thumb = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.THUMB_TIP]
        pinky = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.PINKY_TIP]

        # Calculate the Euclidean distance between the thumb and pinky
        distance = sqrt((thumb.x - pinky.x) ** 2 + (thumb.y - pinky.y) ** 2)

        # Print the distance for debugging purposes
        print(f"Distance between thumb and pinky: {distance}")

        # Set a threshold based on the expected distance for a waving gesture
        waving_threshold = 0.13

        return distance > waving_threshold

    def get_midpoint_x(self, pose_landmarks):
        left_shoulder = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        return (left_shoulder.x + right_shoulder.x) / 2

    def is_hand_left_of_midline(self, index_finger, mid_x):
        """Checks if the index finger is to the left of the midline."""
        return index_finger.x < mid_x

    def is_hand_right_of_midline(self, index_finger, mid_x):
        """Checks if the index finger is to the right of the midline."""
        return index_finger.x > mid_x

    def is_pointing_left(self, hand_landmarks, mid_x, pose_landmarks):
        """Detects if the hand is pointing left across the chest."""
        index_finger = hand_landmarks.landmark[
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP
        ]

        distance_elbow_hip = 0

        if self.is_visible(
            pose_landmarks,
            [self.mp_pose.PoseLandmark.LEFT_HIP, self.mp_pose.PoseLandmark.LEFT_ELBOW],
        ):
            right_hip = pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_HIP]
            right_elbow = pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_ELBOW]

            # distance between elbow and hip
            distance_elbow_hip = abs(right_elbow.y - right_hip.y)
            # print(f"distance right: {distance_elbow_hip}")

        return (
            self.is_hand_right_of_midline(index_finger, mid_x)
            or distance_elbow_hip > 0.5
        )

    def is_pointing_right(self, hand_landmarks, mid_x, pose_landmarks):
        """Detects if the hand is pointing right across the chest."""
        index_finger = hand_landmarks.landmark[
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP
        ]

        distance_elbow_hip = 0
        # check if hip and elbow are visible
        if self.is_visible(
            pose_landmarks,
            [self.mp_pose.PoseLandmark.LEFT_HIP, self.mp_pose.PoseLandmark.LEFT_ELBOW],
        ):
            left_hip = pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_HIP]
            left_elbow = pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_ELBOW]

            # distance between elbow and hip
            distance_elbow_hip = abs(left_elbow.y - left_hip.y)
            # print(f"distance right: {distance_elbow_hip}")

        return (
            self.is_hand_left_of_midline(index_finger, mid_x)
            or distance_elbow_hip > 0.5
        )

    def is_raising_left_arm(self, pose_landmarks):
        left_elbow = pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_ELBOW]
        left_shoulder = pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        left_wrist = pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST]
        left_hip = pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_HIP]

        angle = self.get_angle(left_shoulder, left_elbow, left_wrist)

        # Distance between shoulder and elbow
        # distance_shoulder_elbow = abs(left_shoulder.y - left_elbow.y)
        # Distance between elbow and hip
        distance_elbow_hip = abs(left_elbow.y - left_hip.y)
        # print(f"angle: {angle}")
        # print(f"right wrist: {left_wrist.y < left_shoulder.y}")
        # print(f"right elbow: {left_elbow.y < left_shoulder.y}")
        # print(f"distance left : {distance_elbow_hip}")

        if (
            angle < 30
            and left_wrist.y < left_shoulder.y
            and left_elbow.y < left_shoulder.y
            and distance_elbow_hip > 0.85
        ):
            return True

        return False

    def is_raising_right_arm(self, pose_landmarks):
        right_elbow = pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_ELBOW]
        right_shoulder = pose_landmarks.landmark[
            self.mp_pose.PoseLandmark.RIGHT_SHOULDER
        ]
        right_wrist = pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST]
        right_hip = pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_HIP]

        angle = self.get_angle(right_shoulder, right_elbow, right_wrist)

        # Distance between shoulder and elbow
        # distance_shoulder_elbow = abs(right_shoulder.y - right_elbow.y)
        # Distance between elbow and hip
        distance_elbow_hip = abs(right_elbow.y - right_hip.y)
        # print(f"angle: {angle}")
        # print(f"right wrist: {right_wrist.y < right_shoulder.y}")
        # print(f"right elbow: {right_elbow.y < right_shoulder.y}")
        # print(f"distance right: {distance_elbow_hip}")

        if (
            angle < 30
            and right_wrist.y < right_shoulder.y
            and right_elbow.y < right_shoulder.y
            and distance_elbow_hip > 0.85
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

        pose_detection.detectPose(frame, return_results=False)
        pose_detection.detectGesture(frame)

    #     if results and results.pose_landmarks:
    #         pose_detection.draw_landmarks(frame, results, pose_detection.mp_pose)

    #     cv2.putText(
    #         frame,
    #         f"Pose: {pose.value}",
    #         (10, 30),
    #         cv2.FONT_HERSHEY_SIMPLEX,
    #         0.8,
    #         (0, 255, 0),
    #         2,
    #     )
    #     cv2.putText(
    #         frame,
    #         f"Gesture: {gesture.value}",
    #         (10, 60),
    #         cv2.FONT_HERSHEY_SIMPLEX,
    #         0.8,
    #         (0, 255, 0),
    #         2,
    #     )

    # cv2.imshow("Pose and Gesture Detection", frame)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
