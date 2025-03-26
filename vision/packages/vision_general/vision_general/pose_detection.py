#!/usr/bin/env python3

import cv2
import mediapipe as mp
import numpy as np


class PoseDetection:
    def __init__(self):
        print("Pose Detection Ready")
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()
        self.mp_drawing = mp.solutions.drawing_utils

    def detectPose(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
            right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]
            left_knee = landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE]
            right_knee = landmarks[self.mp_pose.PoseLandmark.RIGHT_KNEE]
            left_ankle = landmarks[self.mp_pose.PoseLandmark.LEFT_ANKLE]
            right_ankle = landmarks[self.mp_pose.PoseLandmark.RIGHT_ANKLE]

            if left_hip.y < left_knee.y and right_hip.y < right_knee.y:
                return "Standing"
            elif left_hip.y > left_knee.y and right_hip.y > right_knee.y:
                return "Sitting"
            elif left_knee.y > left_ankle.y and right_knee.y > right_ankle.y:
                return "Lying Down"
        return "Unknown Pose"

    def detectGesture(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            left_wrist = landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST]
            right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]
            left_elbow = landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW]
            right_elbow = landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW]
            left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]

            if left_wrist.y < left_shoulder.y and right_wrist.y < right_shoulder.y:
                return "Waving"
            elif left_wrist.y < left_shoulder.y:
                return "Raising Left Arm"
            elif right_wrist.y < right_shoulder.y:
                return "Raising Right Arm"
            elif left_wrist.x < left_elbow.x and left_elbow.x < left_shoulder.x:
                return "Pointing Left"
            elif right_wrist.x > right_elbow.x and right_elbow.x > right_shoulder.x:
                return "Pointing Right"
        return "No Gesture"

    def detectClothes(self):
        pass

    def isChestVisible(self, image):
        # Preprocess the image
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Process the image
        results = self.pose.process(image_rgb)

        # Check for landmarks
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            # Get key points for shoulders and chest (sternum approximate region)
            left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]

            # Check visibility and positioning
            if left_shoulder.visibility > 0.5 and right_shoulder.visibility > 0.5:
                print("Chest is visible.")
                return True
            else:
                print("Chest is not fully visible.")
        else:
            print("No pose detected.")
        return False

    def chestPosition(self, image, save_image=False):
        # Preprocess the image
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Process the image to detect pose landmarks
        results = self.pose.process(image_rgb)

        # Check for landmarks
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]

            # Approximate chest region as below the nose and between shoulders
            if left_shoulder.visibility > 0.5 and right_shoulder.visibility > 0.5:
                chest_x = int((left_shoulder.x + right_shoulder.x) / 2 * image.shape[1])
                chest_y = int((left_shoulder.y + right_shoulder.y) / 2 * image.shape[0])

                # Save the image with the chest center marked
                if save_image:
                    # Draw a circle at the approximated chest position
                    cv2.circle(image, (chest_x, chest_y), 10, (255, 0, 0), -1)
                    cv2.imwrite("./testImages/chest_position.jpg", image)

                return (chest_x, chest_y)

        print("Chest landmarks not detected or not fully visible.")
        return None

    def personAngle(self, image, save_image=False):
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

                # Optionally, draw landmarks on the image and save
                if save_image:
                    annotated_image = image.copy()
                    self.mp_drawing.draw_landmarks(
                        annotated_image,
                        results.pose_landmarks,
                        self.mp_pose.POSE_CONNECTIONS,
                    )
                    cv2.imwrite("../Utils/tests/person_angle.jpg", annotated_image)

                return orientation

        return None


def main():
    pose_detection = PoseDetection()

    img = cv2.imread("../Utils/pose_test_images/standing3.jpg")
    pose = pose_detection.detectPose(img, save_image=True)
    print(f"Detected pose: {pose}")


if __name__ == "__main__":
    main()
