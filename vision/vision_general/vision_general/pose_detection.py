#!/usr/bin/env python3

import cv2
import mediapipe as mp
import numpy as np

import os  # For testing


class PoseDetection:
    def __init__(self):
        print("Pose Detection Ready")
        # rospy.init_node('pose_detection')
        # rospy.loginfo("Pose Detection Ready")

        # Initialize MediaPipe Pose as a class attribute
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()
        self.mp_drawing = mp.solutions.drawing_utils  # For visualizing landmarks

    def detectPose(self):
        pass

    def detectGesture(self):
        pass

    def detectClothes(self):
        pass

    def isChestVisible(self, image_path):
        # Load and preprocess the image
        image = cv2.imread(image_path)
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

    def chestPosition(self, image_path, save_image=False):
        # Load and preprocess the image
        image = cv2.imread(image_path)
        if image is None:
            print("Error: Could not load image.")
            return None

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

    def personAngle(self, image_path):
        # Load and preprocess the image
        image = cv2.imread(image_path)
        if image is None:
            print("Error: Could not load image.")
            return None

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
                # print(f'S2T ratio: {s2t_ratio:.2f}')
                # print(f'nose: {nose.visibility}')

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
                    orientation = "unknown"

                # Optionally, draw landmarks on the image and save
                # annotated_image = image.copy()
                # self.mp_drawing.draw_landmarks(annotated_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
                # cv2.imwrite(f"../Utils/tests/person_angle.jpg", annotated_image)

                return orientation

        print("Pose landmarks not detected or not sufficiently visible.")
        return None


def main():
    # image_path = "./testImages/image4.jpg"

    # pose_detection = PoseDetection()

    # print(pose_detection.isChestVisible(image_path=image_path))

    # chest_coords = pose_detection.chestPosition(image_path=image_path, save_image=True)
    # if chest_coords:
    #     print(f"Chest coordinates: {chest_coords}")

    # angle = pose_detection.personAngle(image_path=image_path)
    # if angle:
    #     print(f"Person angle: {angle:.2f} degrees")

    test_images_dir = "../Utils/angle_test_images"
    pose_detection = PoseDetection()

    for i in range(1, 20):
        image_name = f"{i}.jpeg"
        image_path = os.path.join(test_images_dir, image_name)
        angle = pose_detection.personAngle(image_path=image_path)
        print(f"{i} {angle}")


if __name__ == "__main__":
    main()
